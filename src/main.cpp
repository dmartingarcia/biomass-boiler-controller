#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>
#include <esp_system.h>

// Include project header files
#include "config.h"
#include "temperature_sensors.h"
#include "relay.h"
#include "air_intake.h"
#include "display.h"
#include "log_buffer.h"
#include "network_manager.h"
#include "home_assistant.h"
#include "fs_helper.h"
#include "sensor_config.h"
#include "runtime_config.h"
#include "history.h"
#include "relay_config.h"

// Creación del servidor web directamente en main.cpp
AsyncWebServer webServer(WEB_SERVER_PORT);

// Declaration of global objects
SensorConfig sensorConfig;
TemperatureSensors sensors(&sensorConfig);

// Individual relay instances
Relay boilerPumpRelay(RELAY_BOILER_PUMP, "Boiler Pump");
Relay heatingPumpRelay(RELAY_HEATING_PUMP, "Heating Pump");
Relay fansRelay(RELAY_FANS, "Fans");
Relay otherRelay(RELAY_OTHER, "Other");

// Which physical channel drives each function (editable from the UI)
RelayConfig relayConfig;
Relay* const relayByRole[RELAY_ROLE_COUNT] = { &boilerPumpRelay, &heatingPumpRelay, &fansRelay, &otherRelay };

AirIntake airIntake;
Display display;
LogBuffer logBuffer;
NetworkManager networkManager;

// Objects for MQTT and Home Assistant
WiFiClient wifiClient;
HomeAssistant homeAssistant(wifiClient);

// Variables to manage updates and timing
unsigned long lastDisplayUpdate = 0;
unsigned long lastMqttUpdate = 0;
unsigned long lastSensorRead = 0;
unsigned long lastControlUpdate = 0;

// System state variables
bool killSwitchActive = false;  // Killswitch state (emergency mode)
bool isBurning        = false;  // cached each sensor read — used by PID block

// Manual override mode — stays active until the user switches back to automatic
// (or a critical temperature forces the safety routine). Not kept across reboots.
// Written by the web task and read by loop(), hence volatile.
volatile bool manualMode = false;

// Dallas bus rescan requested from the web task; executed by loop() (sole owner of the 1-Wire bus)
volatile bool dallasScanRequested = false;

// OLED test frame requested from the UI (drawn by loop(), held for a few seconds)
volatile bool displayTestRequested = false;
unsigned long displayHoldUntil = 0;

// Safe mode when the boiler-supply sensor is assigned but unreadable
bool          waterSensorFault = false;
unsigned long waterFaultSecs   = 0;

// Temperature history (1/min for 1 h, then 1/5 min for 24 h)
TemperatureHistory history;
unsigned long lastHistoryRecord = 0;

// Runtime-configurable settings (loaded from LittleFS, defaults from config.h)
RuntimeSettings systemSettings;

// Variables to store sensor values
float ambientLowTemp   = NAN;
float boilerIronTemp   = NAN;
float mixSupplyTemp    = NAN;
float boilerSupplyTemp = NAN;
float floorSupplyTemp  = NAN;
float ambientHighTemp  = NAN;
float floorReturnTemp  = NAN;
float boilerReturnTemp = NAN;
float burningTemp      = NAN;

// Point every relay at the channel pin chosen in relayConfig.
// live=true (runtime change): all channel pins are first driven LOW, then each relay
// re-applies its state on its new pin, so swapping two roles can't leave a stale output.
void applyRelayMapping(bool live) {
    if (live) {
        for (int c = 0; c < RELAY_CHANNEL_COUNT; c++) {
            pinMode(RELAY_CHANNEL_PINS[c], OUTPUT);
            digitalWrite(RELAY_CHANNEL_PINS[c], LOW);
        }
    }
    for (int r = 0; r < RELAY_ROLE_COUNT; r++) {
        uint8_t pin = RELAY_CHANNEL_PINS[relayConfig.channelFor(r)];
        if (live) relayByRole[r]->rebind(pin); else relayByRole[r]->setPin(pin);
    }
    if (live) for (int r = 0; r < RELAY_ROLE_COUNT; r++) relayByRole[r]->reapply();
}

// Temperatures indexed like the history channels (also the PID source index)
float* const tempByChannel[HISTORY_CHANNELS] = {
    &boilerSupplyTemp, &boilerReturnTemp, &boilerIronTemp, &floorSupplyTemp, &floorReturnTemp,
    &mixSupplyTemp,    &ambientLowTemp,   &ambientHighTemp, &burningTemp
};

// Current value of the temperature the PID regulates (NAN if that sensor is missing)
float pidProcessValue() {
    int src = systemSettings.pidSource;
    if (src < 0 || src >= HISTORY_CHANNELS) src = DEFAULT_PID_SOURCE;
    return *tempByChannel[src];
}

// Human-readable cause of the last chip reset
String bootReason = "?";
const char* resetReasonText(esp_reset_reason_t r) {
    switch (r) {
        case ESP_RST_POWERON:   return "Encendido / corte de alimentación";
        case ESP_RST_EXT:       return "Pin de reset externo";
        case ESP_RST_SW:        return "Reinicio por software (OTA o botón de la UI)";
        case ESP_RST_PANIC:     return "FALLO del programa (panic / excepción)";
        case ESP_RST_INT_WDT:   return "Watchdog de interrupciones (programa bloqueado)";
        case ESP_RST_TASK_WDT:  return "Watchdog de tarea (bucle bloqueado)";
        case ESP_RST_WDT:       return "Watchdog";
        case ESP_RST_DEEPSLEEP: return "Salida de deep sleep";
        case ESP_RST_BROWNOUT:  return "CAÍDA DE TENSIÓN (brownout)";
        case ESP_RST_SDIO:      return "SDIO";
        default:                return "Desconocido";
    }
}

// ── Self-test (launched from the UI, never at boot) ──────────────────────────────
// Needs manual mode: the relays are pulsed (inverted for a moment, then restored to the
// state they had), so the automatic logic must not be driving them. Runs inside loop();
// the control loop is paused for the few seconds it lasts.
struct SelfTestState {
    volatile bool running  = false;
    volatile bool done     = false;
    const char*   step     = "";
    bool          error    = false;
    const char*   errorMsg = "";
    bool          displayOk   = false;
    bool          tcOk        = false;
    float         tcTemp      = NAN;
    int           dallasCount = 0;
    int           dallasValid = 0;
    int           relaysPulsed = 0;
} selfTest;
volatile bool selfTestRequested = false;

void runSelfTest() {
    selfTest = SelfTestState();
    selfTest.running = true;
    char buf[32];

    if (!manualMode)    { selfTest.error = true; selfTest.errorMsg = "Requiere modo manual"; selfTest.running = false; selfTest.done = true; return; }
    if (killSwitchActive) { selfTest.error = true; selfTest.errorMsg = "Temperatura crítica";  selfTest.running = false; selfTest.done = true; return; }

    logBuffer.log("Self-test started (from UI)");
    selfTest.displayOk = display.isAvailable();
    selfTest.step = "pantalla";
    display.showMessage("Self-Test", "Display... OK");
    delay(600);

    // Relays: invert for a moment, then put back whatever state they had
    for (int r = 0; r < RELAY_ROLE_COUNT; r++) {
        Relay* rl = relayByRole[r];
        selfTest.step = RelayConfig::roleLabel(r);
        snprintf(buf, sizeof(buf), "%s (GPIO %d)", RelayConfig::roleId(r), rl->getPin());
        bool prev = rl->getState();
        display.showMessage("Relay Test", buf, prev ? "-> OFF (1 s)" : "-> ON (1 s)");
        rl->forceState(!prev);
        delay(1000);
        rl->forceState(prev);
        delay(300);
        selfTest.relaysPulsed++;
    }

    selfTest.step = "termopar";
    display.showMessage("Thermocouple", "MAX6675...");
    delay(300);
    float burnT = sensors.getBurningTemperature();
    selfTest.tcTemp = burnT;
    selfTest.tcOk = !isnan(burnT);
    if (selfTest.tcOk) { snprintf(buf, sizeof(buf), "%.1f C", burnT); display.showMessage("Thermocouple", "MAX6675:", buf, "OK"); }
    else                 display.showMessage("Thermocouple", "MAX6675:", "ERROR / no sensor");
    delay(800);

    selfTest.step = "Dallas";
    int n = sensors.getSensorCount();
    selfTest.dallasCount = n;
    sensors.requestTemperatures();
    delay(800);
    sensors.refreshCache();
    for (int i = 0; i < n; i++)
        if (TemperatureSensors::isValidDallasTemp(sensors.getTempByIndex(i))) selfTest.dallasValid++;
    snprintf(buf, sizeof(buf), "%d/%d con lectura", selfTest.dallasValid, n);
    display.showMessage("Dallas 1-Wire", buf);
    delay(800);

    display.showMessage("Self-Test", "DONE");
    delay(500);

    logBuffer.log(String("Self-test done: thermocouple ") + (selfTest.tcOk ? "OK" : "ERROR") +
                  ", Dallas " + String(selfTest.dallasValid) + "/" + String(n));
    selfTest.running = false;
    selfTest.done    = true;
}

// Functions to handle different system states and actions

// Read every sensor. A failed reading becomes NAN (not the last good value), so the UI,
// history and control logic all see the sensor as missing. Errors are logged once, when
// a sensor goes bad, and again when it recovers — not every second.
void readSensors() {
    static bool wasValid[HISTORY_CHANNELS] = { true, true, true, true, true, true, true, true, true };

    auto update = [&](int ch, const char* name, float raw, bool valid, float& var) {
        var = valid ? raw : NAN;
        if (valid != wasValid[ch]) {
            wasValid[ch] = valid;
            logBuffer.log(valid ? String("Sensor OK again: ") + name
                                : String("Sensor error: ") + name + " = " + String(raw));
        }
    };
    auto dallas = [&](int ch, const char* name, float raw, float& var) {
        update(ch, name, raw, TemperatureSensors::isValidDallasTemp(raw), var);
    };

    sensors.refreshCache();   // single pass over the 1-Wire bus per second
    dallas(0, "boiler_supply", sensors.getBoilerSupplyTemp(), boilerSupplyTemp);
    dallas(1, "boiler_return", sensors.getBoilerReturnTemp(), boilerReturnTemp);
    dallas(2, "boiler_iron",   sensors.getBoilerIronTemp(),   boilerIronTemp);
    dallas(3, "floor_supply",  sensors.getFloorSupplyTemp(),  floorSupplyTemp);
    dallas(4, "floor_return",  sensors.getFloorReturnTemp(),  floorReturnTemp);
    dallas(5, "mix_supply",    sensors.getMixSupplyTemp(),    mixSupplyTemp);
    dallas(6, "ambient_low",   sensors.getAmbientLowTemp(),   ambientLowTemp);
    dallas(7, "ambient_high",  sensors.getAmbientHighTemp(),  ambientHighTemp);

    float t = sensors.getBurningTemperature();
    update(8, "thermocouple", t, !isnan(t) && t >= 0, burningTemp);

    // Request next conversion (non-blocking, result available next cycle)
    sensors.requestTemperatures();
}

// Push one sample into the history ring buffers
void recordHistory() {
    int16_t temps[HISTORY_CHANNELS] = {
        TemperatureHistory::encode(boilerSupplyTemp, !isnan(boilerSupplyTemp)),
        TemperatureHistory::encode(boilerReturnTemp, !isnan(boilerReturnTemp)),
        TemperatureHistory::encode(boilerIronTemp,   !isnan(boilerIronTemp)),
        TemperatureHistory::encode(floorSupplyTemp,  !isnan(floorSupplyTemp)),
        TemperatureHistory::encode(floorReturnTemp,  !isnan(floorReturnTemp)),
        TemperatureHistory::encode(mixSupplyTemp,    !isnan(mixSupplyTemp)),
        TemperatureHistory::encode(ambientLowTemp,   !isnan(ambientLowTemp)),
        TemperatureHistory::encode(ambientHighTemp,  !isnan(ambientHighTemp)),
        TemperatureHistory::encode(burningTemp,      !isnan(burningTemp)),
    };
    uint8_t flags = (boilerPumpRelay.getState()  ? HIST_FLAG_BOILER_PUMP  : 0)
                  | (heatingPumpRelay.getState() ? HIST_FLAG_HEATING_PUMP : 0)
                  | (fansRelay.getState()        ? HIST_FLAG_FANS         : 0)
                  | (otherRelay.getState()       ? HIST_FLAG_OTHER        : 0)
                  | (manualMode                  ? HIST_FLAG_MANUAL       : 0);
    history.record(temps, (uint8_t)constrain((int)airIntake.getCurrentOutput(), 0, 100), flags);
}

void handleCriticalTemperature() {
    // Bypass debounce — evacuate heat immediately
    boilerPumpRelay.forceState(true);
    heatingPumpRelay.forceState(true);
    fansRelay.forceState(true);
    airIntake.setPosition(0);

    if (!killSwitchActive) {
        logBuffer.log("ALERT! Critical water temperature: " + String(boilerSupplyTemp) + "°C - Emergency mode activated");
        killSwitchActive = true;
    }
}

void handleNormalOperation(bool isBurning, bool isBoilerWaterHot) {
    if (killSwitchActive) {
        logBuffer.log("Water temperature normalized: " + String(boilerSupplyTemp) + "°C - Normal mode restored");
        killSwitchActive = false;
    }

    // Boiler pump: ON while burning, while boiler iron body is warm, or while water is hot
    // — keeps extracting residual heat from the boiler mass after combustion ends
    bool ironWarm = TemperatureSensors::isValidDallasTemp(boilerIronTemp)
                    && boilerIronTemp > systemSettings.boilerIronThreshold;
    bool wantBoilerPump = isBurning || ironWarm || isBoilerWaterHot;

    // Heating pump: ON only when boiler pump is running AND water is hot
    //               AND (ida_caldera − retorno_suelo) > threshold
    //               — no differential sensor → fallback to water-hot check only
    bool floorReturnValid = TemperatureSensors::isValidDallasTemp(floorReturnTemp);
    bool deltaOk = !floorReturnValid
                   || (boilerSupplyTemp - floorReturnTemp) > systemSettings.floorDeltaThreshold;
    bool wantHeatingPump = wantBoilerPump && isBoilerWaterHot && deltaOk;

    // Fans: ON while burning
    boilerPumpRelay.setState(wantBoilerPump);
    heatingPumpRelay.setState(wantHeatingPump);
    fansRelay.setState(isBurning);
}

// Boiler-supply sensor unreadable → water temperature unknown. While there is a heat
// source (combustion or hot iron body): pumps forced ON to carry heat away and the air
// damper closed so the fire dies down. With no heat source, behave normally.
void handleSensorFault(bool isBurning) {
    bool ironWarm = TemperatureSensors::isValidDallasTemp(boilerIronTemp)
                    && boilerIronTemp > systemSettings.boilerIronThreshold;
    if (isBurning || ironWarm) {
        boilerPumpRelay.forceState(true);
        heatingPumpRelay.forceState(true);
        fansRelay.setState(isBurning);
        airIntake.setPosition(0);
    } else {
        handleNormalOperation(false, false);
        airIntake.setPosition(0);
    }
}

void turnOffAllRelays() {
    boilerPumpRelay.forceState(false);
    heatingPumpRelay.forceState(false);
    fansRelay.forceState(false);
    otherRelay.forceState(false);
}

void updateHomeAssistant() {
    if (networkManager.isConnected()) {
        homeAssistant.update(
            ambientLowTemp, boilerIronTemp, mixSupplyTemp, boilerSupplyTemp,
            floorSupplyTemp, ambientHighTemp, floorReturnTemp, boilerReturnTemp,
            burningTemp,
            boilerPumpRelay.getState(),
            heatingPumpRelay.getState(),
            fansRelay.getState(),
            otherRelay.getState(),
            airIntake.getTargetTemperature(),
            airIntake.getCurrentOutput()
        );
    }
}

// Collects a (possibly fragmented) request body. Returns a NUL-terminated buffer once
// the whole body arrived, nullptr otherwise (the buffer is owned and freed by the request).
static char* collectBody(AsyncWebServerRequest* req, uint8_t* data, size_t len,
                         size_t index, size_t total, size_t maxSize = 4096) {
    if (total == 0 || total > maxSize) {
        if (index == 0) req->send(413, "application/json", "{\"success\":false,\"error\":\"Bad body size\"}");
        return nullptr;
    }
    if (index == 0) {
        req->_tempObject = malloc(total + 1);
        if (!req->_tempObject) {
            req->send(500, "application/json", "{\"success\":false,\"error\":\"Out of memory\"}");
            return nullptr;
        }
    }
    if (!req->_tempObject) return nullptr;
    memcpy((uint8_t*)req->_tempObject + index, data, len);
    if (index + len < total) return nullptr;
    ((char*)req->_tempObject)[total] = '\0';
    return (char*)req->_tempObject;
}

// Callback to receive MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String topicStr = String(topic);
    String message;
    message.concat((const char*)payload, length);

    logBuffer.log("MQTT received: " + topicStr + " -> " + message);

    // Relay commands are ignored while the user drives the actuators by hand
    if (manualMode && !topicStr.endsWith("/set/target_burning_temp")) {
        logBuffer.log("MQTT command ignored: manual mode active");
        return;
    }

    if (topicStr.endsWith("/set/target_burning_temp")) {
        float targetTemp = message.toFloat();
        airIntake.setTargetTemperature(targetTemp);
        logBuffer.log("New target temperature from MQTT: " + String(targetTemp) + "°C");
    } else if (topicStr.endsWith("/set/boiler_pump")) {
        bool state = (message == "ON");
        boilerPumpRelay.setState(state);
        logBuffer.log("Boiler pump state from MQTT: " + String(state ? "ON" : "OFF"));
    } else if (topicStr.endsWith("/set/heating_pump")) {
        bool state = (message == "ON");
        heatingPumpRelay.setState(state);
        logBuffer.log("Heating pump state from MQTT: " + String(state ? "ON" : "OFF"));
    } else if (topicStr.endsWith("/set/fans")) {
        bool state = (message == "ON");
        fansRelay.setState(state);
        logBuffer.log("Fans state from MQTT: " + String(state ? "ON" : "OFF"));
    } else if (topicStr.endsWith("/set/other_relay")) {
        bool state = (message == "ON");
        otherRelay.setState(state);
        logBuffer.log("Other device state from MQTT: " + String(state ? "ON" : "OFF"));
    }
}

// Minimal WiFi setup page served in AP mode (and at /wifi-setup when connected)
static const char WIFI_SETUP_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html><head><meta charset=UTF-8><meta name=viewport content="width=device-width,initial-scale=1">
<title>WiFi Setup</title><style>
*{box-sizing:border-box}body{font-family:Arial,sans-serif;max-width:440px;margin:20px auto;padding:16px;background:#f5f5f5}
h2{color:#333;margin-bottom:12px}
.card{background:#fff;padding:16px;border-radius:8px;box-shadow:0 2px 8px rgba(0,0,0,.1);margin-bottom:16px}
.net-item{padding:9px 12px;border-bottom:1px solid #eee;cursor:pointer;display:flex;justify-content:space-between;align-items:center;font-size:14px}
.net-item:hover{background:#f0f7ff}.net-item:last-child{border-bottom:0}
label{display:block;margin:10px 0 4px;font-weight:bold;font-size:13px}
input[type=text],input[type=password]{width:100%;padding:8px 10px;border:1px solid #ddd;border-radius:4px;font-size:14px}
.toggle{display:flex;align-items:center;gap:8px;margin:12px 0}
#staticFields{padding:8px;background:#f9f9f9;border-radius:4px;margin-top:4px}
button{width:100%;padding:10px;border:none;border-radius:4px;font-size:15px;cursor:pointer;margin-top:8px;color:#fff}
.btn-save{background:#007bff}.btn-save:hover{background:#0056b3}
.btn-reset{background:#dc3545;margin-top:6px}.btn-reset:hover{background:#a71d2a}
#status{margin-top:10px;padding:10px;border-radius:4px;display:none;font-size:14px}
.ok{background:#d4edda;color:#155724}.err{background:#f8d7da;color:#721c24}
#loading{color:#888;padding:10px;font-size:14px}
</style></head><body>
<h2>&#x1F4F6; WiFi Setup</h2>
<div class=card>
  <b style="font-size:14px">Nearby networks</b>
  <div id=nets><div id=loading>Scanning...</div></div>
</div>
<div class=card>
  <form id=f>
    <label>SSID</label>
    <input type=text id=ssid required placeholder="Network name">
    <label>Password</label>
    <input type=password id=pw placeholder="Leave empty for open network">
    <div class=toggle>
      <input type=checkbox id=si onchange="document.getElementById('sf').style.display=this.checked?'block':'none'">
      <label style="margin:0">Use static IP</label>
    </div>
    <div id=sf style=display:none>
      <label>IP Address</label><input type=text id=ip value="192.168.0.188">
      <label>Gateway</label><input type=text id=gw value="192.168.0.1">
      <label>Subnet Mask</label><input type=text id=sn value="255.255.255.0">
      <label>DNS</label><input type=text id=dns value="192.168.0.1">
    </div>
    <button type=submit class=btn-save>&#x1F4BE; Save &amp; Connect</button>
    <button type=button class=btn-reset onclick=doReset()>&#x1F504; Reset WiFi config</button>
    <div id=status></div>
  </form>
</div>
<script>
function showStatus(msg,ok){var s=document.getElementById('status');s.textContent=msg;s.className=ok?'ok':'err';s.style.display='block'}
fetch('/api/wifi/scan',{method:'POST'}).then(()=>pollScan());
function pollScan(){
  fetch('/api/wifi/scan').then(r=>r.json()).then(d=>{
    if(d.scanning){setTimeout(pollScan,1000);return}
    var el=document.getElementById('nets');
    if(!d.networks||!d.networks.length){el.innerHTML='<div style="padding:10px;font-size:13px">No networks found</div>';return}
    el.innerHTML='';
    d.networks.sort((a,b)=>b.rssi-a.rssi).forEach(n=>{
      var div=document.createElement('div');div.className='net-item';
      div.innerHTML='<span>'+(n.encrypted?'&#x1F512; ':'')+n.ssid+'</span><span style="color:#888;font-size:12px">'+n.rssi+'dBm</span>';
      div.onclick=function(){document.getElementById('ssid').value=n.ssid};
      el.appendChild(div);
    });
  }).catch(()=>setTimeout(pollScan,2000));
}
document.getElementById('f').onsubmit=function(e){
  e.preventDefault();
  var body={ssid:document.getElementById('ssid').value,password:document.getElementById('pw').value,
    static_ip:document.getElementById('si').checked,ip:document.getElementById('ip').value,
    gateway:document.getElementById('gw').value,subnet:document.getElementById('sn').value,
    dns:document.getElementById('dns').value};
  fetch('/api/wifi/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(body)})
    .then(r=>r.json()).then(d=>{if(d.success)showStatus(d.message,true);else showStatus(d.error||'Error',false)})
    .catch(()=>showStatus('Connection error',false));
};
function doReset(){if(confirm('Reset WiFi config and reboot in AP mode?'))
  fetch('/api/wifi/reset',{method:'POST'}).then(r=>r.json()).then(d=>showStatus(d.message,true)).catch(()=>showStatus('Error',false))}
</script></body></html>
)rawhtml";

// Callback for when WiFi connects
void onWiFiConnected() {
    display.setScreen(Display::SCREEN_NETWORK_INFO);
}

// Callback for when WiFi disconnects
void onWiFiDisconnected() {
    // Could update display to show disconnected status
}

// Callback for when AP mode starts
void onAPModeStarted() {
    display.setScreen(Display::SCREEN_NETWORK_INFO);
}

void setup() {
    // Start serial communication
    Serial.begin(115200);
    Serial.println("\n\n--- Lumber Boiler Manager ---");
    Serial.println("Starting...");

    // Temperature history buffers
    history.begin();

    // Initialize log system
    logBuffer.begin();
    logBuffer.log("System started");

    // Mount LittleFS first — sensor config depends on it
    FSHelper::initializeLittleFS();

    // Load runtime settings (must be after LittleFS init)
    if (systemSettings.load()) {
        logBuffer.log("Runtime settings loaded from LittleFS");
    }

    // Load Dallas sensor role assignments
    sensorConfig.begin();
    logBuffer.log("Sensor config loaded");

    // Initialize temperature sensors
    sensors.begin();
    logBuffer.log("Temperature sensors initialized");

    // Load which channel drives which function, then initialise relays on those pins
    relayConfig.begin();
    applyRelayMapping(false);

    // Initialize relays
    boilerPumpRelay.begin();
    heatingPumpRelay.begin();
    fansRelay.begin();
    otherRelay.begin();
    logBuffer.log("Relays initialized");

    // Apply relay debounce — prevents rapid pump cycling (protects motors)
    {
        unsigned long debounceMs = (unsigned long)systemSettings.relayDebounceMin * 60000UL;
        boilerPumpRelay.setDebounce(debounceMs);
        heatingPumpRelay.setDebounce(debounceMs);
        fansRelay.setDebounce(debounceMs);
        otherRelay.setDebounce(debounceMs);
    }

    // Apply runtime settings to sensors, air intake and MQTT
    sensors.setBurningThreshold(systemSettings.burningTempThreshold);
    sensors.setBoilerWaterThreshold(systemSettings.boilerWaterTempThreshold);
    sensors.setBoilerWaterCriticalTemp(systemSettings.boilerWaterCriticalTemp);
    homeAssistant.configure(
        systemSettings.mqttServer, systemSettings.mqttPort,
        systemSettings.mqttUser,   systemSettings.mqttPassword,
        systemSettings.mqttBaseTopic
    );

    // Initialize air intake servo
    airIntake.begin();
    airIntake.setTargetTemperature(systemSettings.targetBurningTemp);
    airIntake.setTunings(systemSettings.pidKp, systemSettings.pidKi, systemSettings.pidKd);
    airIntake.setServoMin(systemSettings.servoMin);
    airIntake.setServoMax(systemSettings.servoMax);
    logBuffer.log("Air intake control initialized");

    // Initialize GLCD display
    display.begin();
    if (display.isAvailable()) {
        logBuffer.log("Display initialized");
    } else {
        logBuffer.log("WARNING: Display not found — check wiring, type (SSD1306/SH1106) and I2C address");
    }

    // Set up WiFi connection callbacks
    networkManager.setOnWifiConnectedCallback(onWiFiConnected);
    networkManager.setOnWifiDisconnectedCallback(onWiFiDisconnected);
    networkManager.setOnAPModeCallback(onAPModeStarted);

    // Configure MQTT callback
    homeAssistant.setCallback(mqttCallback);

    // Initialize WiFi connectivity and web server with references to other components
    networkManager.begin(&logBuffer, &homeAssistant);

    // Root: AP mode → setup page; otherwise serve index.html from LittleFS
    // Must be registered BEFORE serveStatic so it takes priority
    webServer.on("/", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (networkManager.isAPMode()) {
            request->send_P(200, "text/html", WIFI_SETUP_HTML);
        } else {
            request->send(LittleFS, "/index.html", "text/html");
        }
    });

    // WiFi setup page — only in AP mode; redirect to main app when connected
    webServer.on("/wifi-setup", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (networkManager.isAPMode()) {
            request->send_P(200, "text/html", WIFI_SETUP_HTML);
        } else {
            request->redirect("/");
        }
    });

    // Configuración de rutas web estáticas desde LittleFS
    webServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // API: Obtener estado del sistema
    webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
        // Create JSON response with current system state
        AsyncResponseStream *response = request->beginResponseStream("application/json");

        StaticJsonDocument<1024> doc;
        auto setTemp = [&](const char* key, float v) {
            if (TemperatureSensors::isValidDallasTemp(v)) doc[key] = v; else doc[key] = nullptr;
        };
        setTemp("ambient_low",   ambientLowTemp);
        setTemp("boiler_iron",   boilerIronTemp);
        setTemp("mix_supply",    mixSupplyTemp);
        setTemp("boiler_supply", boilerSupplyTemp);
        setTemp("floor_supply",  floorSupplyTemp);
        setTemp("ambient_high",  ambientHighTemp);
        setTemp("floor_return",  floorReturnTemp);
        setTemp("boiler_return", boilerReturnTemp);
        if (!isnan(burningTemp)) doc["burning_temp"] = burningTemp; else doc["burning_temp"] = nullptr;
        doc["boiler_pump"]         = boilerPumpRelay.getState();
        doc["boiler_pump_desired"] = boilerPumpRelay.getDesiredState();
        doc["heating_pump"]        = heatingPumpRelay.getState();
        doc["heating_pump_desired"]= heatingPumpRelay.getDesiredState();
        doc["fans"]                = fansRelay.getState();
        doc["fans_desired"]        = fansRelay.getDesiredState();
        doc["other"]               = otherRelay.getState();
        doc["other_desired"]       = otherRelay.getDesiredState();
        doc["target_burning_temp"] = airIntake.getTargetTemperature();
        doc["air_intake"] = airIntake.getCurrentOutput();
        doc["auto_tuning"] = airIntake.isAutoTuning();
        doc["killswitch_active"] = killSwitchActive;
        doc["manual_mode"] = manualMode;
        doc["uptime_s"] = millis() / 1000UL;
        doc["pid_source"] = systemSettings.pidSource;
        doc["water_sensor_fault"] = waterSensorFault;
        doc["reset_reason"] = bootReason;
        { float pv = pidProcessValue(); if (!isnan(pv)) doc["pid_value"] = pv; else doc["pid_value"] = nullptr; }

        // Add servo range configuration
        doc["servo_min"] = airIntake.getServoMin();
        doc["servo_max"] = airIntake.getServoMax();

        // Add current PID parameters
        JsonObject pid = doc.createNestedObject("pid");
        pid["kp"] = airIntake.getKp();
        pid["ki"] = airIntake.getKi();
        pid["kd"] = airIntake.getKd();

        serializeJson(doc, *response);
        request->send(response);
    });

    // API: Obtener logs del sistema
    webServer.on("/api/logs", HTTP_GET, [](AsyncWebServerRequest *request){
        request->send(200, "text/plain", logBuffer.getAll());
    });

    // API: Configurar ajustes del sistema
    webServer.on("/api/settings", HTTP_POST,
        [](AsyncWebServerRequest *request){},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total){
            if (len > 0) {
                StaticJsonDocument<256> doc;
                DeserializationError error = deserializeJson(doc, data, len);

                if (!error) {
                    if (doc.containsKey("target_burning_temp")) {
                        float targetTemp = doc["target_burning_temp"];
                        airIntake.setTargetTemperature(targetTemp);
                        logBuffer.log("New target temperature: " + String(targetTemp) + "°C");
                    }

                    // Handle servo min/max configuration
                    if (doc.containsKey("servo_min")) {
                        int servoMin = doc["servo_min"];
                        airIntake.setServoMin(servoMin);
                        logBuffer.log("New servo minimum position: " + String(servoMin));
                    }

                    if (doc.containsKey("servo_max")) {
                        int servoMax = doc["servo_max"];
                        airIntake.setServoMax(servoMax);
                        logBuffer.log("New servo maximum position: " + String(servoMax));
                    }

                    // Handle PID auto-tuning request
                    if (doc.containsKey("autotune") && doc["autotune"].as<bool>()) {
                        if (airIntake.isAutoTuning()) {
                            // If there's already an auto-tuning in progress, cancel it
                            airIntake.cancelAutoTune();
                            logBuffer.log("PID auto-tuning canceled");
                        } else {
                            // Start new auto-tuning (needs combustion, automatic mode, no alarms:
                            // the PID — and so the tuning — only runs in that situation)
                            if (!isBurning || manualMode || killSwitchActive || waterSensorFault) {
                                logBuffer.log("PID auto-tuning refused: needs combustion, automatic mode and no alarms");
                            } else if (airIntake.startAutoTune()) {
                                logBuffer.log("Starting PID auto-tuning. This may take several minutes...");
                            } else {
                                logBuffer.log("Could not start PID auto-tuning");
                            }
                        }
                    }

                    AsyncResponseStream *response = request->beginResponseStream("application/json");
                    StaticJsonDocument<64> respDoc;
                    respDoc["success"] = true;
                    serializeJson(respDoc, *response);
                    request->send(response);
                } else {
                    request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                }
            } else {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"No data\"}");
            }
        }
    );

    // GET /api/dallas/sensors — list all Dallas sensors with temp and role
    webServer.on("/api/dallas/sensors", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");

        StaticJsonDocument<1024> doc;
        JsonArray arr = doc.createNestedArray("sensors");

        int n = sensors.getSensorCount();
        for (int i = 0; i < n; i++) {
            char addr[17];
            sensors.getAddressStr(i, addr);
            float t = sensors.getTempByIndex(i);
            const char* role = sensors.getRoleByIndex(i);

            JsonObject s = arr.createNestedObject();
            s["address"] = addr;
            if (TemperatureSensors::isValidDallasTemp(t)) s["temperature"] = t; else s["temperature"] = nullptr;
            s["role"] = role;
            s["index"] = i;
        }

        JsonObject assignments = doc.createNestedObject("assignments");
        assignments[ROLE_AMBIENT_LOW]   = sensorConfig.ambientLowAddr;
        assignments[ROLE_BOILER_IRON]   = sensorConfig.boilerIronAddr;
        assignments[ROLE_MIX_SUPPLY]    = sensorConfig.mixSupplyAddr;
        assignments[ROLE_BOILER_SUPPLY] = sensorConfig.boilerSupplyAddr;
        assignments[ROLE_FLOOR_SUPPLY]  = sensorConfig.floorSupplyAddr;
        assignments[ROLE_AMBIENT_HIGH]  = sensorConfig.ambientHighAddr;
        assignments[ROLE_FLOOR_RETURN]  = sensorConfig.floorReturnAddr;
        assignments[ROLE_BOILER_RETURN] = sensorConfig.boilerReturnAddr;

        // Cached value: don't bit-bang the MAX6675 from the web task
        if (!isnan(burningTemp)) doc["thermocouple_temp"] = burningTemp; else doc["thermocouple_temp"] = nullptr;
        doc["count"] = n;

        serializeJson(doc, *response);
        request->send(response);
    });

    // POST /api/dallas/assign — assign sensor address to a role
    // Body: {"role": "boiler_water", "address": "28FF..."} (empty address = unassign)
    webServer.on("/api/dallas/assign", HTTP_POST,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            StaticJsonDocument<128> doc;
            if (deserializeJson(doc, data, len)) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }
            const char* role = doc["role"] | "";
            const char* addr = doc["address"] | "";

            if (strlen(role) == 0) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Missing role\"}");
                return;
            }

            bool ok = sensorConfig.setRole(role, addr);
            if (ok) {
                logBuffer.log("Sensor assigned: " + String(addr) + " -> " + String(role));
                request->send(200, "application/json", "{\"success\":true}");
            } else {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Unknown role\"}");
            }
        }
    );

    // GET /api/dallas/roles — list all known roles with their display labels
    webServer.on("/api/dallas/roles", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        StaticJsonDocument<512> doc;
        JsonArray arr = doc.createNestedArray("roles");

        auto add = [&](const char* id, const char* label) {
            JsonObject o = arr.createNestedObject();
            o["id"]    = id;
            o["label"] = label;
        };
        add(ROLE_AMBIENT_LOW,   "Ambiente abajo");
        add(ROLE_BOILER_IRON,   "Hierro caldera");
        add(ROLE_MIX_SUPPLY,    "Ida mezcla");
        add(ROLE_BOILER_SUPPLY, "Ida caldera");
        add(ROLE_FLOOR_SUPPLY,  "Ida suelo");
        add(ROLE_AMBIENT_HIGH,  "Ambiente arriba");
        add(ROLE_FLOOR_RETURN,  "Retorno suelo");
        add(ROLE_BOILER_RETURN, "Retorno caldera");

        serializeJson(doc, *response);
        request->send(response);
    });

    // POST /api/dallas/scan — rescan the 1-Wire bus
    webServer.on("/api/dallas/scan", HTTP_POST, [](AsyncWebServerRequest *request) {
        // The loop task owns the 1-Wire bus: just ask it to rescan on its next pass
        dallasScanRequested = true;
        request->send(200, "application/json",
            "{\"success\":true,\"count\":" + String(sensors.getSensorCount()) + ",\"scanning\":true}");
    });

    // ── WiFi config API ─────────────────────────────────────────────────────

    // GET /api/wifi/config — current config (password redacted)
    webServer.on("/api/wifi/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        StaticJsonDocument<256> doc;
        WifiConfig& cfg = networkManager.getConfig();
        doc["ssid"]       = cfg.ssid;
        doc["static_ip"]  = cfg.staticIp;
        doc["ip"]         = cfg.ip;
        doc["gateway"]    = cfg.gateway;
        doc["subnet"]     = cfg.subnet;
        doc["dns"]        = cfg.dns;
        doc["ap_mode"]    = networkManager.isAPMode();
        doc["connected"]  = networkManager.isConnected();
        doc["current_ip"] = networkManager.isConnected()
            ? networkManager.getIPAddress()
            : (networkManager.isAPMode() ? networkManager.getAPIP().toString() : "");
        serializeJson(doc, *response);
        request->send(response);
    });

    // POST /api/wifi/config — save new config and reboot
    webServer.on("/api/wifi/config", HTTP_POST,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            StaticJsonDocument<512> doc;
            if (deserializeJson(doc, data, len)) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }

            WifiConfig cfg;
            strlcpy(cfg.ssid,     doc["ssid"]     | "", sizeof(cfg.ssid));
            strlcpy(cfg.password, doc["password"] | "", sizeof(cfg.password));
            cfg.staticIp = doc["static_ip"] | false;
            strlcpy(cfg.ip,      doc["ip"]      | DEFAULT_STATIC_IP,      sizeof(cfg.ip));
            strlcpy(cfg.gateway, doc["gateway"] | DEFAULT_STATIC_GATEWAY, sizeof(cfg.gateway));
            strlcpy(cfg.subnet,  doc["subnet"]  | DEFAULT_STATIC_SUBNET,  sizeof(cfg.subnet));
            strlcpy(cfg.dns,     doc["dns"]     | DEFAULT_STATIC_DNS,      sizeof(cfg.dns));

            if (strlen(cfg.ssid) == 0) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"SSID required\"}");
                return;
            }

            networkManager.saveConfig(cfg);
            logBuffer.log("WiFi config saved: " + String(cfg.ssid) +
                          (cfg.staticIp ? " static " + String(cfg.ip) : " DHCP"));

            request->send(200, "application/json",
                "{\"success\":true,\"message\":\"Saved. Rebooting...\"}");
            delay(400);
            ESP.restart();
        }
    );

    // POST /api/wifi/scan — start async WiFi scan
    webServer.on("/api/wifi/scan", HTTP_POST, [](AsyncWebServerRequest *request) {
        networkManager.startScan();
        request->send(200, "application/json", "{\"scanning\":true}");
    });

    // GET /api/wifi/scan — return cached scan results
    webServer.on("/api/wifi/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
        if (networkManager.isScanRunning()) {
            request->send(200, "application/json", "{\"scanning\":true,\"networks\":[]}");
        } else if (networkManager.isScanDone()) {
            request->send(200, "application/json", networkManager.getScanJson());
        } else {
            // Not started yet — auto-start and return scanning
            networkManager.startScan();
            request->send(200, "application/json", "{\"scanning\":true,\"networks\":[]}");
        }
    });

    // ── System configuration API ───────────────────────────────────────────

    // NOTE on route order: ESPAsyncWebServer matches by prefix ("/api/config" also matches
    // "/api/config/export"), and the first registered handler wins. Register the specific
    // sub-routes BEFORE their base route, or the base handler swallows them.

    // GET /api/config/export — full backup: all settings + sensor role assignments
    webServer.on("/api/config/export", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        response->addHeader("Content-Disposition", "attachment; filename=\"boiler-config.json\"");

        StaticJsonDocument<2048> doc;
        doc["version"] = 1;

        JsonObject s = doc.createNestedObject("settings");
        s["burning_temp_threshold"] = systemSettings.burningTempThreshold;
        s["boiler_water_threshold"] = systemSettings.boilerWaterTempThreshold;
        s["boiler_water_critical"]  = systemSettings.boilerWaterCriticalTemp;
        s["target_burning_temp"]    = systemSettings.targetBurningTemp;
        s["pid_source"]             = systemSettings.pidSource;
        s["pid_kp"]                 = systemSettings.pidKp;
        s["pid_ki"]                 = systemSettings.pidKi;
        s["pid_kd"]                 = systemSettings.pidKd;
        s["servo_min"]              = systemSettings.servoMin;
        s["servo_max"]              = systemSettings.servoMax;
        s["relay_debounce_min"]     = systemSettings.relayDebounceMin;
        s["boiler_iron_threshold"]  = systemSettings.boilerIronThreshold;
        s["floor_delta_threshold"]  = systemSettings.floorDeltaThreshold;
        s["mqtt_server"]            = systemSettings.mqttServer;
        s["mqtt_port"]              = systemSettings.mqttPort;
        s["mqtt_user"]              = systemSettings.mqttUser;
        s["mqtt_password"]          = systemSettings.mqttPassword;
        s["mqtt_base_topic"]        = systemSettings.mqttBaseTopic;

        JsonObject relayMap = doc.createNestedObject("relay_map");
        for (int r = 0; r < RELAY_ROLE_COUNT; r++) relayMap[RelayConfig::roleId(r)] = relayConfig.channelFor(r);

        JsonObject roles = doc.createNestedObject("sensor_roles");
        roles[ROLE_AMBIENT_LOW]   = sensorConfig.ambientLowAddr;
        roles[ROLE_BOILER_IRON]   = sensorConfig.boilerIronAddr;
        roles[ROLE_MIX_SUPPLY]    = sensorConfig.mixSupplyAddr;
        roles[ROLE_BOILER_SUPPLY] = sensorConfig.boilerSupplyAddr;
        roles[ROLE_FLOOR_SUPPLY]  = sensorConfig.floorSupplyAddr;
        roles[ROLE_AMBIENT_HIGH]  = sensorConfig.ambientHighAddr;
        roles[ROLE_FLOOR_RETURN]  = sensorConfig.floorReturnAddr;
        roles[ROLE_BOILER_RETURN] = sensorConfig.boilerReturnAddr;

        serializeJson(doc, *response);
        request->send(response);
        logBuffer.log("Config exported");
    });

    // POST /api/config/import — restore backup created by /api/config/export
    webServer.on("/api/config/import", HTTP_POST,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            char* body = collectBody(request, data, len, index, total, 6144);
            if (!body) return;
            StaticJsonDocument<2048> doc;
            if (deserializeJson(doc, body)) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }

            // Restore settings (rolled back if the imported set is not coherent)
            if (doc.containsKey("settings")) {
                RuntimeSettings backup = systemSettings;
                JsonObjectConst s = doc["settings"].as<JsonObjectConst>();
                if (s.containsKey("burning_temp_threshold")) systemSettings.burningTempThreshold    = s["burning_temp_threshold"];
                if (s.containsKey("boiler_water_threshold")) systemSettings.boilerWaterTempThreshold = s["boiler_water_threshold"];
                if (s.containsKey("boiler_water_critical"))  systemSettings.boilerWaterCriticalTemp  = s["boiler_water_critical"];
                if (s.containsKey("target_burning_temp"))    systemSettings.targetBurningTemp        = s["target_burning_temp"];
                if (s.containsKey("pid_source")) {
                    int src = s["pid_source"] | -1;
                    if (src >= 0 && src < HISTORY_CHANNELS) systemSettings.pidSource = src;
                }
                if (s.containsKey("pid_kp"))   systemSettings.pidKp = s["pid_kp"];
                if (s.containsKey("pid_ki"))   systemSettings.pidKi = s["pid_ki"];
                if (s.containsKey("pid_kd"))   systemSettings.pidKd = s["pid_kd"];
                if (s.containsKey("servo_min"))          systemSettings.servoMin             = s["servo_min"];
                if (s.containsKey("servo_max"))          systemSettings.servoMax             = s["servo_max"];
                if (s.containsKey("relay_debounce_min")) {
                    systemSettings.relayDebounceMin = s["relay_debounce_min"];
                    unsigned long ms = (unsigned long)systemSettings.relayDebounceMin * 60000UL;
                    boilerPumpRelay.setDebounce(ms);
                    heatingPumpRelay.setDebounce(ms);
                    fansRelay.setDebounce(ms);
                    otherRelay.setDebounce(ms);
                }
                if (s.containsKey("boiler_iron_threshold")) systemSettings.boilerIronThreshold = s["boiler_iron_threshold"];
                if (s.containsKey("floor_delta_threshold"))  systemSettings.floorDeltaThreshold = s["floor_delta_threshold"];
                if (s.containsKey("mqtt_server"))
                    strlcpy(systemSettings.mqttServer,    s["mqtt_server"]    | "", sizeof(systemSettings.mqttServer));
                if (s.containsKey("mqtt_port"))
                    systemSettings.mqttPort = s["mqtt_port"].as<uint16_t>();
                if (s.containsKey("mqtt_user"))
                    strlcpy(systemSettings.mqttUser,      s["mqtt_user"]      | "", sizeof(systemSettings.mqttUser));
                if (s.containsKey("mqtt_password") && strlen(s["mqtt_password"] | "") > 0)
                    strlcpy(systemSettings.mqttPassword,  s["mqtt_password"]  | "", sizeof(systemSettings.mqttPassword));
                if (s.containsKey("mqtt_base_topic"))
                    strlcpy(systemSettings.mqttBaseTopic, s["mqtt_base_topic"] | "", sizeof(systemSettings.mqttBaseTopic));

                if (const char* err = systemSettings.validate()) {
                    systemSettings = backup;
                    request->send(400, "application/json", String("{\"success\":false,\"error\":\"Importación rechazada: ") + err + "\"}");
                    logBuffer.log(String("Config import rejected: ") + err);
                    return;
                }

                sensors.setBurningThreshold(systemSettings.burningTempThreshold);
                sensors.setBoilerWaterThreshold(systemSettings.boilerWaterTempThreshold);
                sensors.setBoilerWaterCriticalTemp(systemSettings.boilerWaterCriticalTemp);
                airIntake.setTargetTemperature(systemSettings.targetBurningTemp);
                airIntake.setTunings(systemSettings.pidKp, systemSettings.pidKi, systemSettings.pidKd);
                airIntake.setServoMin(systemSettings.servoMin);
                airIntake.setServoMax(systemSettings.servoMax);
                homeAssistant.configure(
                    systemSettings.mqttServer, systemSettings.mqttPort,
                    systemSettings.mqttUser,   systemSettings.mqttPassword,
                    systemSettings.mqttBaseTopic
                );
                systemSettings.save();
            }

            // Restore relay → channel assignment (a valid permutation reproduces exactly)
            if (doc.containsKey("relay_map")) {
                JsonObjectConst m = doc["relay_map"].as<JsonObjectConst>();
                for (int r = 0; r < RELAY_ROLE_COUNT; r++) {
                    const char* id = RelayConfig::roleId(r);
                    if (m.containsKey(id)) relayConfig.assign(r, m[id] | -1);
                }
                applyRelayMapping(true);
            }

            // Restore sensor role assignments (set fields directly, one save at the end)
            if (doc.containsKey("sensor_roles")) {
                JsonObjectConst r = doc["sensor_roles"].as<JsonObjectConst>();
                if (r.containsKey(ROLE_AMBIENT_LOW))   strlcpy(sensorConfig.ambientLowAddr,   r[ROLE_AMBIENT_LOW]   | "", 17);
                if (r.containsKey(ROLE_BOILER_IRON))   strlcpy(sensorConfig.boilerIronAddr,   r[ROLE_BOILER_IRON]   | "", 17);
                if (r.containsKey(ROLE_MIX_SUPPLY))    strlcpy(sensorConfig.mixSupplyAddr,    r[ROLE_MIX_SUPPLY]    | "", 17);
                if (r.containsKey(ROLE_BOILER_SUPPLY)) strlcpy(sensorConfig.boilerSupplyAddr, r[ROLE_BOILER_SUPPLY] | "", 17);
                if (r.containsKey(ROLE_FLOOR_SUPPLY))  strlcpy(sensorConfig.floorSupplyAddr,  r[ROLE_FLOOR_SUPPLY]  | "", 17);
                if (r.containsKey(ROLE_AMBIENT_HIGH))  strlcpy(sensorConfig.ambientHighAddr,  r[ROLE_AMBIENT_HIGH]  | "", 17);
                if (r.containsKey(ROLE_FLOOR_RETURN))  strlcpy(sensorConfig.floorReturnAddr,  r[ROLE_FLOOR_RETURN]  | "", 17);
                if (r.containsKey(ROLE_BOILER_RETURN)) strlcpy(sensorConfig.boilerReturnAddr, r[ROLE_BOILER_RETURN] | "", 17);
                sensorConfig.save();
            }

            logBuffer.log("Config imported via /api/config/import");
            request->send(200, "application/json", "{\"success\":true}");
        }
    );

    // POST /api/config/reset — restore compile-time defaults and clear NVS namespaces
    webServer.on("/api/config/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
        systemSettings.resetToDefaults(); // clears NVS + reinitializes struct to compile-time defaults

        // Re-apply defaults to running components
        sensors.setBurningThreshold(systemSettings.burningTempThreshold);
        sensors.setBoilerWaterThreshold(systemSettings.boilerWaterTempThreshold);
        sensors.setBoilerWaterCriticalTemp(systemSettings.boilerWaterCriticalTemp);
        airIntake.setTargetTemperature(systemSettings.targetBurningTemp);
        airIntake.setTunings(systemSettings.pidKp, systemSettings.pidKi, systemSettings.pidKd);
        airIntake.setServoMin(systemSettings.servoMin);
        airIntake.setServoMax(systemSettings.servoMax);
        {
            unsigned long ms = (unsigned long)systemSettings.relayDebounceMin * 60000UL;
            boilerPumpRelay.setDebounce(ms);
            heatingPumpRelay.setDebounce(ms);
            fansRelay.setDebounce(ms);
            otherRelay.setDebounce(ms);
        }
        homeAssistant.configure(
            systemSettings.mqttServer, systemSettings.mqttPort,
            systemSettings.mqttUser,   systemSettings.mqttPassword,
            systemSettings.mqttBaseTopic
        );
        homeAssistant.forceReconnect();

        logBuffer.log("Settings reset to factory defaults");
        request->send(200, "application/json", "{\"success\":true}");
    });

    // GET /api/config — return all runtime-configurable settings
    webServer.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        StaticJsonDocument<1536> doc;
        doc["burning_temp_threshold"] = systemSettings.burningTempThreshold;
        doc["boiler_water_threshold"] = systemSettings.boilerWaterTempThreshold;
        doc["boiler_water_critical"]  = systemSettings.boilerWaterCriticalTemp;
        doc["target_burning_temp"]    = systemSettings.targetBurningTemp;
        doc["pid_source"]             = systemSettings.pidSource;
        doc["pid_kp"]                 = systemSettings.pidKp;
        doc["pid_ki"]                 = systemSettings.pidKi;
        doc["pid_kd"]                 = systemSettings.pidKd;
        doc["servo_min"]              = systemSettings.servoMin;
        doc["servo_max"]              = systemSettings.servoMax;
        doc["relay_debounce_min"]     = systemSettings.relayDebounceMin;
        doc["boiler_iron_threshold"]  = systemSettings.boilerIronThreshold;
        doc["floor_delta_threshold"]  = systemSettings.floorDeltaThreshold;
        doc["mqtt_server"]            = systemSettings.mqttServer;
        doc["mqtt_port"]              = systemSettings.mqttPort;
        doc["mqtt_user"]              = systemSettings.mqttUser;
        // password intentionally omitted from GET — send non-empty string in POST to update
        doc["mqtt_base_topic"]        = systemSettings.mqttBaseTopic;
        // Hardware / validity constants (used by the UI to avoid hardcoding)
        doc["one_wire_pin"]           = ONE_WIRE_PIN;
        doc["dallas_min_valid_temp"]  = DALLAS_MIN_VALID_TEMP;
        doc["dallas_max_valid_temp"]  = DALLAS_MAX_VALID_TEMP;
        doc["target_temp_min"]        = TARGET_TEMP_MIN;
        doc["target_temp_max"]        = TARGET_TEMP_MAX;
        // Include compile-time defaults for reference in the UI
        JsonObject defaults = doc.createNestedObject("defaults");
        defaults["burning_temp_threshold"] = BURNING_TEMP_THRESHOLD;
        defaults["boiler_water_threshold"] = BOILER_WATER_TEMP_THRESHOLD;
        defaults["boiler_water_critical"]  = BOILER_WATER_CRITICAL_TEMP;
        defaults["target_burning_temp"]    = DEFAULT_TARGET_BURNING_TEMP;
        defaults["pid_source"]             = DEFAULT_PID_SOURCE;
        defaults["pid_kp"]                 = PID_KP;
        defaults["pid_ki"]                 = PID_KI;
        defaults["pid_kd"]                 = PID_KD;
        defaults["servo_min"]              = DEFAULT_SERVO_MIN;
        defaults["servo_max"]              = DEFAULT_SERVO_MAX;
        defaults["relay_debounce_min"]     = RELAY_DEBOUNCE_MIN;
        defaults["boiler_iron_threshold"]  = BOILER_IRON_PUMP_THRESHOLD;
        defaults["floor_delta_threshold"]  = FLOOR_DELTA_THRESHOLD;
        defaults["mqtt_server"]            = MQTT_SERVER;
        defaults["mqtt_port"]              = MQTT_PORT;
        defaults["mqtt_user"]              = MQTT_USER;
        defaults["mqtt_base_topic"]        = MQTT_BASE_TOPIC;
        serializeJson(doc, *response);
        request->send(response);
    });

    // POST /api/config — update one or more settings (partial update supported)
    // Any missing key keeps its current value
    webServer.on("/api/config", HTTP_POST,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            char* body = collectBody(request, data, len, index, total);
            if (!body) return;
            StaticJsonDocument<768> doc;
            if (deserializeJson(doc, body)) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }

            // Work on a copy: nothing changes unless the whole set is coherent
            RuntimeSettings next = systemSettings;

            // Update only the fields present in the request
            if (doc.containsKey("burning_temp_threshold"))
                next.burningTempThreshold    = doc["burning_temp_threshold"];
            if (doc.containsKey("boiler_water_threshold"))
                next.boilerWaterTempThreshold = doc["boiler_water_threshold"];
            if (doc.containsKey("boiler_water_critical"))
                next.boilerWaterCriticalTemp  = doc["boiler_water_critical"];
            if (doc.containsKey("target_burning_temp"))
                next.targetBurningTemp        = doc["target_burning_temp"];
            if (doc.containsKey("pid_source")) next.pidSource = doc["pid_source"] | -1;
            if (doc.containsKey("pid_kp"))  next.pidKp = doc["pid_kp"];
            if (doc.containsKey("pid_ki"))  next.pidKi = doc["pid_ki"];
            if (doc.containsKey("pid_kd"))  next.pidKd = doc["pid_kd"];
            if (doc.containsKey("servo_min")) next.servoMin = doc["servo_min"];
            if (doc.containsKey("servo_max")) next.servoMax = doc["servo_max"];
            if (doc.containsKey("relay_debounce_min")) next.relayDebounceMin = doc["relay_debounce_min"];
            if (doc.containsKey("boiler_iron_threshold"))
                next.boilerIronThreshold = doc["boiler_iron_threshold"];
            if (doc.containsKey("floor_delta_threshold"))
                next.floorDeltaThreshold = doc["floor_delta_threshold"];

            bool mqttChanged = false;
            if (doc.containsKey("mqtt_server")) {
                strlcpy(next.mqttServer, doc["mqtt_server"] | "", sizeof(next.mqttServer));
                mqttChanged = true;
            }
            if (doc.containsKey("mqtt_port")) {
                next.mqttPort = doc["mqtt_port"].as<uint16_t>();
                mqttChanged = true;
            }
            if (doc.containsKey("mqtt_user")) {
                strlcpy(next.mqttUser, doc["mqtt_user"] | "", sizeof(next.mqttUser));
                mqttChanged = true;
            }
            if (doc.containsKey("mqtt_password") && strlen(doc["mqtt_password"] | "") > 0) {
                strlcpy(next.mqttPassword, doc["mqtt_password"] | "", sizeof(next.mqttPassword));
                mqttChanged = true;
            }
            if (doc.containsKey("mqtt_base_topic")) {
                strlcpy(next.mqttBaseTopic, doc["mqtt_base_topic"] | "", sizeof(next.mqttBaseTopic));
                mqttChanged = true;
            }

            if (const char* err = next.validate()) {
                request->send(400, "application/json",
                    String("{\"success\":false,\"error\":\"") + err + "\"}");
                logBuffer.log(String("Config rejected: ") + err);
                return;
            }
            bool pidSourceChanged = next.pidSource != systemSettings.pidSource;
            systemSettings = next;
            if (pidSourceChanged) {
                airIntake.resumeFromCurrent();   // different process variable: restart PID smoothly
                logBuffer.log(String("PID source: ") + TemperatureHistory::channelName(systemSettings.pidSource));
            }
            {
                unsigned long ms = (unsigned long)systemSettings.relayDebounceMin * 60000UL;
                boilerPumpRelay.setDebounce(ms);
                heatingPumpRelay.setDebounce(ms);
                fansRelay.setDebounce(ms);
                otherRelay.setDebounce(ms);
            }

            // Apply immediately to running components
            sensors.setBurningThreshold(systemSettings.burningTempThreshold);
            sensors.setBoilerWaterThreshold(systemSettings.boilerWaterTempThreshold);
            sensors.setBoilerWaterCriticalTemp(systemSettings.boilerWaterCriticalTemp);
            airIntake.setTargetTemperature(systemSettings.targetBurningTemp);
            airIntake.setTunings(systemSettings.pidKp, systemSettings.pidKi, systemSettings.pidKd);
            airIntake.setServoRange(systemSettings.servoMin, systemSettings.servoMax);
            if (mqttChanged) {
                homeAssistant.configure(
                    systemSettings.mqttServer, systemSettings.mqttPort,
                    systemSettings.mqttUser,   systemSettings.mqttPassword,
                    systemSettings.mqttBaseTopic
                );
                homeAssistant.forceReconnect(); // NetworkManager will reconnect on next cycle
                logBuffer.log("MQTT config updated, reconnecting...");
            }

            // Persist to LittleFS
            bool saved = systemSettings.save();
            logBuffer.log(saved ? "Settings saved to LittleFS" : "Settings applied (save failed)");

            request->send(200, "application/json", saved
                ? "{\"success\":true}"
                : "{\"success\":true,\"warning\":\"Could not persist to flash\"}");
        }
    );

    // ── Manual override API ────────────────────────────────────────────────

    // (Registered before /api/manual on purpose — see the route-order note above.)
    // POST /api/manual/actuators — drive relays and servo while in manual mode
    // {"boiler_pump": true, "heating_pump": false, "fans": true, "other": false, "servo": 75}
    // Any missing key is left unchanged (partial updates allowed).
    webServer.on("/api/manual/actuators", HTTP_POST,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            char* body = collectBody(request, data, len, index, total, 256);
            if (!body) return;

            if (!manualMode) {
                request->send(409, "application/json",
                    "{\"success\":false,\"error\":\"Not in manual mode\"}");
                return;
            }

            StaticJsonDocument<128> doc;
            if (deserializeJson(doc, body)) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }

            // Manual commands bypass debounce for immediate response
            auto apply = [&](const char* key, Relay& relay, const char* label) {
                if (!doc.containsKey(key)) return;
                bool st = doc[key].as<bool>();
                if (relay.getState() == st) return;
                relay.forceState(st);
                logBuffer.log(String("Manual: ") + label + (st ? " ON" : " OFF"));
            };
            apply("boiler_pump",  boilerPumpRelay,  "bomba caldera");
            apply("heating_pump", heatingPumpRelay, "bomba calefacción");
            apply("fans",         fansRelay,        "ventiladores");
            apply("other",        otherRelay,       "relé auxiliar");

            if (doc.containsKey("servo")) {
                int pos = constrain(doc["servo"].as<int>(), 0, 100);
                airIntake.setPosition(pos);
                logBuffer.log("Manual: servo " + String(pos) + "%");
            }

            request->send(200, "application/json", "{\"success\":true}");
        }
    );

    // GET /api/manual — {"active": bool}
    webServer.on("/api/manual", HTTP_GET, [](AsyncWebServerRequest *request) {
        request->send(200, "application/json",
            String("{\"active\":") + (manualMode ? "true" : "false") + "}");
    });

    // POST /api/manual {"active": true|false}
    // Manual mode has no timeout: it stays until switched off. Leaving it hands
    // control back to the automatic logic immediately (debounce cleared, PID resumes
    // from the current servo position).
    webServer.on("/api/manual", HTTP_POST,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            char* body = collectBody(request, data, len, index, total, 256);
            if (!body) return;

            StaticJsonDocument<128> doc;
            if (deserializeJson(doc, body) || !doc.containsKey("active")) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }
            bool activate = doc["active"].as<bool>();

            if (activate && killSwitchActive) {
                request->send(409, "application/json",
                    "{\"success\":false,\"error\":\"Temperatura crítica: manual bloqueado\"}");
                return;
            }

            if (activate && !manualMode) {
                manualMode = true;
                logBuffer.log("Manual mode ON");
            } else if (!activate && manualMode) {
                manualMode = false;
                boilerPumpRelay.resetDebounce();
                heatingPumpRelay.resetDebounce();
                fansRelay.resetDebounce();
                otherRelay.resetDebounce();
                airIntake.resumeFromCurrent();
                lastSensorRead = 0;   // re-evaluate automatic logic on the next loop pass
                logBuffer.log("Manual mode OFF — automatic control restored");
            }

            request->send(200, "application/json",
                String("{\"success\":true,\"active\":") + (manualMode ? "true" : "false") + "}");
        }
    );

    // ── Relay channel assignment ───────────────────────────────────────────

    // GET /api/relays/map — channels, roles and the current role → channel assignment
    webServer.on("/api/relays/map", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        StaticJsonDocument<768> doc;
        JsonArray chs = doc.createNestedArray("channels");
        for (int c = 0; c < RELAY_CHANNEL_COUNT; c++) {
            JsonObject o = chs.createNestedObject();
            o["id"]  = c;
            o["pin"] = RELAY_CHANNEL_PINS[c];
        }
        JsonArray roles = doc.createNestedArray("roles");
        for (int r = 0; r < RELAY_ROLE_COUNT; r++) {
            JsonObject o = roles.createNestedObject();
            o["id"]      = RelayConfig::roleId(r);
            o["label"]   = RelayConfig::roleLabel(r);
            o["channel"] = relayConfig.channelFor(r);
            o["state"]   = relayByRole[r]->getState();
        }
        serializeJson(doc, *response);
        request->send(response);
    });

    // POST /api/relays/map {"role": "boiler_pump", "channel": 2}
    // If another role already uses that channel the two swap, so the mapping stays valid.
    webServer.on("/api/relays/map", HTTP_POST,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            char* body = collectBody(request, data, len, index, total, 256);
            if (!body) return;
            StaticJsonDocument<128> doc;
            if (deserializeJson(doc, body) || !doc.containsKey("role") || !doc.containsKey("channel")) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }
            int role = RelayConfig::roleFromId(doc["role"] | "");
            int ch   = doc["channel"] | -1;
            if (role < 0 || ch < 0 || ch >= RELAY_CHANNEL_COUNT) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Unknown role or channel\"}");
                return;
            }
            if (relayConfig.channelFor(role) != ch) {
                int swapped = relayConfig.roleOn(ch);
                bool saved = relayConfig.assign(role, ch);
                applyRelayMapping(true);
                logBuffer.log(String("Relay map: ") + RelayConfig::roleLabel(role) + " -> canal " + String(ch + 1) +
                              " (GPIO " + String(RELAY_CHANNEL_PINS[ch]) + ")" +
                              (swapped >= 0 ? String(", intercambiado con ") + RelayConfig::roleLabel(swapped) : String("")) +
                              (saved ? "" : " [no se pudo guardar]"));
            }
            request->send(200, "application/json", "{\"success\":true}");
        }
    );

    // ── Self-test (UI-launched) ────────────────────────────────────────────

    // POST /api/selftest — start it (manual mode only; runs inside loop())
    webServer.on("/api/selftest", HTTP_POST, [](AsyncWebServerRequest *request) {
        if (!manualMode) {
            request->send(409, "application/json", "{\"success\":false,\"error\":\"Activa el modo manual primero\"}");
            return;
        }
        if (killSwitchActive) {
            request->send(409, "application/json", "{\"success\":false,\"error\":\"Temperatura crítica\"}");
            return;
        }
        if (selfTest.running || selfTestRequested) {
            request->send(409, "application/json", "{\"success\":false,\"error\":\"Ya en marcha\"}");
            return;
        }
        selfTest.done = false;
        selfTestRequested = true;
        request->send(200, "application/json", "{\"success\":true}");
    });

    // GET /api/selftest — progress / result
    webServer.on("/api/selftest", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        StaticJsonDocument<384> doc;
        doc["running"] = selfTest.running || selfTestRequested;
        doc["done"]    = selfTest.done;
        doc["step"]    = selfTest.step;
        if (selfTest.error) doc["error"] = selfTest.errorMsg;
        doc["display_ok"]    = selfTest.displayOk;
        doc["relays_pulsed"] = selfTest.relaysPulsed;
        doc["tc_ok"]         = selfTest.tcOk;
        if (selfTest.tcOk) doc["tc_temp"] = selfTest.tcTemp; else doc["tc_temp"] = nullptr;
        doc["dallas_count"]  = selfTest.dallasCount;
        doc["dallas_valid"]  = selfTest.dallasValid;
        serializeJson(doc, *response);
        request->send(response);
    });

    // GET /api/history — temperature history (see history.h for the format)
    webServer.on("/api/history", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        history.writeJson(*response);
        request->send(response);
    });

    // GET /api/mqtt/status — connection state, fail count, next retry countdown
    webServer.on("/api/mqtt/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        HomeAssistant::Status st = homeAssistant.getStatus();
        unsigned long now = millis();
        StaticJsonDocument<256> doc;
        doc["connected"]            = st.connected;
        doc["fail_count"]           = st.failCount;
        doc["last_rc"]              = st.lastRcCode;
        doc["publish_count"]        = st.publishCount;
        doc["server"]               = st.server;
        doc["port"]                 = st.port;
        doc["last_publish_ago_sec"] = (st.lastPublishMs > 0)
                                        ? (int32_t)((now - st.lastPublishMs) / 1000) : -1;
        if (!st.connected && st.nextRetryMs > 0 && st.nextRetryMs > now)
            doc["next_retry_sec"] = (int32_t)((st.nextRetryMs - now) / 1000);
        else
            doc["next_retry_sec"] = st.connected ? -1 : 0;
        serializeJson(doc, *response);
        request->send(response);
    });

    // POST /api/mqtt/retry — reset backoff and attempt an immediate reconnect
    webServer.on("/api/mqtt/retry", HTTP_POST, [](AsyncWebServerRequest *request) {
        homeAssistant.forceRetry();
        logBuffer.log("MQTT: manual retry triggered from UI");
        request->send(200, "application/json", "{\"success\":true}");
    });

    // GET /api/display/status — available flag + configured type + pins
    webServer.on("/api/display/status", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        StaticJsonDocument<128> doc;
        doc["available"] = display.isAvailable();
#ifdef OLED_TYPE_SH1106
        doc["type"] = "SH1106";
#else
        doc["type"] = "SSD1306";
#endif
        doc["sda_pin"] = OLED_SDA;
        doc["scl_pin"] = OLED_SCL;
        serializeJson(doc, *response);
        request->send(response);
    });

    // POST /api/display/test — draw a quick test frame so the user can verify visibility
    webServer.on("/api/display/test", HTTP_POST, [](AsyncWebServerRequest *request) {
        displayTestRequested = true;   // drawn by loop(): the OLED is not touched from the web task
        request->send(200, "application/json",
            String("{\"success\":true,\"available\":") + (display.isAvailable() ? "true" : "false") + "}");
    });

    // GET /api/i2c/scan — software I2C scan (bit-bang, push-pull) on OLED pins.
    // Uses OUTPUT for driving data bits so the line rises instantly (no slow pullup
    // rise-time false-ACK). Only the ACK bit releases SDA to INPUT so the device
    // can pull it low. Wire hardware I2C is not used.
    webServer.on("/api/i2c/scan", HTTP_GET, [](AsyncWebServerRequest *request) {
        const int sda = OLED_SDA;
        const int scl = OLED_SCL;

        // Both pins as push-pull OUTPUT starting from idle (high)
        pinMode(sda, OUTPUT); digitalWrite(sda, HIGH);
        pinMode(scl, OUTPUT); digitalWrite(scl, HIGH);
        delayMicroseconds(20);

        // Returns true if a device ACKs at addr
        auto probe = [&](uint8_t addr) -> bool {
            // START: SDA falls while SCL is high
            digitalWrite(sda, HIGH); digitalWrite(scl, HIGH); delayMicroseconds(10);
            digitalWrite(sda, LOW);  delayMicroseconds(10);
            digitalWrite(scl, LOW);  delayMicroseconds(5);

            // Send address byte (addr<<1 | 0 = write direction)
            uint8_t b = (uint8_t)(addr << 1);
            for (int i = 7; i >= 0; i--) {
                digitalWrite(sda, (b >> i) & 1 ? HIGH : LOW);
                delayMicroseconds(5);
                digitalWrite(scl, HIGH); delayMicroseconds(10);
                digitalWrite(scl, LOW);  delayMicroseconds(5);
            }

            // ACK: release SDA to INPUT (device pulls LOW to ACK via its pullup)
            pinMode(sda, INPUT);
            delayMicroseconds(15);              // let line settle
            digitalWrite(scl, HIGH);
            delayMicroseconds(10);
            bool ack = (digitalRead(sda) == LOW);
            digitalWrite(scl, LOW);
            delayMicroseconds(5);

            // Restore SDA to output for STOP
            pinMode(sda, OUTPUT);

            // STOP: SDA rises while SCL is high
            digitalWrite(sda, LOW);  delayMicroseconds(5);
            digitalWrite(scl, HIGH); delayMicroseconds(10);
            digitalWrite(sda, HIGH); delayMicroseconds(10);

            return ack;
        };

        AsyncResponseStream *response = request->beginResponseStream("application/json");
        StaticJsonDocument<512> doc;
        JsonArray found = doc.createNestedArray("devices");
        for (uint8_t addr = 0x08; addr < 0x78; addr++) {
            if (probe(addr)) {
                char buf[7];
                snprintf(buf, sizeof(buf), "0x%02X", addr);
                found.add(buf);
            }
        }

        // Leave pins as idle-high outputs so U8g2 can retake them normally
        digitalWrite(sda, HIGH); digitalWrite(scl, HIGH);

        doc["count"]   = (int)found.size();
        doc["sda_pin"] = sda;
        doc["scl_pin"] = scl;
        serializeJson(doc, *response);
        request->send(response);
    });

    // POST /api/reset — reboot the ESP32
    webServer.on("/api/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
        logBuffer.log("Manual reboot requested via UI");
        request->send(200, "application/json", "{\"success\":true,\"message\":\"Reiniciando...\"}");
        delay(400);
        ESP.restart();
    });

    // POST /api/wifi/reset — delete config and reboot into AP mode
    webServer.on("/api/wifi/reset", HTTP_POST, [](AsyncWebServerRequest *request) {
        networkManager.deleteConfig();
        logBuffer.log("WiFi config reset — rebooting in AP mode");
        request->send(200, "application/json",
            "{\"success\":true,\"message\":\"Config deleted. Rebooting in AP mode...\"}");
        delay(400);
        ESP.restart();
    });

    // Iniciar el servidor web después de configurar todas las rutas
    webServer.begin();
    logBuffer.log("Web server started on port " + String(WEB_SERVER_PORT));

    // Set display to toggle screens every 5 seconds
    display.setScreenToggleInterval(5000);

    // Initial async temperature request before self-test reads
    sensors.requestTemperatures();

    // Why did the chip restart? Logged in plain words and exposed in /api/status.
    {
        esp_reset_reason_t rr = esp_reset_reason();
        bootReason = String(resetReasonText(rr)) + " (código " + String((int)rr) + ")";
        logBuffer.log("Último reinicio: " + bootReason);
    }


    logBuffer.log("System ready!");
}

void loop() {
    unsigned long currentMillis = millis();

    // Update network manager (handles WiFi connection, MQTT reconnection and OTA)
    networkManager.update();

    // UI-requested self-test (blocking, a few seconds, manual mode only)
    if (selfTestRequested) {
        selfTestRequested = false;
        runSelfTest();
    }

    // Tick relay debounce timers — only in automatic mode; manual mode uses forceState() directly
    if (!manualMode) {
        if (boilerPumpRelay.tick())  logBuffer.log("Bomba caldera: "    + String(boilerPumpRelay.getState()  ? "ON" : "OFF"));
        if (heatingPumpRelay.tick()) logBuffer.log("Bomba calefacción: " + String(heatingPumpRelay.getState() ? "ON" : "OFF"));
        if (fansRelay.tick())        logBuffer.log("Ventiladores: "      + String(fansRelay.getState()        ? "ON" : "OFF"));
        if (otherRelay.tick())       logBuffer.log("Relé auxiliar: "     + String(otherRelay.getState()       ? "ON" : "OFF"));
    }

    // Read sensors every second
    if (currentMillis - lastSensorRead >= 1000) {
        lastSensorRead = currentMillis;

        // Read all sensors (or rescan the bus if the UI asked for it)
        if (dallasScanRequested) {
            dallasScanRequested = false;
            int n = sensors.scanSensors();
            sensors.requestTemperatures();
            logBuffer.log("Dallas bus rescanned: " + String(n) + " sensors found");
        } else {
            readSensors();
        }

        // Check operation conditions
        // Flue-gas thermocouple with hysteresis: on above T, off below T - hysteresis
        if (isnan(burningTemp))                                                   isBurning = false;
        else if (burningTemp > systemSettings.burningTempThreshold)               isBurning = true;
        else if (burningTemp < systemSettings.burningTempThreshold - BURNING_HYSTERESIS) isBurning = false;
        bool isBoilerWaterHot = sensors.isBoilerWaterHot();
        bool isBoilerWaterCritical = sensors.isBoilerWaterCritical();

        // Boiler-supply sensor watchdog (only if a sensor is actually assigned to that role)
        bool supplyAssigned = sensorConfig.boilerSupplyAddr[0] != '\0';
        if (supplyAssigned && isnan(boilerSupplyTemp)) waterFaultSecs++; else waterFaultSecs = 0;
        bool fault = waterFaultSecs >= WATER_SENSOR_FAULT_SECS;
        if (fault != waterSensorFault) {
            waterSensorFault = fault;
            logBuffer.log(fault ? "ALERT: boiler supply sensor has no reading — safe mode (pumps ON while hot, air closed)"
                                : "Boiler supply sensor recovered — normal control restored");
            if (!fault) airIntake.resumeFromCurrent();
        }

        // Critical safety always overrides everything, including manual mode
        if (isBoilerWaterCritical) {
            if (manualMode) {
                manualMode = false;
                logBuffer.log("Manual mode cancelled — critical temperature override");
            }
            handleCriticalTemperature();
        } else if (manualMode) {
            // Critical condition cleared while the user is in manual mode
            if (killSwitchActive) {
                killSwitchActive = false;
                logBuffer.log("Water temperature normalized — manual mode still active");
            }
        } else if (waterSensorFault) {
            handleSensorFault(isBurning);
        } else {
            handleNormalOperation(isBurning, isBoilerWaterHot);
        }
    }

    // History sample once per minute
    if (currentMillis - lastHistoryRecord >= HISTORY_FINE_INTERVAL_MS) {
        lastHistoryRecord = currentMillis;
        recordHistory();
    }

    // PID + servo update — only in automatic mode
    if (currentMillis - lastControlUpdate >= PID_SAMPLE_TIME && !killSwitchActive && !manualMode && !waterSensorFault) {
        lastControlUpdate = currentMillis;
        static bool prevBurning = false;
        if (isBurning) {
            if (!prevBurning) {
                // Combustion just started — restart the PID from the current damper
                // position so windup from the cold period doesn't slam the servo.
                airIntake.resumeFromCurrent();
                logBuffer.log("Combustion detected — air intake PID resumed");
            }
            float pv = pidProcessValue();
            static bool pvWasBad = false;
            if (isnan(pv)) {
                // Regulated sensor missing: close the damper rather than steer blind
                if (!pvWasBad) logBuffer.log("PID source sensor invalid — air intake closed");
                airIntake.setPosition(0);
            } else {
                if (pvWasBad) { airIntake.resumeFromCurrent(); logBuffer.log("PID source sensor recovered"); }
                airIntake.update(pv);
            }
            pvWasBad = isnan(pv);
        } else {
            if (prevBurning) {
                logBuffer.log("Combustion lost — air intake closed");
            }
            airIntake.setPosition(0);   // no combustion → close damper
        }
        prevBurning = isBurning;
    }

    if (displayTestRequested) {
        displayTestRequested = false;
        char sdaLine[16], sclLine[16];
        snprintf(sdaLine, sizeof(sdaLine), "SDA: %d", OLED_SDA);
        snprintf(sclLine, sizeof(sclLine), "SCL: %d", OLED_SCL);
        display.showMessage("Display Test", sdaLine, sclLine);
        displayHoldUntil = currentMillis + 4000;
    }

    // Persist the gains when an auto-tune finishes
    {
        double kp, ki, kd;
        if (airIntake.takeTuneResult(kp, ki, kd)) {
            systemSettings.pidKp = kp; systemSettings.pidKi = ki; systemSettings.pidKd = kd;
            systemSettings.save();
            logBuffer.log("PID auto-tune finished: Kp=" + String(kp, 3) + " Ki=" + String(ki, 4) + " Kd=" + String(kd, 3) + " (saved)");
        }
    }

    // Update display every 500ms (not while a test frame is being shown)
    if (currentMillis - lastDisplayUpdate >= 500 && currentMillis >= displayHoldUntil) {
        lastDisplayUpdate = currentMillis;

        // Update display with current values
        display.update(
            boilerSupplyTemp,
            floorSupplyTemp,
            burningTemp,
            ambientLowTemp,
            boilerPumpRelay.getState(),
            heatingPumpRelay.getState(),
            fansRelay.getState(),
            airIntake.getTargetTemperature(),
            airIntake.getCurrentOutput()
        );
    }

    // MQTT keepalive + incoming message processing (every loop iteration)
    homeAssistant.tick();

    // Publish to MQTT every 10 seconds
    if (currentMillis - lastMqttUpdate >= MQTT_PUBLISH_INTERVAL) {
        lastMqttUpdate = currentMillis;
        updateHomeAssistant();
    }
}