#include <Arduino.h>
#include <WiFi.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include <ESPAsyncWebServer.h>

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

// Manual override mode
bool          manualMode        = false;
unsigned long manualModeStart   = 0;
unsigned long manualModeTimeout = MANUAL_MODE_DEFAULT_TIMEOUT_MIN * 60UL * 1000UL;

// Runtime-configurable settings (loaded from LittleFS, defaults from config.h)
RuntimeSettings systemSettings;

// Variables to store sensor values
float ambientLowTemp   = 0;
float boilerIronTemp   = 0;
float mixSupplyTemp    = 0;
float boilerSupplyTemp = 0;
float floorSupplyTemp  = 0;
float ambientHighTemp  = 0;
float floorReturnTemp  = 0;
float boilerReturnTemp = 0;
float burningTemp      = 0;

void runSelfTest() {
    char buf[32];

    display.showMessage("Self-Test", "Display... OK");
    delay(800);

    Relay* relays[] = { &boilerPumpRelay, &heatingPumpRelay, &fansRelay, &otherRelay };
    const char* relayNames[] = { "Boiler Pump", "Heating Pump", "Fans", "Other" };

    for (int i = 0; i < 4; i++) {
        snprintf(buf, sizeof(buf), "Relay: %s", relayNames[i]);
        display.showMessage("Relay Test", buf, "-> ON");
        relays[i]->forceState(true);
        delay(600);
        display.showMessage("Relay Test", buf, "-> OFF");
        relays[i]->forceState(false);
        delay(400);
    }

    display.showMessage("Thermocouple", "MAX6675...");
    delay(300);
    float burnT = sensors.getBurningTemperature();
    if (isnan(burnT)) {
        display.showMessage("Thermocouple", "MAX6675:", "ERROR / no sensor");
    } else {
        snprintf(buf, sizeof(buf), "%.1f C", burnT);
        display.showMessage("Thermocouple", "MAX6675:", buf, "OK");
    }
    delay(1200);

    int n = sensors.getSensorCount();
    snprintf(buf, sizeof(buf), "Found: %d sensor%s", n, n == 1 ? "" : "s");
    display.showMessage("Dallas 1-Wire", buf);
    delay(1000);

    sensors.requestTemperatures();
    delay(800);

    for (int i = 0; i < n; i++) {
        char addr[17];
        sensors.getAddressStr(i, addr);
        float t = sensors.getTempByIndex(i);
        const char* role = sensors.getRoleByIndex(i);

        char line1[32], line2[32], line3[32];
        snprintf(line1, sizeof(line1), "Sensor %d/%d", i + 1, n);
        snprintf(line2, sizeof(line2), "%.8s..%.6s", addr, addr + 10);
        if (isnan(t) || t <= -100) {
            snprintf(line3, sizeof(line3), "Err | %s", strlen(role) ? role : "unassigned");
        } else {
            snprintf(line3, sizeof(line3), "%.1fC | %s", t, strlen(role) ? role : "unassigned");
        }
        display.showMessage(line1, line2, line3);
        delay(1500);
    }

    if (n == 0) {
        display.showMessage("Dallas 1-Wire", "No sensors found!", "Check wiring");
        delay(1500);
    }

    display.showMessage("Self-Test", "DONE", "Starting system...");
    delay(800);
}

// Functions to handle different system states and actions
void readSensors() {
    float t;

    #define READ_DALLAS(getter, var, name) \
        t = sensors.getter(); \
        if (TemperatureSensors::isValidDallasTemp(t)) { var = t; } \
        else { logBuffer.log("Sensor error: " name " = " + String(t)); }

    READ_DALLAS(getAmbientLowTemp,   ambientLowTemp,   "ambient_low")
    READ_DALLAS(getBoilerIronTemp,   boilerIronTemp,   "boiler_iron")
    READ_DALLAS(getMixSupplyTemp,    mixSupplyTemp,    "mix_supply")
    READ_DALLAS(getBoilerSupplyTemp, boilerSupplyTemp, "boiler_supply")
    READ_DALLAS(getFloorSupplyTemp,  floorSupplyTemp,  "floor_supply")
    READ_DALLAS(getAmbientHighTemp,  ambientHighTemp,  "ambient_high")
    READ_DALLAS(getFloorReturnTemp,  floorReturnTemp,  "floor_return")
    READ_DALLAS(getBoilerReturnTemp, boilerReturnTemp, "boiler_return")

    #undef READ_DALLAS

    t = sensors.getBurningTemperature();
    if (!isnan(t) && t >= 0) {
        burningTemp = t;
    } else {
        logBuffer.log("Sensor error: thermocouple = NaN");
    }

    // Request next conversion (non-blocking, result available next cycle)
    sensors.requestTemperatures();
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

// Callback to receive MQTT messages
void mqttCallback(char* topic, byte* payload, unsigned int length) {
    String topicStr = String(topic);
    payload[length] = '\0';
    String message = String((char*)payload);

    logBuffer.log("MQTT received: " + topicStr + " -> " + message);

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

    // Apply runtime settings to sensors, air intake, MQTT and manual mode
    sensors.setBurningThreshold(systemSettings.burningTempThreshold);
    sensors.setBoilerWaterThreshold(systemSettings.boilerWaterTempThreshold);
    sensors.setBoilerWaterCriticalTemp(systemSettings.boilerWaterCriticalTemp);
    manualModeTimeout = systemSettings.manualModeTimeoutMin * 60UL * 1000UL;
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

        StaticJsonDocument<768> doc;
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
        if (!isnan(burningTemp) && burningTemp >= 0) doc["burning_temp"] = burningTemp; else doc["burning_temp"] = nullptr;
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
        doc["manual_timeout_minutes"] = manualModeTimeout / 60000UL;
        if (manualMode) {
            unsigned long elapsed = millis() - manualModeStart;
            doc["manual_remaining_seconds"] =
                (manualModeTimeout > elapsed) ? (manualModeTimeout - elapsed) / 1000UL : 0;
        }

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
                            // Start new auto-tuning
                            if (airIntake.startAutoTune()) {
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

        float tcTemp = sensors.getBurningTemperature();
        if (!isnan(tcTemp) && tcTemp >= 0) doc["thermocouple_temp"] = tcTemp; else doc["thermocouple_temp"] = nullptr;
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
        int n = sensors.scanSensors();
        sensors.requestTemperatures();
        String resp = "{\"success\":true,\"count\":" + String(n) + "}";
        request->send(200, "application/json", resp);
        logBuffer.log("Dallas bus rescanned: " + String(n) + " sensors found");
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

    // GET /api/config — return all runtime-configurable settings
    webServer.on("/api/config", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        StaticJsonDocument<512> doc;
        doc["burning_temp_threshold"] = systemSettings.burningTempThreshold;
        doc["boiler_water_threshold"] = systemSettings.boilerWaterTempThreshold;
        doc["boiler_water_critical"]  = systemSettings.boilerWaterCriticalTemp;
        doc["target_burning_temp"]    = systemSettings.targetBurningTemp;
        doc["pid_kp"]                 = systemSettings.pidKp;
        doc["pid_ki"]                 = systemSettings.pidKi;
        doc["pid_kd"]                 = systemSettings.pidKd;
        doc["servo_min"]              = systemSettings.servoMin;
        doc["servo_max"]              = systemSettings.servoMax;
        doc["manual_timeout_min"]     = systemSettings.manualModeTimeoutMin;
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
        defaults["pid_kp"]                 = PID_KP;
        defaults["pid_ki"]                 = PID_KI;
        defaults["pid_kd"]                 = PID_KD;
        defaults["servo_min"]              = DEFAULT_SERVO_MIN;
        defaults["servo_max"]              = DEFAULT_SERVO_MAX;
        defaults["manual_timeout_min"]     = MANUAL_MODE_DEFAULT_TIMEOUT_MIN;
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
            StaticJsonDocument<512> doc;
            if (deserializeJson(doc, data, len)) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }

            // Update only the fields present in the request
            if (doc.containsKey("burning_temp_threshold"))
                systemSettings.burningTempThreshold    = doc["burning_temp_threshold"];
            if (doc.containsKey("boiler_water_threshold"))
                systemSettings.boilerWaterTempThreshold = doc["boiler_water_threshold"];
            if (doc.containsKey("boiler_water_critical"))
                systemSettings.boilerWaterCriticalTemp  = doc["boiler_water_critical"];
            if (doc.containsKey("target_burning_temp"))
                systemSettings.targetBurningTemp        = doc["target_burning_temp"];
            if (doc.containsKey("pid_kp"))  systemSettings.pidKp = doc["pid_kp"];
            if (doc.containsKey("pid_ki"))  systemSettings.pidKi = doc["pid_ki"];
            if (doc.containsKey("pid_kd"))  systemSettings.pidKd = doc["pid_kd"];
            if (doc.containsKey("servo_min")) systemSettings.servoMin = doc["servo_min"];
            if (doc.containsKey("servo_max")) systemSettings.servoMax = doc["servo_max"];
            if (doc.containsKey("manual_timeout_min"))
                systemSettings.manualModeTimeoutMin = doc["manual_timeout_min"];
            if (doc.containsKey("relay_debounce_min")) {
                systemSettings.relayDebounceMin = doc["relay_debounce_min"];
                unsigned long ms = (unsigned long)systemSettings.relayDebounceMin * 60000UL;
                boilerPumpRelay.setDebounce(ms);
                heatingPumpRelay.setDebounce(ms);
                fansRelay.setDebounce(ms);
                otherRelay.setDebounce(ms);
            }
            if (doc.containsKey("boiler_iron_threshold"))
                systemSettings.boilerIronThreshold = doc["boiler_iron_threshold"];
            if (doc.containsKey("floor_delta_threshold"))
                systemSettings.floorDeltaThreshold = doc["floor_delta_threshold"];

            bool mqttChanged = false;
            if (doc.containsKey("mqtt_server")) {
                strlcpy(systemSettings.mqttServer, doc["mqtt_server"] | "", sizeof(systemSettings.mqttServer));
                mqttChanged = true;
            }
            if (doc.containsKey("mqtt_port")) {
                systemSettings.mqttPort = doc["mqtt_port"].as<uint16_t>();
                mqttChanged = true;
            }
            if (doc.containsKey("mqtt_user")) {
                strlcpy(systemSettings.mqttUser, doc["mqtt_user"] | "", sizeof(systemSettings.mqttUser));
                mqttChanged = true;
            }
            if (doc.containsKey("mqtt_password") && strlen(doc["mqtt_password"] | "") > 0) {
                strlcpy(systemSettings.mqttPassword, doc["mqtt_password"] | "", sizeof(systemSettings.mqttPassword));
                mqttChanged = true;
            }
            if (doc.containsKey("mqtt_base_topic")) {
                strlcpy(systemSettings.mqttBaseTopic, doc["mqtt_base_topic"] | "", sizeof(systemSettings.mqttBaseTopic));
                mqttChanged = true;
            }

            // Apply immediately to running components
            sensors.setBurningThreshold(systemSettings.burningTempThreshold);
            sensors.setBoilerWaterThreshold(systemSettings.boilerWaterTempThreshold);
            sensors.setBoilerWaterCriticalTemp(systemSettings.boilerWaterCriticalTemp);
            airIntake.setTargetTemperature(systemSettings.targetBurningTemp);
            airIntake.setTunings(systemSettings.pidKp, systemSettings.pidKi, systemSettings.pidKd);
            airIntake.setServoMin(systemSettings.servoMin);
            airIntake.setServoMax(systemSettings.servoMax);
            manualModeTimeout = systemSettings.manualModeTimeoutMin * 60UL * 1000UL;
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
        s["pid_kp"]                 = systemSettings.pidKp;
        s["pid_ki"]                 = systemSettings.pidKi;
        s["pid_kd"]                 = systemSettings.pidKd;
        s["servo_min"]              = systemSettings.servoMin;
        s["servo_max"]              = systemSettings.servoMax;
        s["manual_timeout_min"]     = systemSettings.manualModeTimeoutMin;
        s["relay_debounce_min"]     = systemSettings.relayDebounceMin;
        s["boiler_iron_threshold"]  = systemSettings.boilerIronThreshold;
        s["floor_delta_threshold"]  = systemSettings.floorDeltaThreshold;
        s["mqtt_server"]            = systemSettings.mqttServer;
        s["mqtt_port"]              = systemSettings.mqttPort;
        s["mqtt_user"]              = systemSettings.mqttUser;
        s["mqtt_password"]          = systemSettings.mqttPassword;
        s["mqtt_base_topic"]        = systemSettings.mqttBaseTopic;

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
            StaticJsonDocument<2048> doc;
            if (deserializeJson(doc, data, len)) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }

            // Restore settings
            if (doc.containsKey("settings")) {
                JsonObjectConst s = doc["settings"].as<JsonObjectConst>();
                if (s.containsKey("burning_temp_threshold")) systemSettings.burningTempThreshold    = s["burning_temp_threshold"];
                if (s.containsKey("boiler_water_threshold")) systemSettings.boilerWaterTempThreshold = s["boiler_water_threshold"];
                if (s.containsKey("boiler_water_critical"))  systemSettings.boilerWaterCriticalTemp  = s["boiler_water_critical"];
                if (s.containsKey("target_burning_temp"))    systemSettings.targetBurningTemp        = s["target_burning_temp"];
                if (s.containsKey("pid_kp"))   systemSettings.pidKp = s["pid_kp"];
                if (s.containsKey("pid_ki"))   systemSettings.pidKi = s["pid_ki"];
                if (s.containsKey("pid_kd"))   systemSettings.pidKd = s["pid_kd"];
                if (s.containsKey("servo_min"))          systemSettings.servoMin             = s["servo_min"];
                if (s.containsKey("servo_max"))          systemSettings.servoMax             = s["servo_max"];
                if (s.containsKey("manual_timeout_min")) systemSettings.manualModeTimeoutMin = s["manual_timeout_min"];
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

                sensors.setBurningThreshold(systemSettings.burningTempThreshold);
                sensors.setBoilerWaterThreshold(systemSettings.boilerWaterTempThreshold);
                sensors.setBoilerWaterCriticalTemp(systemSettings.boilerWaterCriticalTemp);
                airIntake.setTargetTemperature(systemSettings.targetBurningTemp);
                airIntake.setTunings(systemSettings.pidKp, systemSettings.pidKi, systemSettings.pidKd);
                airIntake.setServoMin(systemSettings.servoMin);
                airIntake.setServoMax(systemSettings.servoMax);
                manualModeTimeout = systemSettings.manualModeTimeoutMin * 60UL * 1000UL;
                homeAssistant.configure(
                    systemSettings.mqttServer, systemSettings.mqttPort,
                    systemSettings.mqttUser,   systemSettings.mqttPassword,
                    systemSettings.mqttBaseTopic
                );
                systemSettings.save();
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
        manualModeTimeout = systemSettings.manualModeTimeoutMin * 60UL * 1000UL;
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

    // ── Manual override API ────────────────────────────────────────────────

    // GET /api/manual — current manual mode state
    webServer.on("/api/manual", HTTP_GET, [](AsyncWebServerRequest *request) {
        AsyncResponseStream *response = request->beginResponseStream("application/json");
        StaticJsonDocument<128> doc;
        doc["active"]           = manualMode;
        doc["timeout_minutes"]  = manualModeTimeout / 60000UL;
        if (manualMode) {
            unsigned long elapsed = millis() - manualModeStart;
            doc["remaining_seconds"] =
                (manualModeTimeout > elapsed) ? (manualModeTimeout - elapsed) / 1000UL : 0;
        }
        serializeJson(doc, *response);
        request->send(response);
    });

    // POST /api/manual
    // {"active": true}                     — enable with current timeout
    // {"active": false}                    — return to automatic immediately
    // {"active": true, "timeout_minutes": 15} — enable with custom timeout
    webServer.on("/api/manual", HTTP_POST,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            StaticJsonDocument<128> doc;
            if (deserializeJson(doc, data, len)) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }

            bool activate = doc["active"] | false;

            if (doc.containsKey("timeout_minutes")) {
                unsigned long mins = doc["timeout_minutes"].as<unsigned long>();
                if (mins > 0 && mins <= 120) {
                    manualModeTimeout = mins * 60UL * 1000UL;
                }
            }

            if (activate && !manualMode) {
                manualMode      = true;
                manualModeStart = millis();
                logBuffer.log("Manual mode ON (timeout: " + String(manualModeTimeout / 60000UL) + " min)");
            } else if (!activate && manualMode) {
                manualMode = false;
                logBuffer.log("Manual mode OFF — automatic control restored");
            }

            AsyncResponseStream *response = request->beginResponseStream("application/json");
            StaticJsonDocument<64> resp;
            resp["success"] = true;
            resp["active"]  = manualMode;
            serializeJson(resp, *response);
            request->send(response);
        }
    );

    // POST /api/manual/actuators — control relays and servo while in manual mode
    // {"boiler_pump": true, "heating_pump": false, "fans": true, "other": false, "servo": 75}
    // Any missing key is ignored (partial updates allowed)
    webServer.on("/api/manual/actuators", HTTP_POST,
        [](AsyncWebServerRequest *request) {},
        nullptr,
        [](AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
            if (!manualMode) {
                request->send(400, "application/json",
                    "{\"success\":false,\"error\":\"Not in manual mode\"}");
                return;
            }

            StaticJsonDocument<128> doc;
            if (deserializeJson(doc, data, len)) {
                request->send(400, "application/json", "{\"success\":false,\"error\":\"Invalid JSON\"}");
                return;
            }

            // Manual commands bypass debounce for immediate response
            if (doc.containsKey("boiler_pump"))  boilerPumpRelay.forceState(doc["boiler_pump"].as<bool>());
            if (doc.containsKey("heating_pump")) heatingPumpRelay.forceState(doc["heating_pump"].as<bool>());
            if (doc.containsKey("fans"))         fansRelay.forceState(doc["fans"].as<bool>());
            if (doc.containsKey("other"))        otherRelay.forceState(doc["other"].as<bool>());
            if (doc.containsKey("servo")) {
                int pos = doc["servo"].as<int>();
                pos = constrain(pos, 0, 100);
                airIntake.setPosition(pos);
            }

            // Reset the timeout on each actuator command (user is still active)
            manualModeStart = millis();

            request->send(200, "application/json", "{\"success\":true}");
        }
    );

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
        char sdaLine[16], sclLine[16];
        snprintf(sdaLine, sizeof(sdaLine), "SDA: %d", OLED_SDA);
        snprintf(sclLine, sizeof(sclLine), "SCL: %d", OLED_SCL);
        display.showMessage("Display Test", sdaLine, sclLine);
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

    // Boot self-test: display, relays, thermocouple, Dallas sensors
    runSelfTest();

    logBuffer.log("System ready!");
}

void loop() {
    unsigned long currentMillis = millis();

    // Update network manager (handles WiFi connection, MQTT reconnection and OTA)
    networkManager.update();

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

        // Read all sensors
        readSensors();

        // Check operation conditions
        isBurning = sensors.isBurning();          // update global — PID block reads it
        bool isBoilerWaterHot = sensors.isBoilerWaterHot();
        bool isBoilerWaterCritical = sensors.isBoilerWaterCritical();

        // Critical safety always overrides everything, including manual mode
        if (isBoilerWaterCritical) {
            if (manualMode) {
                manualMode = false;
                logBuffer.log("Manual mode cancelled — critical temperature override");
            }
            handleCriticalTemperature();
        } else if (!manualMode) {
            handleNormalOperation(isBurning, isBoilerWaterHot);
        }
    }

    // Manual mode timeout — revert to automatic
    if (manualMode && (currentMillis - manualModeStart >= manualModeTimeout)) {
        manualMode = false;
        logBuffer.log("Manual mode timeout — returning to automatic");
    }

    // PID + servo update — only in automatic mode
    if (currentMillis - lastControlUpdate >= PID_SAMPLE_TIME && !killSwitchActive && !manualMode) {
        lastControlUpdate = currentMillis;
        static bool prevBurning = false;
        if (isBurning) {
            if (!prevBurning) {
                // Combustion just started — reset integral so windup from the cold period
                // doesn't slam the servo to 100% on the first tick.
                airIntake.resetIntegral();
                logBuffer.log("Combustion detected — air intake PID resumed");
            }
            airIntake.update(burningTemp);
        } else {
            if (prevBurning) {
                logBuffer.log("Combustion lost — air intake closed");
            }
            airIntake.setPosition(0);   // no combustion → close damper
        }
        prevBurning = isBurning;
    }

    // Update display every 500ms
    if (currentMillis - lastDisplayUpdate >= 500) {
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