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

// Creación del servidor web directamente en main.cpp
AsyncWebServer webServer(WEB_SERVER_PORT);

// Declaration of global objects
TemperatureSensors sensors;

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

// Variables to store sensor values
float boilerWaterTemp = 0;
float heatingTemp = 0;
float burningTemp = 0;
float ambientTemp = 0;

// Functions to handle different system states and actions
void readSensors() {
    // Read temperatures from sensors
    boilerWaterTemp = sensors.getBoilerWaterTemperature();
    heatingTemp = sensors.getHeatingTemperature();
    burningTemp = sensors.getBurningTemperature();
    ambientTemp = sensors.getAmbientTemperature();
}

void handleCriticalTemperature() {
    // Activate all pumps to evacuate heat
    boilerPumpRelay.setState(true);
    heatingPumpRelay.setState(true);

    // Completely close the air intake
    airIntake.setPosition(0);

    // Activate fans for cooling
    fansRelay.setState(true);

    // Log critical event if it's the first time it's activated
    if (!killSwitchActive) {
        logBuffer.log("ALERT! Critical water temperature: " + String(boilerWaterTemp) + "°C - Emergency mode activated");
        killSwitchActive = true;
    }
}

void handleNormalOperation(bool isBurning, bool isBoilerWaterHot) {
    // Restore normal operation if we were in critical mode
    if (killSwitchActive) {
        logBuffer.log("Water temperature normalized: " + String(boilerWaterTemp) + "°C - Normal mode restored");
        killSwitchActive = false;
    }

    // Update relay status based on normal conditions
    if (isBurning) {
        // If there is combustion, activate boiler pump and fans
        boilerPumpRelay.setState(true);
        fansRelay.setState(true);

        if (isBoilerWaterHot) {
            // If boiler water is hot, activate heating pump
            heatingPumpRelay.setState(true);
        } else {
            // If water is not hot enough, deactivate heating pump
            heatingPumpRelay.setState(false);
        }
    } else {
        // If there is no combustion, deactivate everything
        boilerPumpRelay.setState(false);
        fansRelay.setState(false);
        heatingPumpRelay.setState(false);
    }
}

void turnOffAllRelays() {
    boilerPumpRelay.setState(false);
    heatingPumpRelay.setState(false);
    fansRelay.setState(false);
    otherRelay.setState(false);
}

void updateHomeAssistant() {
    // Send data to Home Assistant only if there is MQTT connection
    if (networkManager.isConnected() && homeAssistant.isMqttConnected()) {
        homeAssistant.update(
            boilerWaterTemp,
            heatingTemp,
            burningTemp,
            ambientTemp,
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

// Callback for when WiFi connects
void onWiFiConnected() {
    display.setScreen(Display::SCREEN_NETWORK_INFO);
}

// Callback for when WiFi disconnects
void onWiFiDisconnected() {
    // Could update display to show disconnected status
}

void setup() {
    // Start serial communication
    Serial.begin(115200);
    Serial.println("\n\n--- Lumber Boiler Manager ---");
    Serial.println("Starting...");

    // Initialize log system
    logBuffer.begin();
    logBuffer.log("System started");

    // Initialize temperature sensors
    sensors.begin();
    logBuffer.log("Temperature sensors initialized");

    // Initialize relays
    boilerPumpRelay.begin();
    heatingPumpRelay.begin();
    fansRelay.begin();
    otherRelay.begin();
    logBuffer.log("Relays initialized");

    // Initialize air intake servo
    airIntake.begin();
    logBuffer.log("Air intake control initialized");

    // Initialize GLCD display
    display.begin();
    logBuffer.log("Display initialized");

    // Set up WiFi connection callbacks
    networkManager.setOnWifiConnectedCallback(onWiFiConnected);
    networkManager.setOnWifiDisconnectedCallback(onWiFiDisconnected);

    // Configure MQTT callback
    homeAssistant.setCallback(mqttCallback);

    // Initialize WiFi connectivity and web server with references to other components
    networkManager.begin(&logBuffer, &homeAssistant);

    // Configuración de rutas web estáticas desde LittleFS
    webServer.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

    // API: Obtener estado del sistema
    webServer.on("/api/status", HTTP_GET, [](AsyncWebServerRequest *request){
        // Create JSON response with current system state
        AsyncResponseStream *response = request->beginResponseStream("application/json");

        StaticJsonDocument<512> doc;
        doc["boiler_water_temp"] = boilerWaterTemp;
        doc["heating_temp"] = heatingTemp;
        doc["burning_temp"] = burningTemp;
        doc["ambient_temp"] = ambientTemp;
        doc["boiler_pump"] = boilerPumpRelay.getState();
        doc["heating_pump"] = heatingPumpRelay.getState();
        doc["fans"] = fansRelay.getState();
        doc["other"] = otherRelay.getState();
        doc["target_burning_temp"] = airIntake.getTargetTemperature();
        doc["air_intake"] = airIntake.getCurrentOutput();
        doc["auto_tuning"] = airIntake.isAutoTuning();
        doc["killswitch_active"] = killSwitchActive;

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

    // Iniciar el servidor web después de configurar todas las rutas
    webServer.begin();
    logBuffer.log("Web server started on port " + String(WEB_SERVER_PORT));

    // Set display to toggle screens every 5 seconds
    display.setScreenToggleInterval(5000);

    logBuffer.log("System ready!");
}

void loop() {
    unsigned long currentMillis = millis();

    // Update network manager (handles WiFi connection, MQTT reconnection and OTA)
    networkManager.update();

    // Read sensors every second
    if (currentMillis - lastSensorRead >= 1000) {
        lastSensorRead = currentMillis;

        // Read all sensors
        readSensors();

        // Check operation conditions
        bool isBurning = sensors.isBurning();
        bool isBoilerWaterHot = sensors.isBoilerWaterHot();
        bool isBoilerWaterCritical = sensors.isBoilerWaterCritical();

        // Handle different system states
        if (isBoilerWaterCritical) {
            // Emergency mode - critical temperature
            handleCriticalTemperature();
        } else {
            // Normal operation
            handleNormalOperation(isBurning, isBoilerWaterHot);
        }
    }

    // Update air intake control
    if (currentMillis - lastControlUpdate >= PID_SAMPLE_TIME && !killSwitchActive) {
        lastControlUpdate = currentMillis;

        // In normal mode, update PID and servo
        airIntake.update(burningTemp);
    }

    // Update display every 500ms
    if (currentMillis - lastDisplayUpdate >= 500) {
        lastDisplayUpdate = currentMillis;

        // Update display with current values
        display.update(
            boilerWaterTemp,
            heatingTemp,
            burningTemp,
            ambientTemp,
            boilerPumpRelay.getState(),
            heatingPumpRelay.getState(),
            fansRelay.getState(),
            airIntake.getTargetTemperature(),
            airIntake.getCurrentOutput()
        );
    }

    // Update MQTT for Home Assistant every 10 seconds
    if (currentMillis - lastMqttUpdate >= MQTT_PUBLISH_INTERVAL) {
        lastMqttUpdate = currentMillis;

        // Only try to update MQTT if there is WiFi
        if (networkManager.isConnected()) {
            updateHomeAssistant();
        }
    }
}