#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include "config.h"
#include "secrets.h"
#include "home_assistant.h"
#include "log_buffer.h"
#include "fs_helper.h"

class NetworkManager {
public:
    enum WiFiState {
        WIFI_DISCONNECTED,
        WIFI_CONNECTING,
        WIFI_CONNECTED
    };

    public:
    NetworkManager() {
        wifiState = WIFI_DISCONNECTED;
        connectionStartTime = 0;
        connectionAttempt = 0;
        lastWifiCheckTime = 0;
        wifiWasConnected = false;
        onWifiConnected = nullptr;
        onWifiDisconnected = nullptr;
    }

    void begin(LogBuffer* logBufferPtr = nullptr, HomeAssistant* homeAssistantPtr = nullptr) {
        logBuffer = logBufferPtr;
        homeAssistant = homeAssistantPtr;

        // Configure WiFi in station mode
        WiFi.mode(WIFI_STA);
        WiFi.hostname(HOSTNAME);

        // Start WiFi connection process
        startWiFiConnection();

        // Initialize LittleFS filesystem with our helper
        if (!FSHelper::initializeLittleFS()) {
            Serial.println("Error initializing LittleFS");
            if (logBuffer) logBuffer->log("Error initializing LittleFS");
        }

        // Configure and start ArduinoOTA
        setupOTA();
    }    // This method should be called regularly from loop()
    void update() {
        unsigned long currentMillis = millis();

        // Handle OTA if connected
        if (wifiState == WIFI_CONNECTED) {
            ArduinoOTA.handle();
        }

        // Handle connection process if trying to connect
        if (wifiState == WIFI_CONNECTING) {
            handleWiFiConnection();
        }

        // Periodically check WiFi connection status
        if (currentMillis - lastWifiCheckTime >= WIFI_RECONNECT_INTERVAL) {
            lastWifiCheckTime = currentMillis;

            // Try to reconnect WiFi if connection was lost and not already trying to connect
            if (WiFi.status() != WL_CONNECTED && wifiState != WIFI_CONNECTED && wifiState != WIFI_CONNECTING) {
                Serial.println("WiFi connection lost. Attempting to reconnect...");
                if (logBuffer) logBuffer->log("WiFi connection lost. Attempting to reconnect...");
                startWiFiConnection();
            }

            // Check for WiFi state changes
            checkWifiStateChanges(currentMillis);

            // Try to reconnect MQTT if WiFi is connected but MQTT isn't
            if (homeAssistant && wifiState == WIFI_CONNECTED && !homeAssistant->isMqttConnected()) {
                homeAssistant->begin();
                if (homeAssistant->isMqttConnected()) {
                    if (logBuffer) logBuffer->log("Reconnected to MQTT");
                }
            }
        }
    }

    bool isConnected() const {
        return wifiState == WIFI_CONNECTED;
    }

    bool isConnecting() const {
        return wifiState == WIFI_CONNECTING;
    }

    // Function to get the IP address
    IPAddress getIP() const {
        return WiFi.localIP();
    }

    // Get SSID of connected network
    String getSSID() const {
        return WiFi.SSID();
    }

    // Get IP address as string
    String getIPAddress() const {
        return WiFi.localIP().toString();
    }

    // Get WiFi signal strength
    int getWifiSignalStrength() const {
        return WiFi.RSSI();
    }

    // Get MQTT connection status
    bool isMqttConnected() const {
        return homeAssistant ? homeAssistant->isMqttConnected() : false;
    }    // Setters for callbacks
    void setOnWifiConnectedCallback(void (*callback)()) {
        onWifiConnected = callback;
    }

    void setOnWifiDisconnectedCallback(void (*callback)()) {
        onWifiDisconnected = callback;
    }

    // Function to force display of network information
    void showNetworkInfo() {
        if (onWifiConnected) {
            onWifiConnected();
        }
    }

    // Start a new connection attempt
    void startWiFiConnection() {
        if (wifiState == WIFI_CONNECTING) {
            return; // Already trying to connect
        }

        WiFi.disconnect();
        wifiState = WIFI_CONNECTING;
        connectionStartTime = millis();
        connectionAttempt = 0;

        Serial.println("Starting WiFi connection...");
        WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
    }

private:
    void checkWifiStateChanges(unsigned long currentMillis) {
        // Check for WiFi connection state changes
        if (wifiState == WIFI_CONNECTED && !wifiWasConnected) {
            // WiFi just connected
            wifiWasConnected = true;
            String ipAddress = getIP().toString();
            Serial.println("Connected to WiFi, IP: " + ipAddress);
            if (logBuffer) logBuffer->log("Connected to WiFi, IP: " + ipAddress);

            // Call the connection callback if defined
            if (onWifiConnected) {
                onWifiConnected();
            }

            // Try to connect to MQTT
            if (homeAssistant) {
                homeAssistant->begin();
                if (homeAssistant->isMqttConnected()) {
                    if (logBuffer) logBuffer->log("Home Assistant integration started");
                } else {
                    if (logBuffer) logBuffer->log("Could not connect to MQTT - Continuing without Home Assistant");
                }
            }
        } else if (wifiState != WIFI_CONNECTED && wifiWasConnected) {
            // WiFi just disconnected
            wifiWasConnected = false;
            Serial.println("WiFi connection lost");
            if (logBuffer) logBuffer->log("WiFi connection lost");

            // Call the disconnection callback if defined
            if (onWifiDisconnected) {
                onWifiDisconnected();
            }
        }
    }
    void handleWiFiConnection() {
        unsigned long currentTime = millis();

        // Check connection status
        if (WiFi.status() == WL_CONNECTED) {
            // Connection successful
            Serial.println();
            Serial.print("Connected to WiFi, IP: ");
            Serial.println(WiFi.localIP());

            wifiState = WIFI_CONNECTED;
            Serial.println("OTA service started");
            return;
        }

        // Check for timeout (3 seconds per attempt)
        if (currentTime - connectionStartTime > 3000) {
            connectionAttempt++;

            if (connectionAttempt >= MAX_WIFI_ATTEMPTS) {
                // Max attempts reached, move to disconnected state
                Serial.println("\nFailed to connect to WiFi after maximum attempts");
                Serial.println("System will continue without connectivity");
                wifiState = WIFI_DISCONNECTED;
                return;
            }

            // Try again
            Serial.print(".");
            connectionStartTime = currentTime;
            WiFi.disconnect();
            WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
        }
    }

    void setupOTA() {
        // Configure OTA
        ArduinoOTA.setHostname(HOSTNAME);
        ArduinoOTA.setPassword(OTA_PASSWORD);

        ArduinoOTA.onStart([]() {
            String type;
            if (ArduinoOTA.getCommand() == U_FLASH) {
                type = "sketch";
            } else { // U_SPIFFS
                type = "filesystem";
            }
            Serial.println("Starting OTA update: " + type);
        });

        ArduinoOTA.onEnd([]() {
            Serial.println("\nOTA update completed");
        });

        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("Progress: %u%%\r", (progress / (total / 100)));
        });

        ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("Error OTA[%u]: ", error);
            if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR) Serial.println("End Failed");
        });

        ArduinoOTA.begin();
        Serial.println("OTA configured");
    }

    WiFiState wifiState;
    unsigned long connectionStartTime;
    int connectionAttempt;
    unsigned long lastWifiCheckTime;
    bool wifiWasConnected;

    // Callbacks
    void (*onWifiConnected)();
    void (*onWifiDisconnected)();

    // References to other components
    LogBuffer* logBuffer;
    HomeAssistant* homeAssistant;

    static const int MAX_WIFI_ATTEMPTS = 10; // Maximum number of connection attempts
};

#endif // NETWORK_MANAGER_H
