#ifndef NETWORK_MANAGER_H
#define NETWORK_MANAGER_H

#include <Arduino.h>
#include <WiFi.h>
#include <ArduinoOTA.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "config.h"
#include "secrets.h"
#include "home_assistant.h"
#include "log_buffer.h"
#include "fs_helper.h"

struct WifiConfig {
    char ssid[64];
    char password[64];
    bool staticIp;
    char ip[16];
    char gateway[16];
    char subnet[16];
    char dns[16];

    WifiConfig() {
        ssid[0]     = '\0';
        password[0] = '\0';
        staticIp    = false;
        strlcpy(ip,      DEFAULT_STATIC_IP,      sizeof(ip));
        strlcpy(gateway, DEFAULT_STATIC_GATEWAY, sizeof(gateway));
        strlcpy(subnet,  DEFAULT_STATIC_SUBNET,  sizeof(subnet));
        strlcpy(dns,     DEFAULT_STATIC_DNS,     sizeof(dns));
    }
};

class NetworkManager {
public:
    enum WiFiState {
        WIFI_DISCONNECTED,
        WIFI_CONNECTING,
        WIFI_CONNECTED,
        WIFI_AP_MODE
    };

    enum ScanState {
        SCAN_IDLE,
        SCAN_RUNNING,
        SCAN_DONE
    };

    NetworkManager() {
        wifiState           = WIFI_DISCONNECTED;
        connectionStartTime = 0;
        connectionAttempt   = 0;
        lastWifiCheckTime   = 0;
        wifiWasConnected    = false;
        scanState           = SCAN_IDLE;
        onWifiConnected     = nullptr;
        onWifiDisconnected  = nullptr;
        onAPModeStarted     = nullptr;
        logBuffer           = nullptr;
        homeAssistant       = nullptr;
    }

    // LittleFS must be mounted before calling begin()
    void begin(LogBuffer* logBufferPtr = nullptr, HomeAssistant* homeAssistantPtr = nullptr) {
        logBuffer     = logBufferPtr;
        homeAssistant = homeAssistantPtr;

        WiFi.hostname(HOSTNAME);

        if (!loadConfig(currentConfig)) {
            // No config file — migrate from compile-time defaults if present
            if (strlen(WIFI_SSID) > 0) {
                strlcpy(currentConfig.ssid,     WIFI_SSID,     sizeof(currentConfig.ssid));
                strlcpy(currentConfig.password, WIFI_PASSWORD, sizeof(currentConfig.password));
                currentConfig.staticIp = true;
                saveConfig(currentConfig);
                if (logBuffer) logBuffer->log("WiFi config created from firmware defaults");
            }
        }

        if (strlen(currentConfig.ssid) == 0) {
            startAPMode();
        } else {
            startWiFiConnection();
        }

        setupOTA();
    }

    void update() {
        unsigned long currentMillis = millis();

        if (wifiState == WIFI_CONNECTED) {
            ArduinoOTA.handle();
        }

        if (wifiState == WIFI_CONNECTING) {
            handleWiFiConnection();
        }

        // Drive async WiFi scan
        if (scanState == SCAN_RUNNING) {
            int n = WiFi.scanComplete();
            if (n >= 0) {
                buildScanJson(n);
                WiFi.scanDelete();
                scanState = SCAN_DONE;
            } else if (n == WIFI_SCAN_FAILED) {
                scanJson  = "{\"networks\":[],\"error\":\"scan failed\"}";
                scanState = SCAN_IDLE;
            }
        }

        if (currentMillis - lastWifiCheckTime >= WIFI_RECONNECT_INTERVAL) {
            lastWifiCheckTime = currentMillis;

            if (wifiState == WIFI_CONNECTED) {
                checkWifiStateChanges(currentMillis);
                if (homeAssistant && !homeAssistant->isMqttConnected()) {
                    homeAssistant->begin();
                    if (homeAssistant->isMqttConnected()) {
                        if (logBuffer) logBuffer->log("Reconnected to MQTT");
                    }
                }
            } else if (wifiState == WIFI_DISCONNECTED) {
                if (logBuffer) logBuffer->log("WiFi disconnected, reconnecting...");
                startWiFiConnection();
            }
            else if (wifiState == WIFI_AP_MODE) {
                // A saved network exists, so the AP may only be here because the router was
                // slow to come back (e.g. after a power cut). Retry unless somebody is
                // currently connected to the setup AP.
                if (strlen(currentConfig.ssid) > 0 && WiFi.softAPgetStationNum() == 0) {
                    if (logBuffer) logBuffer->log("AP mode: retrying saved WiFi network...");
                    startWiFiConnection();
                }
            }
        }
    }

    bool isConnected()  const { return wifiState == WIFI_CONNECTED; }
    bool isConnecting() const { return wifiState == WIFI_CONNECTING; }
    bool isAPMode()     const { return wifiState == WIFI_AP_MODE; }

    IPAddress getIP()        const { return WiFi.localIP(); }
    IPAddress getAPIP()      const { return WiFi.softAPIP(); }
    String    getSSID()      const { return WiFi.SSID(); }
    String    getIPAddress() const { return WiFi.localIP().toString(); }
    int       getWifiSignalStrength() const { return WiFi.RSSI(); }
    bool      isMqttConnected() const { return homeAssistant ? homeAssistant->isMqttConnected() : false; }

    void setOnWifiConnectedCallback(void (*cb)())    { onWifiConnected    = cb; }
    void setOnWifiDisconnectedCallback(void (*cb)()) { onWifiDisconnected = cb; }
    void setOnAPModeCallback(void (*cb)())           { onAPModeStarted    = cb; }

    void showNetworkInfo() { if (onWifiConnected) onWifiConnected(); }

    // ── Config management ──────────────────────────────────────────────────

    bool loadConfig(WifiConfig& cfg) {
        if (!LittleFS.exists(WIFI_CONFIG_FILE)) return false;
        File f = LittleFS.open(WIFI_CONFIG_FILE, "r");
        if (!f) return false;

        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, f);
        f.close();
        if (err) return false;

        strlcpy(cfg.ssid,     doc["ssid"]      | "", sizeof(cfg.ssid));
        strlcpy(cfg.password, doc["password"]  | "", sizeof(cfg.password));
        cfg.staticIp = doc["static_ip"] | false;
        strlcpy(cfg.ip,      doc["ip"]      | DEFAULT_STATIC_IP,      sizeof(cfg.ip));
        strlcpy(cfg.gateway, doc["gateway"] | DEFAULT_STATIC_GATEWAY, sizeof(cfg.gateway));
        strlcpy(cfg.subnet,  doc["subnet"]  | DEFAULT_STATIC_SUBNET, sizeof(cfg.subnet));
        strlcpy(cfg.dns,     doc["dns"]     | DEFAULT_STATIC_DNS,    sizeof(cfg.dns));

        return strlen(cfg.ssid) > 0;
    }

    bool saveConfig(const WifiConfig& cfg) {
        File f = LittleFS.open(WIFI_CONFIG_FILE, "w");
        if (!f) return false;

        StaticJsonDocument<512> doc;
        doc["ssid"]      = cfg.ssid;
        doc["password"]  = cfg.password;
        doc["static_ip"] = cfg.staticIp;
        doc["ip"]        = cfg.ip;
        doc["gateway"]   = cfg.gateway;
        doc["subnet"]    = cfg.subnet;
        doc["dns"]       = cfg.dns;

        serializeJson(doc, f);
        f.close();
        return true;
    }

    bool deleteConfig() { return LittleFS.remove(WIFI_CONFIG_FILE); }

    WifiConfig& getConfig() { return currentConfig; }

    // ── WiFi scan (async) ──────────────────────────────────────────────────

    // Starts a background scan; returns false if a scan is already running
    bool startScan() {
        if (scanState == SCAN_RUNNING) return false;
        scanJson  = "";
        scanState = SCAN_RUNNING;
        WiFi.scanNetworks(true); // non-blocking
        return true;
    }

    bool     isScanRunning() const { return scanState == SCAN_RUNNING; }
    bool     isScanDone()    const { return scanState == SCAN_DONE; }
    String   getScanJson()   const { return scanJson; }

    // ── Connection management ──────────────────────────────────────────────

    void startAPMode() {
        Serial.println("Starting AP mode: " + String(AP_SSID));
        if (logBuffer) logBuffer->log("Starting AP: " + String(AP_SSID));

        WiFi.disconnect();
        // AP_STA so we can still scan for networks from the setup page
        WiFi.mode(WIFI_AP_STA);

        if (strlen(AP_PASSWORD) > 0) {
            WiFi.softAP(AP_SSID, AP_PASSWORD);
        } else {
            WiFi.softAP(AP_SSID);
        }

        IPAddress apIP = WiFi.softAPIP();
        Serial.println("AP IP: " + apIP.toString());
        if (logBuffer) logBuffer->log("AP IP: " + apIP.toString());

        wifiState = WIFI_AP_MODE;
        if (onAPModeStarted) onAPModeStarted();
    }

    void startWiFiConnection() {
        if (wifiState == WIFI_CONNECTING) return;

        WiFi.disconnect();
        WiFi.mode(WIFI_STA);
        WiFi.hostname(HOSTNAME);
        wifiState           = WIFI_CONNECTING;
        connectionStartTime = millis();
        connectionAttempt   = 0;

        if (currentConfig.staticIp) {
            IPAddress ip, gw, sn, dns;
            ip.fromString(currentConfig.ip);
            gw.fromString(currentConfig.gateway);
            sn.fromString(currentConfig.subnet);
            dns.fromString(currentConfig.dns);
            WiFi.config(ip, gw, sn, dns);
        }

        Serial.println("Connecting to: " + String(currentConfig.ssid));
        WiFi.begin(currentConfig.ssid, currentConfig.password);
    }

private:
    void checkWifiStateChanges(unsigned long /*currentMillis*/) {
        bool physically = (WiFi.status() == WL_CONNECTED);

        if (physically && !wifiWasConnected) {
            wifiWasConnected = true;
            String ip = getIP().toString();
            Serial.println("Connected to WiFi, IP: " + ip);
            if (logBuffer) logBuffer->log("Connected to WiFi, IP: " + ip);
            if (onWifiConnected) onWifiConnected();
            if (homeAssistant) {
                homeAssistant->begin();
                if (homeAssistant->isMqttConnected()) {
                    if (logBuffer) logBuffer->log("Home Assistant integration started");
                } else {
                    if (logBuffer) logBuffer->log("Could not connect to MQTT");
                }
            }
        } else if (!physically && wifiWasConnected) {
            wifiWasConnected = false;
            wifiState        = WIFI_DISCONNECTED;
            Serial.println("WiFi connection lost");
            if (logBuffer) logBuffer->log("WiFi connection lost");
            if (onWifiDisconnected) onWifiDisconnected();
        }
    }

    void handleWiFiConnection() {
        unsigned long now = millis();

        if (WiFi.status() == WL_CONNECTED) {
            Serial.println("\nConnected, IP: " + WiFi.localIP().toString());
            wifiState = WIFI_CONNECTED;
            checkWifiStateChanges(now);
            return;
        }

        if (now - connectionStartTime > 3000) {
            connectionAttempt++;

            if (connectionAttempt >= MAX_WIFI_ATTEMPTS) {
                Serial.println("\nWiFi failed — starting AP mode");
                if (logBuffer) logBuffer->log(
                    "WiFi failed after " + String(MAX_WIFI_ATTEMPTS) + " attempts — AP mode");
                startAPMode();
                return;
            }

            Serial.print(".");
            connectionStartTime = now;
            WiFi.disconnect();
            WiFi.begin(currentConfig.ssid, currentConfig.password);
        }
    }

    void buildScanJson(int count) {
        StaticJsonDocument<2048> doc;
        JsonArray arr = doc.createNestedArray("networks");
        for (int i = 0; i < count && i < 20; i++) {
            JsonObject net = arr.createNestedObject();
            net["ssid"]      = WiFi.SSID(i);
            net["rssi"]      = WiFi.RSSI(i);
            net["encrypted"] = (WiFi.encryptionType(i) != WIFI_AUTH_OPEN);
        }
        serializeJson(doc, scanJson);
    }

    void setupOTA() {
        ArduinoOTA.setHostname(HOSTNAME);
        ArduinoOTA.setPassword(OTA_PASSWORD);

        ArduinoOTA.onStart([]() {
            String type = (ArduinoOTA.getCommand() == U_FLASH) ? "sketch" : "filesystem";
            Serial.println("OTA start: " + type);
        });
        ArduinoOTA.onEnd([]() {
            Serial.println("\nOTA complete");
        });
        ArduinoOTA.onProgress([](unsigned int progress, unsigned int total) {
            Serial.printf("OTA: %u%%\r", (progress / (total / 100)));
        });
        ArduinoOTA.onError([](ota_error_t error) {
            Serial.printf("OTA Error[%u]: ", error);
            if      (error == OTA_AUTH_ERROR)    Serial.println("Auth Failed");
            else if (error == OTA_BEGIN_ERROR)   Serial.println("Begin Failed");
            else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
            else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
            else if (error == OTA_END_ERROR)     Serial.println("End Failed");
        });

        ArduinoOTA.begin();
        Serial.println("OTA configured");
    }

    WiFiState  wifiState;
    ScanState  scanState;
    String     scanJson;

    unsigned long connectionStartTime;
    int           connectionAttempt;
    unsigned long lastWifiCheckTime;
    bool          wifiWasConnected;

    WifiConfig currentConfig;

    void (*onWifiConnected)();
    void (*onWifiDisconnected)();
    void (*onAPModeStarted)();

    LogBuffer*    logBuffer;
    HomeAssistant* homeAssistant;

    static const int MAX_WIFI_ATTEMPTS = 10;
};

#endif // NETWORK_MANAGER_H
