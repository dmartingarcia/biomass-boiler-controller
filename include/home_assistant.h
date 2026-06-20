#ifndef HOME_ASSISTANT_H
#define HOME_ASSISTANT_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "config.h"

// Progressive backoff on failed MQTT connects: 30s → 60s → 5min → 30min → 1h
static const unsigned long MQTT_BACKOFF_MS[] = { 30000UL, 60000UL, 300000UL, 1800000UL, 3600000UL };
static const uint8_t MQTT_BACKOFF_STEPS = 5;

class HomeAssistant {
public:
    struct Status {
        bool     connected;
        uint8_t  failCount;
        int      lastRcCode;
        unsigned long lastPublishMs;   // millis() of last successful publish, 0 if never
        unsigned long nextRetryMs;     // millis() when next attempt is scheduled
        uint32_t publishCount;
        char     server[64];
        uint16_t port;
    };

    HomeAssistant(WiFiClient& wifiClient)
        : mqttClient(wifiClient),
          mqttConnected(false),
          failCount(0),
          nextRetryMs(0),
          lastPublishMs(0),
          lastRcCode(0),
          publishCount(0),
          storedCallback(nullptr)
    {
        strlcpy(server,    MQTT_SERVER,     sizeof(server));
        port = MQTT_PORT;
        strlcpy(user,      MQTT_USER,       sizeof(user));
        strlcpy(password,  MQTT_PASSWORD,   sizeof(password));
        strlcpy(baseTopic, MQTT_BASE_TOPIC, sizeof(baseTopic));
        strlcpy(clientId,  MQTT_CLIENT_ID,  sizeof(clientId));
    }

    void configure(const char* srv, uint16_t prt,
                   const char* usr, const char* pwd, const char* topic) {
        strlcpy(server,    srv,   sizeof(server));
        port = prt;
        strlcpy(user,      usr,   sizeof(user));
        strlcpy(password,  pwd,   sizeof(password));
        strlcpy(baseTopic, topic, sizeof(baseTopic));
    }

    // Store callback; also apply it immediately if already connected.
    void setCallback(void (*cb)(char*, uint8_t*, unsigned int)) {
        storedCallback = cb;
        if (mqttConnected && cb) mqttClient.setCallback(cb);
    }

    void begin() {
        if (WiFi.status() == WL_CONNECTED) {
            mqttConnected = _connect();
            if (mqttConnected) publishDiscovery();
        }
    }

    // Call every loop iteration: keeps incoming messages flowing.
    void tick() {
        if (mqttConnected && mqttClient.connected()) {
            mqttClient.loop();
        }
    }

    // Reset backoff and schedule an immediate retry.
    void forceRetry() {
        failCount  = 0;
        nextRetryMs = 0;
        if (mqttClient.connected()) { mqttClient.disconnect(); }
        mqttConnected = false;
    }

    void forceReconnect() { forceRetry(); }

    void update(float ambientLow, float boilerIron, float mixSupply, float boilerSupply,
                float floorSupply, float ambientHigh, float floorReturn, float boilerReturn,
                float burningTemp,
                bool boilerPump, bool heatingPump, bool fans, bool otherRelay,
                float targetBurningTemp, int airIntakePosition) {

        if (!mqttConnected || !mqttClient.connected()) {
            mqttConnected = false;
            _scheduleReconnect();
            return;
        }

        StaticJsonDocument<768> doc;
        doc["ambient_low"]         = ambientLow;
        doc["boiler_iron"]         = boilerIron;
        doc["mix_supply"]          = mixSupply;
        doc["boiler_supply"]       = boilerSupply;
        doc["floor_supply"]        = floorSupply;
        doc["ambient_high"]        = ambientHigh;
        doc["floor_return"]        = floorReturn;
        doc["boiler_return"]       = boilerReturn;
        doc["burning_temp"]        = burningTemp;
        doc["boiler_pump"]         = boilerPump  ? "ON" : "OFF";
        doc["heating_pump"]        = heatingPump ? "ON" : "OFF";
        doc["fans"]                = fans        ? "ON" : "OFF";
        doc["other_relay"]         = otherRelay  ? "ON" : "OFF";
        doc["target_burning_temp"] = targetBurningTemp;
        doc["air_intake"]          = map(airIntakePosition, 0, 180, 0, 100);

        char buffer[768];
        serializeJson(doc, buffer);

        if (mqttClient.publish((String(baseTopic) + "/state").c_str(), buffer, true)) {
            lastPublishMs = millis();
            publishCount++;
        }
        mqttClient.loop();
    }

    bool isMqttConnected() const { return mqttConnected; }

    Status getStatus() const {
        Status s;
        s.connected      = mqttConnected;
        s.failCount      = failCount;
        s.lastRcCode     = lastRcCode;
        s.lastPublishMs  = lastPublishMs;
        s.nextRetryMs    = nextRetryMs;
        s.publishCount   = publishCount;
        strlcpy(s.server, server, sizeof(s.server));
        s.port           = port;
        return s;
    }

private:
    // Single non-blocking connect attempt. Updates backoff state on failure.
    bool _connect() {
        if (WiFi.status() != WL_CONNECTED) return false;

        mqttClient.setServer(server, port);
        Serial.printf("[MQTT] Connecting to %s:%u ...\n", server, port);

        if (mqttClient.connect(clientId, user, password)) {
            Serial.println("[MQTT] Connected");
            failCount   = 0;
            nextRetryMs = 0;
            lastRcCode  = 0;

            mqttClient.subscribe((String(baseTopic) + "/set/target_burning_temp").c_str());
            mqttClient.subscribe((String(baseTopic) + "/set/boiler_pump").c_str());
            mqttClient.subscribe((String(baseTopic) + "/set/heating_pump").c_str());
            mqttClient.subscribe((String(baseTopic) + "/set/fans").c_str());
            mqttClient.subscribe((String(baseTopic) + "/set/other_relay").c_str());

            if (storedCallback) mqttClient.setCallback(storedCallback);
            return true;
        }

        lastRcCode = mqttClient.state();
        failCount++;
        uint8_t step = (failCount - 1 < MQTT_BACKOFF_STEPS) ? (failCount - 1) : (MQTT_BACKOFF_STEPS - 1);
        nextRetryMs  = millis() + MQTT_BACKOFF_MS[step];

        Serial.printf("[MQTT] Failed rc=%d — next retry in %lus (fail #%u)\n",
                      lastRcCode, MQTT_BACKOFF_MS[step] / 1000UL, failCount);
        return false;
    }

    void _scheduleReconnect() {
        if (WiFi.status() != WL_CONNECTED) return;
        unsigned long now = millis();
        if (nextRetryMs == 0 || now >= nextRetryMs) {
            mqttConnected = _connect();
            if (mqttConnected) publishDiscovery();
        }
    }

    void publishDiscovery() {
        publishSensor("ambient_low",   "Ambiente Abajo",         "temperature", "°C");
        publishSensor("boiler_iron",   "Hierro Caldera",         "temperature", "°C");
        publishSensor("mix_supply",    "Ida Mezcla",             "temperature", "°C");
        publishSensor("boiler_supply", "Ida Caldera",            "temperature", "°C");
        publishSensor("floor_supply",  "Ida Suelo",              "temperature", "°C");
        publishSensor("ambient_high",  "Ambiente Arriba",        "temperature", "°C");
        publishSensor("floor_return",  "Retorno Suelo",          "temperature", "°C");
        publishSensor("boiler_return", "Retorno Caldera",        "temperature", "°C");
        publishSensor("burning_temp",  "Temperatura Combustión", "temperature", "°C");
        publishBinarySensor("boiler_pump",  "Bomba Caldera",     "connectivity");
        publishBinarySensor("heating_pump", "Bomba Calefacción", "connectivity");
        publishBinarySensor("fans",         "Ventiladores",      "connectivity");
        publishBinarySensor("other_relay",  "Otro Dispositivo",  "connectivity");
        publishNumberControl("target_burning_temp", "Temperatura Objetivo", "temperature", "°C", 60, 100, 1);
        publishSensor("air_intake", "Entrada de Aire", "power_factor", "%");
    }

    void publishSensor(const String& id, const String& name,
                       const String& deviceClass, const String& unit) {
        StaticJsonDocument<512> doc;
        doc["name"]                = name;
        doc["state_topic"]         = String(baseTopic) + "/state";
        doc["value_template"]      = "{{ value_json." + id + " }}";
        doc["unique_id"]           = String("lumber_boiler_") + id;
        doc["device_class"]        = deviceClass;
        doc["unit_of_measurement"] = unit;
        _addDevice(doc);
        char buf[512]; serializeJson(doc, buf);
        mqttClient.publish((String("homeassistant/sensor/") + clientId + "/" + id + "/config").c_str(), buf, true);
    }

    void publishBinarySensor(const String& id, const String& name, const String& deviceClass) {
        StaticJsonDocument<512> doc;
        doc["name"]           = name;
        doc["state_topic"]    = String(baseTopic) + "/state";
        doc["value_template"] = "{{ value_json." + id + " }}";
        doc["unique_id"]      = String("lumber_boiler_") + id;
        doc["device_class"]   = deviceClass;
        _addDevice(doc);
        char buf[512]; serializeJson(doc, buf);
        mqttClient.publish((String("homeassistant/binary_sensor/") + clientId + "/" + id + "/config").c_str(), buf, true);
    }

    void publishNumberControl(const String& id, const String& name, const String& deviceClass,
                              const String& unit, float minV, float maxV, float step) {
        StaticJsonDocument<512> doc;
        doc["name"]                = name;
        doc["command_topic"]       = String(baseTopic) + "/set/" + id;
        doc["state_topic"]         = String(baseTopic) + "/state";
        doc["value_template"]      = "{{ value_json." + id + " }}";
        doc["unique_id"]           = String("lumber_boiler_") + id;
        doc["device_class"]        = deviceClass;
        doc["unit_of_measurement"] = unit;
        doc["min"]  = minV;
        doc["max"]  = maxV;
        doc["step"] = step;
        _addDevice(doc);
        char buf[512]; serializeJson(doc, buf);
        mqttClient.publish((String("homeassistant/number/") + clientId + "/" + id + "/config").c_str(), buf, true);
    }

    void _addDevice(JsonDocument& doc) {
        JsonObject dev = doc.createNestedObject("device");
        dev["identifiers"] = clientId;
        dev["name"]        = "Caldera de Biomasa";
        dev["model"]       = "Lumber Boiler Manager";
        dev["manufacturer"]= "ESP32";
    }

    PubSubClient mqttClient;
    bool         mqttConnected;
    uint8_t      failCount;
    unsigned long nextRetryMs;
    unsigned long lastPublishMs;
    int           lastRcCode;
    uint32_t      publishCount;
    void (*storedCallback)(char*, uint8_t*, unsigned int);

    char     server[64];
    uint16_t port;
    char     user[32];
    char     password[64];
    char     baseTopic[32];
    char     clientId[32];
};

#endif // HOME_ASSISTANT_H
