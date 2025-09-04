#ifndef HOME_ASSISTANT_H
#define HOME_ASSISTANT_H

#include <Arduino.h>
#include <PubSubClient.h>
#include <WiFi.h>
#include <ArduinoJson.h>
#include "config.h"

class HomeAssistant {
public:
    HomeAssistant(WiFiClient& wifiClient) : mqttClient(wifiClient), mqttConnected(false) {}

    void begin() {
        // Only initialize MQTT if there is WiFi connection
        if (WiFi.status() == WL_CONNECTED) {
            mqttClient.setServer(MQTT_SERVER, MQTT_PORT);
            mqttConnected = connect();

            if (mqttConnected) {
                publishDiscovery();
                Serial.println("MQTT and Home Assistant integration completed");
            }
        } else {
            mqttConnected = false;
            Serial.println("No WiFi connection, MQTT disabled");
        }
    }

    void update(float boilerWaterTemp, float heatingTemp, float burningTemp, float ambientTemp,
                bool boilerPump, bool heatingPump, bool fans, bool otherRelay,
                float targetBurningTemp, int airIntakePosition) {

        // If there is no WiFi or MQTT, exit immediately
        if (WiFi.status() != WL_CONNECTED || !mqttConnected) {
            // Try to reconnect to WiFi and MQTT periodically
            static unsigned long lastReconnectAttempt = 0;
            unsigned long now = millis();

            // Try to reconnect every 30 seconds
            if (now - lastReconnectAttempt > 30000) {
                lastReconnectAttempt = now;

                // If WiFi is disconnected, we don't try MQTT
                if (WiFi.status() != WL_CONNECTED) {
                    return;
                }

                // Try to reconnect MQTT
                mqttConnected = connect();
                if (mqttConnected) {
                    Serial.println("Reconnected to MQTT");
                    publishDiscovery(); // Republish on reconnect
                }
            }
            return;
        }

        // Comprobar conexión MQTT
        if (!mqttClient.connected()) {
            mqttConnected = connect();
            if (!mqttConnected) {
                return; // Si no podemos conectar, salimos
            }
        }

        // Crear JSON para los datos de sensores
        StaticJsonDocument<512> doc;
        doc["boiler_water_temp"] = boilerWaterTemp;
        doc["heating_temp"] = heatingTemp;
        doc["burning_temp"] = burningTemp;
        doc["ambient_temp"] = ambientTemp;
        doc["boiler_pump"] = boilerPump ? "ON" : "OFF";
        doc["heating_pump"] = heatingPump ? "ON" : "OFF";
        doc["fans"] = fans ? "ON" : "OFF";
        doc["other_relay"] = otherRelay ? "ON" : "OFF";
        doc["target_burning_temp"] = targetBurningTemp;
        doc["air_intake"] = map(airIntakePosition, 0, 180, 0, 100); // Convertir a porcentaje

        char buffer[512];
        serializeJson(doc, buffer);

        // Publicar estado
        mqttClient.publish(String(MQTT_BASE_TOPIC + String("/state")).c_str(), buffer, true);

        // Manejar mensajes entrantes
        mqttClient.loop();
    }

    void setCallback(MQTT_CALLBACK_SIGNATURE) {
        // Solo establecer el callback si hay conexión MQTT
        if (mqttConnected) {
            mqttClient.setCallback(callback);
        }
    }

    bool isMqttConnected() const {
        return mqttConnected;
    }

private:
    bool connect() {
        // Check if WiFi is connected
        if (WiFi.status() != WL_CONNECTED) {
            Serial.println("No WiFi connection, cannot connect to MQTT");
            return false;
        }

        // Try to connect to MQTT server
        Serial.print("Connecting to MQTT...");

        int attempts = 0;
        const int maxAttempts = 3; // Limit attempts to avoid blocking

        while (!mqttClient.connected() && attempts < maxAttempts) {
            if (mqttClient.connect(MQTT_CLIENT_ID, MQTT_USER, MQTT_PASSWORD)) {
                Serial.println("connected");

                // Subscribe to control topics
                mqttClient.subscribe(String(MQTT_BASE_TOPIC + String("/set/target_burning_temp")).c_str());
                mqttClient.subscribe(String(MQTT_BASE_TOPIC + String("/set/boiler_pump")).c_str());
                mqttClient.subscribe(String(MQTT_BASE_TOPIC + String("/set/heating_pump")).c_str());
                mqttClient.subscribe(String(MQTT_BASE_TOPIC + String("/set/fans")).c_str());
                mqttClient.subscribe(String(MQTT_BASE_TOPIC + String("/set/other_relay")).c_str());

                return true;
            } else {
                Serial.print("failed, rc=");
                Serial.print(mqttClient.state());
                Serial.println(" Trying again...");
                delay(1000);
                attempts++;
            }
        }

        if (attempts >= maxAttempts) {
            Serial.println("Could not connect to MQTT after several attempts - System will continue without MQTT");
        }

        return false;
    }

    void publishDiscovery() {
        // Publicar información de descubrimiento para Home Assistant
        // Esto permite que HA detecte automáticamente los sensores y controles

        // Temperatura del agua de la caldera
        publishSensor("boiler_water_temp", "Temperatura Agua Caldera", "temperature", "°C");

        // Temperatura de calefacción
        publishSensor("heating_temp", "Temperatura Calefacción", "temperature", "°C");

        // Temperatura de combustión
        publishSensor("burning_temp", "Temperatura Combustión", "temperature", "°C");

        // Temperatura ambiente
        publishSensor("ambient_temp", "Temperatura Ambiente", "temperature", "°C");

        // Bomba de la caldera
        publishBinarySensor("boiler_pump", "Bomba Caldera", "connectivity");

        // Bomba de calefacción
        publishBinarySensor("heating_pump", "Bomba Calefacción", "connectivity");

        // Ventiladores
        publishBinarySensor("fans", "Ventiladores", "connectivity");

        // Otro relé
        publishBinarySensor("other_relay", "Otro Dispositivo", "connectivity");

        // Temperatura objetivo de combustión (control)
        publishNumberControl("target_burning_temp", "Temperatura Objetivo", "temperature", "°C", 60, 100, 1);

        // Posición de entrada de aire
        publishSensor("air_intake", "Entrada de Aire", "power_factor", "%");
    }

    void publishSensor(const String& id, const String& name, const String& deviceClass, const String& unitOfMeasurement) {
        StaticJsonDocument<512> doc;

        doc["name"] = name;
        doc["state_topic"] = String(MQTT_BASE_TOPIC) + "/state";
        doc["value_template"] = "{{ value_json." + id + " }}";
        doc["unique_id"] = String("lumber_boiler_") + id;
        doc["device_class"] = deviceClass;
        doc["unit_of_measurement"] = unitOfMeasurement;

        JsonObject device = doc.createNestedObject("device");
        device["identifiers"] = MQTT_CLIENT_ID;
        device["name"] = "Caldera de Biomasa";
        device["model"] = "Lumber Boiler Manager";
        device["manufacturer"] = "ESP32";

        char buffer[512];
        serializeJson(doc, buffer);

        String topic = String("homeassistant/sensor/lumber_boiler/") + id + "/config";
        mqttClient.publish(topic.c_str(), buffer, true);
    }

    void publishBinarySensor(const String& id, const String& name, const String& deviceClass) {
        StaticJsonDocument<512> doc;

        doc["name"] = name;
        doc["state_topic"] = String(MQTT_BASE_TOPIC) + "/state";
        doc["value_template"] = "{{ value_json." + id + " }}";
        doc["unique_id"] = String("lumber_boiler_") + id;
        doc["device_class"] = deviceClass;

        JsonObject device = doc.createNestedObject("device");
        device["identifiers"] = MQTT_CLIENT_ID;
        device["name"] = "Caldera de Biomasa";
        device["model"] = "Lumber Boiler Manager";
        device["manufacturer"] = "ESP32";

        char buffer[512];
        serializeJson(doc, buffer);

        String topic = String("homeassistant/binary_sensor/lumber_boiler/") + id + "/config";
        mqttClient.publish(topic.c_str(), buffer, true);
    }

    void publishNumberControl(const String& id, const String& name, const String& deviceClass,
                             const String& unitOfMeasurement, float min, float max, float step) {
        StaticJsonDocument<512> doc;

        doc["name"] = name;
        doc["command_topic"] = String(MQTT_BASE_TOPIC) + "/set/" + id;
        doc["state_topic"] = String(MQTT_BASE_TOPIC) + "/state";
        doc["value_template"] = "{{ value_json." + id + " }}";
        doc["unique_id"] = String("lumber_boiler_") + id;
        doc["device_class"] = deviceClass;
        doc["unit_of_measurement"] = unitOfMeasurement;
        doc["min"] = min;
        doc["max"] = max;
        doc["step"] = step;

        JsonObject device = doc.createNestedObject("device");
        device["identifiers"] = MQTT_CLIENT_ID;
        device["name"] = "Caldera de Biomasa";
        device["model"] = "Lumber Boiler Manager";
        device["manufacturer"] = "ESP32";

        char buffer[512];
        serializeJson(doc, buffer);

        String topic = String("homeassistant/number/lumber_boiler/") + id + "/config";
        mqttClient.publish(topic.c_str(), buffer, true);
    }

    PubSubClient mqttClient;
    bool mqttConnected;
};

#endif // HOME_ASSISTANT_H
