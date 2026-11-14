#ifndef RUNTIME_CONFIG_H
#define RUNTIME_CONFIG_H

#include <Arduino.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "config.h"

#define SETTINGS_FILE   "/settings.json"  // legacy — kept for one-time migration only
#define RUNTIME_PREFS_NS "sys-cfg"

// Config is stored in NVS (survives "Upload Filesystem Image").
// On first boot after migration, any existing LittleFS JSON is imported automatically.
struct RuntimeSettings {
    float    burningTempThreshold;
    float    burningHysteresis;
    float    boilerWaterTempThreshold;
    float    boilerWaterCriticalTemp;
    float    targetBurningTemp;
    int      pidSource;
    float    pidKp;
    float    pidKi;
    float    pidKd;
    int      servoMin;
    int      servoMax;
    int      relayDebounceMin;
    float    boilerIronThreshold;
    float    floorDeltaThreshold;
    float    fanOnTemp;
    int      burnSource, fanSource, ironSource, waterSource;
    char     mqttServer[64];
    uint16_t mqttPort;
    char     mqttUser[32];
    char     mqttPassword[64];
    char     mqttBaseTopic[32];
    uint16_t mqttIntervalSec;

    RuntimeSettings() {
        burningTempThreshold     = BURNING_TEMP_THRESHOLD;
        burningHysteresis        = BURNING_HYSTERESIS;
        boilerWaterTempThreshold = BOILER_WATER_TEMP_THRESHOLD;
        boilerWaterCriticalTemp  = BOILER_WATER_CRITICAL_TEMP;
        targetBurningTemp        = DEFAULT_TARGET_BURNING_TEMP;
        pidSource                = DEFAULT_PID_SOURCE;
        pidKp                    = PID_KP;
        pidKi                    = PID_KI;
        pidKd                    = PID_KD;
        servoMin                 = DEFAULT_SERVO_MIN;
        servoMax                 = DEFAULT_SERVO_MAX;
        relayDebounceMin         = RELAY_DEBOUNCE_MIN;
        boilerIronThreshold      = BOILER_IRON_PUMP_THRESHOLD;
        floorDeltaThreshold      = FLOOR_DELTA_THRESHOLD;
        fanOnTemp                = FAN_ON_TEMP;
        burnSource = DEFAULT_BURN_SOURCE; fanSource = DEFAULT_FAN_SOURCE;
        ironSource = DEFAULT_IRON_SOURCE; waterSource = DEFAULT_WATER_SOURCE;
        strlcpy(mqttServer,    MQTT_SERVER,     sizeof(mqttServer));
        mqttPort = MQTT_PORT;
        mqttIntervalSec = MQTT_PUBLISH_INTERVAL / 1000;
        strlcpy(mqttUser,      MQTT_USER,       sizeof(mqttUser));
        strlcpy(mqttPassword,  MQTT_PASSWORD,   sizeof(mqttPassword));
        strlcpy(mqttBaseTopic, MQTT_BASE_TOPIC, sizeof(mqttBaseTopic));
    }

    bool load() {
        Preferences prefs;
        bool migrate = false;
        if (prefs.begin(RUNTIME_PREFS_NS, true)) {
            if (prefs.isKey("v")) {
                migrate = prefs.getUChar("v", 1) < 2;
                burningTempThreshold     = prefs.getFloat("burn_thresh",  burningTempThreshold);
                burningHysteresis        = prefs.getFloat("burn_hyst",    burningHysteresis);
                boilerWaterTempThreshold = prefs.getFloat("water_thresh", boilerWaterTempThreshold);
                boilerWaterCriticalTemp  = prefs.getFloat("water_crit",   boilerWaterCriticalTemp);
                targetBurningTemp        = prefs.getFloat("target_burn",  targetBurningTemp);
                pidSource                = prefs.getInt("pid_src",         pidSource);
                pidKp                    = prefs.getFloat("pid_kp",       pidKp);
                pidKi                    = prefs.getFloat("pid_ki",       pidKi);
                pidKd                    = prefs.getFloat("pid_kd",       pidKd);
                servoMin                 = prefs.getInt("servo_min",       servoMin);
                servoMax                 = prefs.getInt("servo_max",       servoMax);
                relayDebounceMin         = prefs.getInt("debounce_min",    relayDebounceMin);
                boilerIronThreshold      = prefs.getFloat("iron_thresh",  boilerIronThreshold);
                floorDeltaThreshold      = prefs.getFloat("floor_delta",  floorDeltaThreshold);
                // Settings saved before this existed: fans followed the combustion threshold
                fanOnTemp                = prefs.isKey("fan_on") ? prefs.getFloat("fan_on", fanOnTemp) : burningTempThreshold;
                burnSource               = prefs.getInt("src_burn",       burnSource);
                fanSource                = prefs.getInt("src_fan",        fanSource);
                ironSource               = prefs.getInt("src_iron",       ironSource);
                waterSource              = prefs.getInt("src_water",      waterSource);
                mqttPort                 = prefs.getUShort("mqtt_port",   mqttPort);
                mqttIntervalSec          = prefs.getUShort("mqtt_int",    mqttIntervalSec);
                if (prefs.isKey("mqtt_server")) prefs.getString("mqtt_server", mqttServer,    sizeof(mqttServer));
                if (prefs.isKey("mqtt_user"))   prefs.getString("mqtt_user",   mqttUser,      sizeof(mqttUser));
                if (prefs.isKey("mqtt_pass"))   prefs.getString("mqtt_pass",   mqttPassword,  sizeof(mqttPassword));
                if (prefs.isKey("mqtt_topic"))  prefs.getString("mqtt_topic",  mqttBaseTopic, sizeof(mqttBaseTopic));
                prefs.end();
                // v2: the PID regulates the boiler water (index 0); gases only detect combustion.
                // Applied once to settings saved by version 1 (the stored target is kept).
                if (migrate) { pidSource = DEFAULT_PID_SOURCE; save(); }
                return true;
            }
            prefs.end();
        }

        // One-time migration from legacy LittleFS file
        if (LittleFS.exists(SETTINGS_FILE)) {
            if (_loadLegacy()) {
                save();
                LittleFS.remove(SETTINGS_FILE);
                return true;
            }
        }
        return false;
    }

    bool save() const {
        Preferences prefs;
        if (!prefs.begin(RUNTIME_PREFS_NS, false)) return false;
        prefs.putUChar("v", 2);
        prefs.putFloat("burn_thresh",  burningTempThreshold);
        prefs.putFloat("burn_hyst",    burningHysteresis);
        prefs.putFloat("water_thresh", boilerWaterTempThreshold);
        prefs.putFloat("water_crit",   boilerWaterCriticalTemp);
        prefs.putFloat("target_burn",  targetBurningTemp);
        prefs.putInt("pid_src",         pidSource);
        prefs.putFloat("pid_kp",       pidKp);
        prefs.putFloat("pid_ki",       pidKi);
        prefs.putFloat("pid_kd",       pidKd);
        prefs.putInt("servo_min",       servoMin);
        prefs.putInt("servo_max",       servoMax);
        prefs.putInt("debounce_min",    relayDebounceMin);
        prefs.putFloat("iron_thresh",  boilerIronThreshold);
        prefs.putFloat("floor_delta",  floorDeltaThreshold);
        prefs.putFloat("fan_on",       fanOnTemp);
        prefs.putInt("src_burn",       burnSource);
        prefs.putInt("src_fan",        fanSource);
        prefs.putInt("src_iron",       ironSource);
        prefs.putInt("src_water",      waterSource);
        prefs.putString("mqtt_server", mqttServer);
        prefs.putUShort("mqtt_port",   mqttPort);
        prefs.putUShort("mqtt_int",    mqttIntervalSec);
        prefs.putString("mqtt_user",   mqttUser);
        prefs.putString("mqtt_pass",   mqttPassword);
        prefs.putString("mqtt_topic",  mqttBaseTopic);
        prefs.end();
        return true;
    }

    // Returns nullptr when the values are coherent, otherwise a message for the user
    const char* validate() const {
        auto bad = [](float v) { return isnan(v) || isinf(v); };
        if (bad(burningTempThreshold) || bad(boilerWaterTempThreshold) || bad(boilerWaterCriticalTemp) ||
            bad(targetBurningTemp) || bad(pidKp) || bad(pidKi) || bad(pidKd) ||
            bad(boilerIronThreshold) || bad(floorDeltaThreshold) || bad(fanOnTemp) || bad(burningHysteresis))
            return "Algún valor no es un número";
        if (boilerWaterCriticalTemp <= boilerWaterTempThreshold)
            return "«Agua crítica» debe ser mayor que «Agua caliente»";
        if (boilerWaterCriticalTemp >= DALLAS_MAX_VALID_TEMP)
            return "«Agua crítica» debe ser menor que el máximo válido del sensor (120 °C)";
        if (burningTempThreshold < 0 || burningTempThreshold > 1000)
            return "Detección de combustión fuera de rango (0–1000 °C)";
        if (burningHysteresis < 0 || burningHysteresis > 100)
            return "Histéresis de combustión fuera de rango (0–100 °C)";
        if (targetBurningTemp < TARGET_TEMP_MIN || targetBurningTemp > TARGET_TEMP_MAX)
            return "Objetivo del PID fuera de rango";
        if (pidSource < 0 || pidSource > 8)
            return "Temperatura del PID no válida";
        if (pidKp < 0 || pidKi < 0 || pidKd < 0 || pidKp > 1000 || pidKi > 1000 || pidKd > 1000)
            return "Kp, Ki y Kd deben estar entre 0 y 1000";
        if (servoMin < 0 || servoMax > 180 || servoMin >= servoMax)
            return "Rango de servo no válido (0–180, mínimo < máximo)";
        if (relayDebounceMin < 0 || relayDebounceMin > 60)
            return "Debounce fuera de rango (0–60 min)";
        if (boilerIronThreshold < 0 || boilerIronThreshold > 120)
            return "Umbral de hierro fuera de rango (0–120 °C)";
        if (floorDeltaThreshold < 0 || floorDeltaThreshold > 50)
            return "Delta de suelo fuera de rango (0–50 °C)";
        if (burnSource < 0 || burnSource > 8 || fanSource < 0 || fanSource > 8 ||
            ironSource < 0 || ironSource > 8 || waterSource < 0 || waterSource > 8)
            return "Temperatura de decisión no válida";
        if (fanOnTemp < FAN_ON_TEMP_MIN || fanOnTemp > FAN_ON_TEMP_MAX)
            return "Temperatura de ventiladores fuera de rango (30–300 °C)";
        if (mqttPort == 0)
            return "Puerto MQTT no válido";
        if (mqttIntervalSec < 1 || mqttIntervalSec > 3600)
            return "Intervalo MQTT fuera de rango (1–3600 s)";
        return nullptr;
    }

    // Wipe the NVS namespace and restore compile-time defaults
    void resetToDefaults() {
        Preferences prefs;
        if (prefs.begin(RUNTIME_PREFS_NS, false)) {
            prefs.clear();
            prefs.end();
        }
        *this = RuntimeSettings();
    }

private:
    bool _loadLegacy() {
        File f = LittleFS.open(SETTINGS_FILE, "r");
        if (!f) return false;
        StaticJsonDocument<512> doc;
        if (deserializeJson(doc, f)) { f.close(); return false; }
        f.close();
        burningTempThreshold     = doc["burning_temp_threshold"] | burningTempThreshold;
        boilerWaterTempThreshold = doc["boiler_water_threshold"] | boilerWaterTempThreshold;
        boilerWaterCriticalTemp  = doc["boiler_water_critical"]  | boilerWaterCriticalTemp;
        targetBurningTemp        = doc["target_burning_temp"]    | targetBurningTemp;
        pidKp                    = doc["pid_kp"]                 | pidKp;
        pidKi                    = doc["pid_ki"]                 | pidKi;
        pidKd                    = doc["pid_kd"]                 | pidKd;
        servoMin                 = doc["servo_min"]              | servoMin;
        servoMax                 = doc["servo_max"]              | servoMax;
        relayDebounceMin         = doc["relay_debounce_min"]     | relayDebounceMin;
        boilerIronThreshold      = doc["boiler_iron_threshold"]  | boilerIronThreshold;
        floorDeltaThreshold      = doc["floor_delta_threshold"]  | floorDeltaThreshold;
        if (doc.containsKey("mqtt_server"))
            strlcpy(mqttServer,    doc["mqtt_server"]    | mqttServer,    sizeof(mqttServer));
        mqttPort = doc["mqtt_port"] | mqttPort;
        if (doc.containsKey("mqtt_user"))
            strlcpy(mqttUser,      doc["mqtt_user"]      | mqttUser,      sizeof(mqttUser));
        if (doc.containsKey("mqtt_password"))
            strlcpy(mqttPassword,  doc["mqtt_password"]  | mqttPassword,  sizeof(mqttPassword));
        if (doc.containsKey("mqtt_base_topic"))
            strlcpy(mqttBaseTopic, doc["mqtt_base_topic"] | mqttBaseTopic, sizeof(mqttBaseTopic));
        return true;
    }
};

#endif // RUNTIME_CONFIG_H
