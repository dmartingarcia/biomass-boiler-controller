#ifndef SENSOR_CONFIG_H
#define SENSOR_CONFIG_H

#include <Arduino.h>
#include <Preferences.h>
#include <LittleFS.h>
#include <ArduinoJson.h>

#define SENSOR_CONFIG_FILE "/sensor_config.json"  // legacy — kept for one-time migration only
#define SENSOR_PREFS_NS    "sensor-cfg"

// 8 Dallas DS18B20 roles
#define ROLE_AMBIENT_LOW    "ambient_low"    // ambiente abajo
#define ROLE_BOILER_IRON    "boiler_iron"    // hierro caldera
#define ROLE_MIX_SUPPLY     "mix_supply"     // ida mezcla
#define ROLE_BOILER_SUPPLY  "boiler_supply"  // ida caldera
#define ROLE_FLOOR_SUPPLY   "floor_supply"   // ida suelo
#define ROLE_AMBIENT_HIGH   "ambient_high"   // ambiente arriba
#define ROLE_FLOOR_RETURN   "floor_return"   // retorno suelo
#define ROLE_BOILER_RETURN  "boiler_return"  // retorno caldera

// Config is stored in NVS (survives "Upload Filesystem Image").
// On first boot after migration, any existing LittleFS JSON is imported automatically.
class SensorConfig {
public:
    char ambientLowAddr[17]   = {0};
    char boilerIronAddr[17]   = {0};
    char mixSupplyAddr[17]    = {0};
    char boilerSupplyAddr[17] = {0};
    char floorSupplyAddr[17]  = {0};
    char ambientHighAddr[17]  = {0};
    char floorReturnAddr[17]  = {0};
    char boilerReturnAddr[17] = {0};

    void begin() { load(); }

    bool load() {
        Preferences prefs;
        if (prefs.begin(SENSOR_PREFS_NS, true)) {
            if (prefs.isKey("v")) {
                prefs.getString(ROLE_AMBIENT_LOW,   ambientLowAddr,   sizeof(ambientLowAddr));
                prefs.getString(ROLE_BOILER_IRON,   boilerIronAddr,   sizeof(boilerIronAddr));
                prefs.getString(ROLE_MIX_SUPPLY,    mixSupplyAddr,    sizeof(mixSupplyAddr));
                prefs.getString(ROLE_BOILER_SUPPLY, boilerSupplyAddr, sizeof(boilerSupplyAddr));
                prefs.getString(ROLE_FLOOR_SUPPLY,  floorSupplyAddr,  sizeof(floorSupplyAddr));
                prefs.getString(ROLE_AMBIENT_HIGH,  ambientHighAddr,  sizeof(ambientHighAddr));
                prefs.getString(ROLE_FLOOR_RETURN,  floorReturnAddr,  sizeof(floorReturnAddr));
                prefs.getString(ROLE_BOILER_RETURN, boilerReturnAddr, sizeof(boilerReturnAddr));
                prefs.end();
                return true;
            }
            prefs.end();
        }

        // One-time migration from legacy LittleFS file
        if (LittleFS.exists(SENSOR_CONFIG_FILE)) {
            if (_loadLegacy()) {
                save();
                LittleFS.remove(SENSOR_CONFIG_FILE);
                return true;
            }
        }
        return false;
    }

    bool save() {
        Preferences prefs;
        if (!prefs.begin(SENSOR_PREFS_NS, false)) return false;
        prefs.putUChar("v", 1);
        prefs.putString(ROLE_AMBIENT_LOW,   ambientLowAddr);
        prefs.putString(ROLE_BOILER_IRON,   boilerIronAddr);
        prefs.putString(ROLE_MIX_SUPPLY,    mixSupplyAddr);
        prefs.putString(ROLE_BOILER_SUPPLY, boilerSupplyAddr);
        prefs.putString(ROLE_FLOOR_SUPPLY,  floorSupplyAddr);
        prefs.putString(ROLE_AMBIENT_HIGH,  ambientHighAddr);
        prefs.putString(ROLE_FLOOR_RETURN,  floorReturnAddr);
        prefs.putString(ROLE_BOILER_RETURN, boilerReturnAddr);
        prefs.end();
        return true;
    }

    bool setRole(const char* role, const char* addr) {
        char* target = addrField(role);
        if (!target) return false;
        strlcpy(target, addr, 17);
        return save();
    }

    bool clearRole(const char* role) { return setRole(role, ""); }

    const char* getAddrForRole(const char* role) const {
        if (strcmp(role, ROLE_AMBIENT_LOW)   == 0) return ambientLowAddr;
        if (strcmp(role, ROLE_BOILER_IRON)   == 0) return boilerIronAddr;
        if (strcmp(role, ROLE_MIX_SUPPLY)    == 0) return mixSupplyAddr;
        if (strcmp(role, ROLE_BOILER_SUPPLY) == 0) return boilerSupplyAddr;
        if (strcmp(role, ROLE_FLOOR_SUPPLY)  == 0) return floorSupplyAddr;
        if (strcmp(role, ROLE_AMBIENT_HIGH)  == 0) return ambientHighAddr;
        if (strcmp(role, ROLE_FLOOR_RETURN)  == 0) return floorReturnAddr;
        if (strcmp(role, ROLE_BOILER_RETURN) == 0) return boilerReturnAddr;
        return nullptr;
    }

    const char* getRoleForAddr(const char* addr) const {
        if (strlen(addr) == 0) return "";
        if (strcmp(addr, ambientLowAddr)   == 0) return ROLE_AMBIENT_LOW;
        if (strcmp(addr, boilerIronAddr)   == 0) return ROLE_BOILER_IRON;
        if (strcmp(addr, mixSupplyAddr)    == 0) return ROLE_MIX_SUPPLY;
        if (strcmp(addr, boilerSupplyAddr) == 0) return ROLE_BOILER_SUPPLY;
        if (strcmp(addr, floorSupplyAddr)  == 0) return ROLE_FLOOR_SUPPLY;
        if (strcmp(addr, ambientHighAddr)  == 0) return ROLE_AMBIENT_HIGH;
        if (strcmp(addr, floorReturnAddr)  == 0) return ROLE_FLOOR_RETURN;
        if (strcmp(addr, boilerReturnAddr) == 0) return ROLE_BOILER_RETURN;
        return "";
    }

private:
    bool _loadLegacy() {
        File f = LittleFS.open(SENSOR_CONFIG_FILE, "r");
        if (!f) return false;
        StaticJsonDocument<512> doc;
        DeserializationError err = deserializeJson(doc, f);
        f.close();
        if (err) return false;
        strlcpy(ambientLowAddr,   doc[ROLE_AMBIENT_LOW]   | "", sizeof(ambientLowAddr));
        strlcpy(boilerIronAddr,   doc[ROLE_BOILER_IRON]   | "", sizeof(boilerIronAddr));
        strlcpy(mixSupplyAddr,    doc[ROLE_MIX_SUPPLY]    | "", sizeof(mixSupplyAddr));
        strlcpy(boilerSupplyAddr, doc[ROLE_BOILER_SUPPLY] | "", sizeof(boilerSupplyAddr));
        strlcpy(floorSupplyAddr,  doc[ROLE_FLOOR_SUPPLY]  | "", sizeof(floorSupplyAddr));
        strlcpy(ambientHighAddr,  doc[ROLE_AMBIENT_HIGH]  | "", sizeof(ambientHighAddr));
        strlcpy(floorReturnAddr,  doc[ROLE_FLOOR_RETURN]  | "", sizeof(floorReturnAddr));
        strlcpy(boilerReturnAddr, doc[ROLE_BOILER_RETURN] | "", sizeof(boilerReturnAddr));
        return true;
    }

    char* addrField(const char* role) {
        if (strcmp(role, ROLE_AMBIENT_LOW)   == 0) return ambientLowAddr;
        if (strcmp(role, ROLE_BOILER_IRON)   == 0) return boilerIronAddr;
        if (strcmp(role, ROLE_MIX_SUPPLY)    == 0) return mixSupplyAddr;
        if (strcmp(role, ROLE_BOILER_SUPPLY) == 0) return boilerSupplyAddr;
        if (strcmp(role, ROLE_FLOOR_SUPPLY)  == 0) return floorSupplyAddr;
        if (strcmp(role, ROLE_AMBIENT_HIGH)  == 0) return ambientHighAddr;
        if (strcmp(role, ROLE_FLOOR_RETURN)  == 0) return floorReturnAddr;
        if (strcmp(role, ROLE_BOILER_RETURN) == 0) return boilerReturnAddr;
        return nullptr;
    }
};

#endif // SENSOR_CONFIG_H
