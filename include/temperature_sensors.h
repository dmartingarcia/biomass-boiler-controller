#ifndef TEMPERATURE_SENSORS_H
#define TEMPERATURE_SENSORS_H

#include <Arduino.h>
#include <OneWire.h>
#include <DallasTemperature.h>
#include "config.h"
#include "sensor_config.h"

#define MAX_DALLAS_SENSORS 10

// Helper: 8-byte DeviceAddress → 16-char uppercase hex string
static void addrToStr(const DeviceAddress addr, char* out) {
    for (int i = 0; i < 8; i++) sprintf(out + i * 2, "%02X", addr[i]);
    out[16] = '\0';
}

// Helper: 16-char hex string → DeviceAddress, returns false on bad input
static bool strToAddr(const char* str, DeviceAddress out) {
    if (!str || strlen(str) != 16) return false;
    for (int i = 0; i < 8; i++) {
        char b[3] = {str[i * 2], str[i * 2 + 1], '\0'};
        out[i] = (uint8_t)strtol(b, nullptr, 16);
    }
    return true;
}

// Minimal bit-bang MAX6675 reader (no hardware SPI, no constructor side-effects)
class MAX6675Reader {
public:
    MAX6675Reader() = default;

    void begin() {
        pinMode(MAX6675_CS,  OUTPUT); digitalWrite(MAX6675_CS,  HIGH);
        pinMode(MAX6675_SCK, OUTPUT); digitalWrite(MAX6675_SCK, LOW);
        pinMode(MAX6675_SO,  INPUT);
    }

    float readCelsius() {
        uint16_t raw = spiread16();
        if (raw & 0x0004) return NAN;  // open-thermocouple fault bit
        raw >>= 3;
        return raw * 0.25f;
    }

private:
    uint16_t spiread16() {
        uint16_t d = 0;
        digitalWrite(MAX6675_CS, LOW);
        delayMicroseconds(10);
        for (int i = 15; i >= 0; i--) {
            digitalWrite(MAX6675_SCK, LOW);  delayMicroseconds(5);
            if (digitalRead(MAX6675_SO)) d |= (1 << i);
            digitalWrite(MAX6675_SCK, HIGH); delayMicroseconds(5);
        }
        digitalWrite(MAX6675_CS, HIGH);
        return d;
    }
};

class TemperatureSensors {
public:
    TemperatureSensors(SensorConfig* cfg)
        : oneWire(ONE_WIRE_PIN), dallas(&oneWire), config(cfg), sensorCount(0) {}

    void begin() {
        thermocouple.begin();
        dallas.begin();
        dallas.setWaitForConversion(false);  // async mode
        scanSensors();
    }

    // Re-scan the 1-Wire bus (call after begin() or on demand)
    int scanSensors() {
        dallas.begin();
        sensorCount = dallas.getDeviceCount();
        if (sensorCount > MAX_DALLAS_SENSORS) sensorCount = MAX_DALLAS_SENSORS;
        for (int i = 0; i < MAX_DALLAS_SENSORS; i++) cache[i] = DEVICE_DISCONNECTED_C;
        for (int i = 0; i < sensorCount; i++) {
            dallas.getAddress(addresses[i], i);
            dallas.setResolution(addresses[i], 12);
        }
        return sensorCount;
    }

    // Call this once per cycle; reads complete after ~750 ms (12-bit)
    void requestTemperatures() {
        dallas.requestTemperatures();
    }

    // Read every sensor's scratchpad ONCE and keep the values. Only the main loop may call
    // this (it is the only code that touches the 1-Wire bus); everything else — role getters,
    // the web handlers — reads the cache, so concurrent web requests can't corrupt a read.
    void refreshCache() {
        for (int i = 0; i < sensorCount; i++) cache[i] = dallas.getTempC(addresses[i]);
    }

    // Role-based accessors (return DEVICE_DISCONNECTED_C = -127.0 if unassigned)
    float getAmbientLowTemp()   { return getTempByRole(ROLE_AMBIENT_LOW);   }
    float getBoilerIronTemp()   { return getTempByRole(ROLE_BOILER_IRON);   }
    float getMixSupplyTemp()    { return getTempByRole(ROLE_MIX_SUPPLY);    }
    float getBoilerSupplyTemp() { return getTempByRole(ROLE_BOILER_SUPPLY); }
    float getFloorSupplyTemp()  { return getTempByRole(ROLE_FLOOR_SUPPLY);  }
    float getAmbientHighTemp()  { return getTempByRole(ROLE_AMBIENT_HIGH);  }
    float getFloorReturnTemp()  { return getTempByRole(ROLE_FLOOR_RETURN);  }
    float getBoilerReturnTemp() { return getTempByRole(ROLE_BOILER_RETURN); }

    // Burning temperature from MAX6675 thermocouple
    float getBurningTemperature() { return thermocouple.readCelsius(); }

    static bool isValidDallasTemp(float t) {
        return !isnan(t) && t > DALLAS_MIN_VALID_TEMP && t < DALLAS_MAX_VALID_TEMP;
    }

    void setBurningThreshold(float t)        { burningThreshold     = t; }
    void setBoilerWaterThreshold(float t)    { boilerWaterThreshold = t; }
    void setBoilerWaterCriticalTemp(float t) { boilerWaterCritical  = t; }

    bool isBurning() {
        float t = getBurningTemperature();
        return !isnan(t) && t > burningThreshold;
    }

    // Uses ida caldera (boiler supply) as the primary control temperature
    bool isBoilerWaterHot() {
        float t = getBoilerSupplyTemp();
        return isValidDallasTemp(t) && t > boilerWaterThreshold;
    }

    bool isBoilerWaterCritical() {
        float t = getBoilerSupplyTemp();
        return isValidDallasTemp(t) && t > boilerWaterCritical;
    }

    // Debug: number of Dallas sensors on the bus
    int getSensorCount() const { return sensorCount; }

    // Debug: temperature of sensor at bus index i
    float getTempByIndex(int i) {
        if (i < 0 || i >= sensorCount) return DEVICE_DISCONNECTED_C;
        return cache[i];
    }

    // Debug: hex address string of sensor at bus index i
    void getAddressStr(int i, char* out) const {
        if (i < 0 || i >= sensorCount) { out[0] = '\0'; return; }
        addrToStr(addresses[i], out);
    }

    // Debug: role currently assigned to sensor at bus index i
    const char* getRoleByIndex(int i) const {
        char addr[17];
        if (i < 0 || i >= sensorCount) return "";
        addrToStr(addresses[i], addr);
        return config->getRoleForAddr(addr);
    }

private:
    float getTempByRole(const char* role) {
        const char* addr = config->getAddrForRole(role);
        if (!addr || strlen(addr) == 0) return DEVICE_DISCONNECTED_C;
        DeviceAddress da;
        if (!strToAddr(addr, da)) return DEVICE_DISCONNECTED_C;
        for (int i = 0; i < sensorCount; i++)
            if (memcmp(addresses[i], da, 8) == 0) return cache[i];
        return DEVICE_DISCONNECTED_C;   // assigned sensor not on the bus
    }

    float burningThreshold     = BURNING_TEMP_THRESHOLD;
    float boilerWaterThreshold = BOILER_WATER_TEMP_THRESHOLD;
    float boilerWaterCritical  = BOILER_WATER_CRITICAL_TEMP;

    OneWire oneWire;
    DallasTemperature dallas;
    MAX6675Reader thermocouple;
    SensorConfig* config;

    int sensorCount;
    DeviceAddress addresses[MAX_DALLAS_SENSORS];
    float cache[MAX_DALLAS_SENSORS] = { DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C,
                                        DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C, DEVICE_DISCONNECTED_C };
};

#endif // TEMPERATURE_SENSORS_H
