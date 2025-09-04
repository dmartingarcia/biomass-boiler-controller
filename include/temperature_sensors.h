#ifndef TEMPERATURE_SENSORS_H
#define TEMPERATURE_SENSORS_H

#include <Arduino.h>
#include "config.h"

class TemperatureSensors {
public:
    TemperatureSensors() {
    }

    void begin() {
        // Configure pins for NTC thermistors
        pinMode(NTC_BOILER_WATER_PIN, INPUT);
        pinMode(NTC_HEATING_PIN, INPUT);
        pinMode(NTC_BURNING_PIN, INPUT);
        pinMode(NTC_AMBIENT_PIN, INPUT);

        // Wait a moment for sensors to stabilize
        delay(500);
    }

    float getBoilerWaterTemperature() {
        return readNTC(NTC_BOILER_WATER_PIN);
    }

    float getHeatingTemperature() {
        return readNTC(NTC_HEATING_PIN);
    }

    float getBurningTemperature() {
        return readNTC(NTC_BURNING_PIN);
    }

    float getAmbientTemperature() {
        return readNTC(NTC_AMBIENT_PIN);
    }

    // Method to check if combustion is occurring
    bool isBurning() {
        return getBurningTemperature() > BURNING_TEMP_THRESHOLD;
    }

    // Method to check if boiler water is hot enough
    bool isBoilerWaterHot() {
        return getBoilerWaterTemperature() > BOILER_WATER_TEMP_THRESHOLD;
    }

    // Method to check if water temperature has reached a critical level (killswitch)
    bool isBoilerWaterCritical() {
        return getBoilerWaterTemperature() > BOILER_WATER_CRITICAL_TEMP;
    }

private:
    // Method to convert analog reading to temperature in Celsius degrees
    float readNTC(int pin) {
        int rawADC = analogRead(pin);

        // Constants for 10k NTC thermistor
        float c1 = 1.009249522e-03, c2 = 2.378405444e-04, c3 = 2.019202697e-07;

        // Thermistor resistance
        float resistor = NTC_SERIES_RESISTOR / ((4095.0 / rawADC) - 1.0);

        // Steinhart-Hart formula to convert resistance to temperature
        float logR = log(resistor);
        float temp = 1.0 / (c1 + c2 * logR + c3 * logR * logR * logR);

        // Convert from Kelvin to Celsius
        temp = temp - 273.15;

        return temp;
    }
};

#endif // TEMPERATURE_SENSORS_H
