#ifndef RELAY_H
#define RELAY_H

#include <Arduino.h>
#include "config.h"

// Clase para un relay individual
class Relay {
public:
    Relay(uint8_t pin, const String& name) :
        pin(pin), name(name), state(false) {}

    void begin() {
        pinMode(pin, OUTPUT);
        setState(false); // Iniciar apagado
    }

    void setState(bool newState) {
        digitalWrite(pin, newState ? HIGH : LOW);
        state = newState;
    }

    bool getState() const {
        return state;
    }

    String getName() const {
        return name;
    }

private:
    uint8_t pin;
    String name;
    bool state;
};

#endif // RELAY_H