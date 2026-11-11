#ifndef RELAY_H
#define RELAY_H

#include <Arduino.h>

class Relay {
public:
    Relay(uint8_t pin, const char* name)
        : pin(pin), name(name), state(false), desiredState(false),
          lastChangeTime(0), debounceMs(0), initialized(false) {}

    void begin() {
        pinMode(pin, OUTPUT);
        applyState(false);
        initialized = false;  // first setState() takes effect immediately
    }

    // Express desired state — applied after debounce period expires
    void setState(bool newState) {
        desiredState = newState;
    }

    // Apply immediately, bypassing debounce (emergency / manual override)
    void forceState(bool newState) {
        desiredState = newState;
        applyState(newState);
        initialized = true;
    }

    // Forget the last change time so the next setState() applies immediately
    // (used when leaving manual mode)
    void resetDebounce() { initialized = false; }

    // Call every loop iteration. Returns true if the physical state changed.
    bool tick() {
        if (desiredState == state) return false;
        if (!initialized || (millis() - lastChangeTime) >= debounceMs) {
            applyState(desiredState);
            initialized = true;
            return true;
        }
        return false;
    }

    // Re-assign the output pin. setPin() before begin(); rebind()+reapply() at runtime
    // (caller must drive every old pin LOW first so two relays never share a pin).
    void setPin(uint8_t p) { pin = p; }
    void rebind(uint8_t p) { pin = p; pinMode(pin, OUTPUT); }
    void reapply()         { digitalWrite(pin, state ? HIGH : LOW); }
    uint8_t getPin() const { return pin; }

    bool getState()        const { return state; }
    bool getDesiredState() const { return desiredState; }
    const char* getName()  const { return name; }

    void setDebounce(unsigned long ms) { debounceMs = ms; }
    unsigned long getDebounceMs() const { return debounceMs; }

private:
    void applyState(bool s) {
        digitalWrite(pin, s ? HIGH : LOW);
        state = s;
        lastChangeTime = millis();
    }

    uint8_t       pin;
    const char*   name;
    bool          state;
    bool          desiredState;
    unsigned long lastChangeTime;
    unsigned long debounceMs;
    bool          initialized;
};

#endif // RELAY_H
