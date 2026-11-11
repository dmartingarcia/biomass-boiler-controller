#ifndef SENSOR_HOLD_H
#define SENSOR_HOLD_H

// Keeps the last valid reading of a sensor for a limited time after it starts failing,
// so a short glitch does not make the relay rules flip on and off. After the hold time
// the reading is reported missing (NAN) until a valid one comes back.

#include <math.h>

#define SENSOR_HOLD_MS 60000UL

struct SensorHold {
    float         lastGood = NAN;
    unsigned long lastGoodMs = 0;
    bool          have = false;

    // raw: current reading, valid: whether it is a usable value. Returns the value to use.
    float update(float raw, bool valid, unsigned long nowMs, unsigned long holdMs = SENSOR_HOLD_MS) {
        if (valid && !isnan(raw)) { lastGood = raw; lastGoodMs = nowMs; have = true; return raw; }
        if (have && (unsigned long)(nowMs - lastGoodMs) < holdMs) return lastGood;
        have = false;
        return NAN;
    }
};

#endif // SENSOR_HOLD_H
