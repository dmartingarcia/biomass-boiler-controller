#ifndef HISTORY_H
#define HISTORY_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <math.h>
#include "config.h"

// Three-tier temperature history kept in RAM (lost on reboot). Each tier is its own
// fixed-size ring buffer; nothing is "moved" between them — every tier receives an
// average of the finer one as samples arrive, and old samples are simply overwritten:
//
//   ultra  — one sample every HISTORY_SAMPLE_INTERVAL_MS (5 s), last HISTORY_ULTRA_COUNT
//   fine   — average of HISTORY_FINE_EVERY ultra samples (30 s), last HISTORY_FINE_COUNT
//   coarse — average of HISTORY_COARSE_EVERY fine samples (5 min), last HISTORY_COARSE_COUNT
//
// Temperatures are stored as tenths of a degree in int16_t; HISTORY_INVALID marks a
// missing/invalid reading.

#define HISTORY_CHANNELS   9
#define HISTORY_INVALID    INT16_MIN

// Bits of HistorySample::flags
#define HIST_FLAG_BOILER_PUMP   0x01
#define HIST_FLAG_HEATING_PUMP  0x02
#define HIST_FLAG_FANS          0x04
#define HIST_FLAG_OTHER         0x08
#define HIST_FLAG_MANUAL        0x10

struct HistorySample {
    int16_t t[HISTORY_CHANNELS];
    uint8_t air;     // servo position, %
    uint8_t flags;
};

class TemperatureHistory {
public:
    TemperatureHistory()
        : ultra{ ultraBuf,  HISTORY_ULTRA_COUNT,  0, 0, 0, HISTORY_SAMPLE_INTERVAL_MS / 1000UL },
          fine  { fineBuf,   HISTORY_FINE_COUNT,   0, 0, 0, (HISTORY_SAMPLE_INTERVAL_MS / 1000UL) * HISTORY_FINE_EVERY },
          coarse{ coarseBuf, HISTORY_COARSE_COUNT, 0, 0, 0, (HISTORY_SAMPLE_INTERVAL_MS / 1000UL) * HISTORY_FINE_EVERY * HISTORY_COARSE_EVERY } {}

    void begin() { lock = xSemaphoreCreateMutex(); }

    // Channel order (must match the order of temps[] passed to record())
    static const char* channelName(int i) {
        static const char* const names[HISTORY_CHANNELS] = {
            "boiler_supply", "boiler_return", "boiler_iron",
            "floor_supply",  "floor_return",  "mix_supply",
            "ambient_low",   "ambient_high",  "burning"
        };
        return names[i];
    }

    static int16_t encode(float v, bool valid) {
        if (!valid || isnan(v)) return HISTORY_INVALID;
        return (int16_t)lroundf(v * 10.0f);
    }

    // Call once every HISTORY_SAMPLE_INTERVAL_MS from loop()
    void record(const int16_t temps[HISTORY_CHANNELS], uint8_t air, uint8_t flags) {
        if (!lock || xSemaphoreTake(lock, pdMS_TO_TICKS(100)) != pdTRUE) return;

        HistorySample s;
        memcpy(s.t, temps, sizeof(s.t));
        s.air   = air;
        s.flags = flags;
        push(ultra, s);

        if (++sinceFine >= HISTORY_FINE_EVERY && ultra.count >= HISTORY_FINE_EVERY) {
            sinceFine = 0;
            push(fine, averageLast(ultra, HISTORY_FINE_EVERY));
            if (++sinceCoarse >= HISTORY_COARSE_EVERY && fine.count >= HISTORY_COARSE_EVERY) {
                sinceCoarse = 0;
                push(coarse, averageLast(fine, HISTORY_COARSE_EVERY));
            }
        }
        xSemaphoreGive(lock);
    }

    // Render ONE tier ("ultra", "fine" or "coarse") as JSON (column-oriented, oldest → newest):
    //   {"channels":[...],"<tier>":{"step":s,"age":a,"n":n,"series":{"boiler_supply":[...],...}}}
    // Temperatures are tenths of °C (null = invalid); "air" is %, "flags" a bitmask;
    // step = seconds between samples, age = seconds since the tier's newest sample.
    // One tier per request keeps each response small (the caller renders into a fixed buffer).
    bool writeTierJson(Print& out, const char* tier) {
        const Ring* r = strcmp(tier, "ultra") == 0 ? &ultra : strcmp(tier, "fine") == 0 ? &fine : strcmp(tier, "coarse") == 0 ? &coarse : nullptr;
        if (!r || !lock || xSemaphoreTake(lock, pdMS_TO_TICKS(500)) != pdTRUE) return false;
        unsigned long now = millis();
        out.print("{\"channels\":[");
        for (int c = 0; c < HISTORY_CHANNELS; c++) {
            if (c) out.print(',');
            out.print('"'); out.print(channelName(c)); out.print('"');
        }
        out.print("],");
        writeTier(out, tier, *r, now);
        out.print('}');
        xSemaphoreGive(lock);
        return true;
    }

private:
    struct Ring {
        HistorySample* buf;
        int            cap;
        int            head;       // where the next sample is written
        int            count;      // valid samples (≤ cap)
        unsigned long  lastMs;     // millis() of the newest sample
        unsigned long  stepSec;    // seconds between samples of this tier
    };

    static void push(Ring& r, const HistorySample& s) {
        r.buf[r.head] = s;
        r.head = (r.head + 1) % r.cap;
        if (r.count < r.cap) r.count++;
        r.lastMs = millis();
    }

    // Average of the newest n samples of a ring (invalid values ignored per channel;
    // relay flags OR-ed so a relay that was ON at any point in the window shows ON)
    static HistorySample averageLast(const Ring& r, int n) {
        HistorySample out;
        for (int c = 0; c < HISTORY_CHANNELS; c++) {
            long sum = 0; int valid = 0;
            for (int k = 0; k < n; k++) {
                const HistorySample& s = r.buf[(r.head - 1 - k + r.cap) % r.cap];
                if (s.t[c] != HISTORY_INVALID) { sum += s.t[c]; valid++; }
            }
            out.t[c] = valid ? (int16_t)(sum / valid) : HISTORY_INVALID;
        }
        long airSum = 0; uint8_t flags = 0;
        for (int k = 0; k < n; k++) {
            const HistorySample& s = r.buf[(r.head - 1 - k + r.cap) % r.cap];
            airSum += s.air;
            flags  |= s.flags;
        }
        out.air   = (uint8_t)(airSum / n);
        out.flags = flags;
        return out;
    }

    static void writeTier(Print& out, const char* name, const Ring& r, unsigned long now) {
        out.print('"'); out.print(name); out.print("\":{\"step\":");
        out.print(r.stepSec);
        out.print(",\"age\":");
        out.print(r.count ? (now - r.lastMs) / 1000UL : 0UL);
        out.print(",\"n\":");
        out.print(r.count);
        out.print(",\"series\":{");

        char tmp[12];
        for (int c = 0; c < HISTORY_CHANNELS + 2; c++) {
            if (c) out.print(',');
            out.print('"');
            out.print(c < HISTORY_CHANNELS ? channelName(c) : (c == HISTORY_CHANNELS ? "air" : "flags"));
            out.print("\":[");
            for (int i = 0; i < r.count; i++) {
                const HistorySample& s = r.buf[(r.head - r.count + i + r.cap) % r.cap];
                if (i) out.print(',');
                if (c < HISTORY_CHANNELS) {
                    if (s.t[c] == HISTORY_INVALID) out.print("null");
                    else { int n = snprintf(tmp, sizeof(tmp), "%d", s.t[c]); out.write((const uint8_t*)tmp, n); }
                } else {
                    int n = snprintf(tmp, sizeof(tmp), "%u", c == HISTORY_CHANNELS ? s.air : s.flags);
                    out.write((const uint8_t*)tmp, n);
                }
            }
            out.print(']');
        }
        out.print("}}");
    }

    HistorySample ultraBuf[HISTORY_ULTRA_COUNT];
    HistorySample fineBuf[HISTORY_FINE_COUNT];
    HistorySample coarseBuf[HISTORY_COARSE_COUNT];
    Ring ultra, fine, coarse;
    int sinceFine = 0, sinceCoarse = 0;
    SemaphoreHandle_t lock = nullptr;
};

#endif // HISTORY_H
