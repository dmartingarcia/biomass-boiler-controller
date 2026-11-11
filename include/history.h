#ifndef HISTORY_H
#define HISTORY_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <math.h>
#include "config.h"

// Multi-tier temperature history kept in RAM (lost on reboot). Each tier is its own
// fixed-size ring buffer; nothing is "moved" between them — every tier receives the mean
// of the previous (finer) one as samples arrive, and old samples are simply overwritten:
//
//   ultra  — one sample every HISTORY_SAMPLE_INTERVAL_MS (1 s)
//   fine   — mean of HISTORY_EVERY_0 ultra samples
//   mid    — mean of HISTORY_EVERY_1 fine samples
//   coarse — mean of HISTORY_EVERY_2 mid samples
//
// (sizes and factors in config.h)
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
    TemperatureHistory() {
        static const int caps[HISTORY_TIERS]  = { HISTORY_COUNT_0, HISTORY_COUNT_1, HISTORY_COUNT_2, HISTORY_COUNT_3 };
        HistorySample* bufs[HISTORY_TIERS]    = { buf0, buf1, buf2, buf3 };
        unsigned long step = HISTORY_SAMPLE_INTERVAL_MS / 1000UL;
        for (int i = 0; i < HISTORY_TIERS; i++) {
            tiers[i] = { bufs[i], caps[i], 0, 0, 0, step };
            if (i < HISTORY_TIERS - 1) step *= every(i);
        }
    }

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
        push(tiers[0], s);

        // cascade: every N samples of tier i, tier i+1 gets their mean
        for (int i = 0; i < HISTORY_TIERS - 1; i++) {
            if (++since[i] < every(i) || tiers[i].count < every(i)) break;
            since[i] = 0;
            push(tiers[i + 1], averageLast(tiers[i], every(i)));
        }
        xSemaphoreGive(lock);
    }

    // Render ONE tier ("ultra", "fine", "mid" or "coarse") as JSON (column-oriented, oldest → newest):
    //   {"channels":[...],"<tier>":{"step":s,"age":a,"n":n,"series":{"boiler_supply":[...],...}}}
    // Temperatures are tenths of °C (null = invalid); "air" is %, "flags" a bitmask;
    // step = seconds between samples, age = seconds since the tier's newest sample.
    // One tier per request keeps each response small (the caller renders into a fixed buffer).
    bool writeTierJson(Print& out, const char* tier) {
        const Ring* r = nullptr;
        for (int i = 0; i < HISTORY_TIERS; i++) if (strcmp(tier, tierName(i)) == 0) r = &tiers[i];
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

    static const char* tierName(int i) {
        static const char* const n[HISTORY_TIERS] = { "ultra", "fine", "mid", "coarse" };
        return n[i];
    }
    // samples of tier i that make one sample of tier i+1
    static int every(int i) {
        static const int e[HISTORY_TIERS - 1] = { HISTORY_EVERY_0, HISTORY_EVERY_1, HISTORY_EVERY_2 };
        return e[i];
    }

    HistorySample buf0[HISTORY_COUNT_0];
    HistorySample buf1[HISTORY_COUNT_1];
    HistorySample buf2[HISTORY_COUNT_2];
    HistorySample buf3[HISTORY_COUNT_3];
    Ring tiers[HISTORY_TIERS];
    int since[HISTORY_TIERS - 1] = { 0, 0, 0 };
    SemaphoreHandle_t lock = nullptr;
};

#endif // HISTORY_H
