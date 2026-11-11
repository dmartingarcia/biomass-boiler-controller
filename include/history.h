#ifndef HISTORY_H
#define HISTORY_H

#include <Arduino.h>
#include <freertos/FreeRTOS.h>
#include <freertos/semphr.h>
#include <math.h>
#include "config.h"

// Two-tier temperature history kept in RAM (lost on reboot):
//   fine   — one sample per minute, last HISTORY_FINE_COUNT samples (1 h)
//   coarse — one sample per HISTORY_COARSE_EVERY minutes (average of the fine
//            samples in that window), last HISTORY_COARSE_COUNT samples (24 h)
//
// Temperatures are stored as tenths of a degree in int16_t; HISTORY_INVALID
// marks a missing/invalid reading.

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

    // Call once per minute from loop()
    void record(const int16_t temps[HISTORY_CHANNELS], uint8_t air, uint8_t flags) {
        if (!lock || xSemaphoreTake(lock, pdMS_TO_TICKS(100)) != pdTRUE) return;

        HistorySample s;
        memcpy(s.t, temps, sizeof(s.t));
        s.air   = air;
        s.flags = flags;
        push(fine, fineHead, fineCount, HISTORY_FINE_COUNT, s);
        lastFineMs = millis();

        if (++sinceCoarse >= HISTORY_COARSE_EVERY && fineCount >= HISTORY_COARSE_EVERY) {
            sinceCoarse = 0;
            HistorySample avg = averageLastFine(HISTORY_COARSE_EVERY);
            push(coarse, coarseHead, coarseCount, HISTORY_COARSE_COUNT, avg);
            lastCoarseMs = millis();
        }
        xSemaphoreGive(lock);
    }

    // Stream the whole history as JSON (column-oriented, oldest → newest).
    // Temperatures are tenths of °C (null = invalid); "air" is %, "flags" a bitmask.
    // fine_age / coarse_age = seconds elapsed since the newest sample of that tier.
    void writeJson(Print& out) {
        if (!lock || xSemaphoreTake(lock, pdMS_TO_TICKS(500)) != pdTRUE) {
            out.print("{\"error\":\"busy\"}");
            return;
        }
        unsigned long now = millis();
        out.print("{\"channels\":[");
        for (int c = 0; c < HISTORY_CHANNELS; c++) {
            if (c) out.print(',');
            out.print('"'); out.print(channelName(c)); out.print('"');
        }
        out.print("],");
        writeTier(out, "fine",   fine,   fineHead,   fineCount,   HISTORY_FINE_COUNT,
                  60UL, lastFineMs, now);
        out.print(',');
        writeTier(out, "coarse", coarse, coarseHead, coarseCount, HISTORY_COARSE_COUNT,
                  (unsigned long)HISTORY_COARSE_EVERY * 60UL, lastCoarseMs, now);
        out.print('}');
        xSemaphoreGive(lock);
    }

private:
    static void push(HistorySample* buf, int& head, int& count, int cap, const HistorySample& s) {
        buf[head] = s;
        head = (head + 1) % cap;
        if (count < cap) count++;
    }

    // Average of the last n fine samples (invalid values ignored per channel)
    HistorySample averageLastFine(int n) const {
        HistorySample out;
        for (int c = 0; c < HISTORY_CHANNELS; c++) {
            long sum = 0; int valid = 0;
            for (int k = 0; k < n; k++) {
                const HistorySample& s = fine[(fineHead - 1 - k + HISTORY_FINE_COUNT) % HISTORY_FINE_COUNT];
                if (s.t[c] != HISTORY_INVALID) { sum += s.t[c]; valid++; }
            }
            out.t[c] = valid ? (int16_t)(sum / valid) : HISTORY_INVALID;
        }
        long airSum = 0;
        uint8_t flags = 0;
        for (int k = 0; k < n; k++) {
            const HistorySample& s = fine[(fineHead - 1 - k + HISTORY_FINE_COUNT) % HISTORY_FINE_COUNT];
            airSum += s.air;
            flags  |= s.flags;
        }
        out.air   = (uint8_t)(airSum / n);
        out.flags = flags;
        return out;
    }

    void writeTier(Print& out, const char* name, const HistorySample* buf, int head, int count,
                   int cap, unsigned long stepSec, unsigned long lastMs, unsigned long now) {
        out.print('"'); out.print(name); out.print("\":{\"step\":");
        out.print(stepSec);
        out.print(",\"age\":");
        out.print(count ? (now - lastMs) / 1000UL : 0UL);
        out.print(",\"n\":");
        out.print(count);
        out.print(",\"series\":{");

        char tmp[12];
        for (int c = 0; c < HISTORY_CHANNELS + 2; c++) {
            if (c) out.print(',');
            out.print('"');
            out.print(c < HISTORY_CHANNELS ? channelName(c) : (c == HISTORY_CHANNELS ? "air" : "flags"));
            out.print("\":[");
            for (int i = 0; i < count; i++) {
                const HistorySample& s = buf[(head - count + i + cap) % cap];
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

    HistorySample fine[HISTORY_FINE_COUNT];
    HistorySample coarse[HISTORY_COARSE_COUNT];
    int fineHead = 0,   fineCount = 0;
    int coarseHead = 0, coarseCount = 0;
    int sinceCoarse = 0;
    unsigned long lastFineMs = 0, lastCoarseMs = 0;
    SemaphoreHandle_t lock = nullptr;
};

#endif // HISTORY_H
