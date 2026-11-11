#ifndef RELAY_CONFIG_H
#define RELAY_CONFIG_H

#include <Arduino.h>
#include <Preferences.h>
#include "config.h"

// Physical relay outputs ("channels"), indexed 0..3.
// Which function each channel drives is configurable at runtime (stored in NVS),
// so a swapped wiring can be fixed from the UI without reflashing.
#define RELAY_CHANNEL_COUNT  4
static const uint8_t RELAY_CHANNEL_PINS[RELAY_CHANNEL_COUNT] = {
    RELAY_OTHER, RELAY_BOILER_PUMP, RELAY_HEATING_PUMP, RELAY_FANS
};

// Functions (roles). Order matters: it is the index used everywhere.
enum RelayRole { ROLE_IDX_BOILER_PUMP = 0, ROLE_IDX_HEATING_PUMP, ROLE_IDX_FANS, ROLE_IDX_OTHER, RELAY_ROLE_COUNT };

#define RELAY_PREFS_NS "relay-cfg"

class RelayConfig {
public:
    static const char* roleId(int r) {
        static const char* const ids[RELAY_ROLE_COUNT] = { "boiler_pump", "heating_pump", "fans", "other" };
        return (r >= 0 && r < RELAY_ROLE_COUNT) ? ids[r] : nullptr;
    }
    static const char* roleLabel(int r) {
        static const char* const lb[RELAY_ROLE_COUNT] = { "Bomba caldera", "Bomba calefacción", "Ventiladores", "Relé auxiliar" };
        return (r >= 0 && r < RELAY_ROLE_COUNT) ? lb[r] : "";
    }
    static int roleFromId(const char* id) {
        for (int r = 0; r < RELAY_ROLE_COUNT; r++) if (strcmp(id, roleId(r)) == 0) return r;
        return -1;
    }

    RelayConfig() { setDefaults(); }

    void begin() {
        Preferences prefs;
        if (!prefs.begin(RELAY_PREFS_NS, true)) return;
        if (prefs.isKey("v")) {
            int8_t tmp[RELAY_ROLE_COUNT];
            for (int r = 0; r < RELAY_ROLE_COUNT; r++) tmp[r] = prefs.getChar(roleId(r), -1);
            if (isPermutation(tmp)) memcpy(channel, tmp, sizeof(channel));   // ignore corrupt data
        }
        prefs.end();
    }

    // Channel (0..3) currently driving `role`
    int channelFor(int role) const { return channel[role]; }

    // Role currently on `ch`, or -1
    int roleOn(int ch) const {
        for (int r = 0; r < RELAY_ROLE_COUNT; r++) if (channel[r] == ch) return r;
        return -1;
    }

    // Give `role` the channel `ch`. If another role was using it, the two swap channels,
    // so the mapping always stays a valid permutation. Returns false on bad arguments.
    bool assign(int role, int ch) {
        if (role < 0 || role >= RELAY_ROLE_COUNT || ch < 0 || ch >= RELAY_CHANNEL_COUNT) return false;
        int other = roleOn(ch);
        if (other >= 0 && other != role) channel[other] = channel[role];
        channel[role] = ch;
        return save();
    }

    bool save() const {
        Preferences prefs;
        if (!prefs.begin(RELAY_PREFS_NS, false)) return false;
        prefs.putUChar("v", 1);
        for (int r = 0; r < RELAY_ROLE_COUNT; r++) prefs.putChar(roleId(r), channel[r]);
        prefs.end();
        return true;
    }

    void resetToDefaults() {
        setDefaults();
        Preferences prefs;
        if (prefs.begin(RELAY_PREFS_NS, false)) { prefs.clear(); prefs.end(); }
    }

private:
    // Defaults reproduce the original fixed wiring (see config.h)
    void setDefaults() {
        channel[ROLE_IDX_BOILER_PUMP]  = 1;
        channel[ROLE_IDX_HEATING_PUMP] = 2;
        channel[ROLE_IDX_FANS]         = 3;
        channel[ROLE_IDX_OTHER]        = 0;
    }
    static bool isPermutation(const int8_t* m) {
        bool used[RELAY_CHANNEL_COUNT] = {};
        for (int r = 0; r < RELAY_ROLE_COUNT; r++) {
            if (m[r] < 0 || m[r] >= RELAY_CHANNEL_COUNT || used[m[r]]) return false;
            used[m[r]] = true;
        }
        return true;
    }

    int8_t channel[RELAY_ROLE_COUNT];
};

#endif // RELAY_CONFIG_H
