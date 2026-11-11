#ifndef RULES_IO_H
#define RULES_IO_H

// JSON <-> RuleSet and NVS persistence for the relay rules.
// JSON shape (also used by GET/POST /api/rules):
//   {"boiler_pump":[[{"a":10,"op":">","v":0.5,"h":0}], ...], "heating_pump":[...], "fans":[...]}
//   a = variable, b = optional second temperature (a - b), op = ">", "<" or "missing" (no reading),
//   v = value, h = hysteresis (both absent for "missing")
// Each relay key holds a list of groups (OR); each group a list of conditions (AND).

#include <ArduinoJson.h>
#include "rules.h"

#define RULES_PREFS_NS  "rules"
#define RULES_PREFS_KEY "json"

// Parses the relays present in `obj` into `out` (missing relays are left untouched).
// Returns nullptr on success, otherwise a message. Validates the result.
inline const char* rulesFromJson(JsonObjectConst obj, RuleSet& out) {
    RuleSet next = out;
    for (int r = 0; r < RULE_RELAYS; r++) {
        if (!obj.containsKey(ruleRelayKey(r))) continue;
        JsonArrayConst groups = obj[ruleRelayKey(r)];
        if (groups.isNull()) return "Formato de reglas no válido";
        RelayRule rr;
        for (JsonVariantConst gv : groups) {
            JsonArrayConst conds = gv.as<JsonArrayConst>();
            if (conds.isNull()) return "Formato de reglas no válido";
            if (rr.n >= RULE_MAX_GROUPS) return "Demasiados grupos de reglas";
            RuleGroup& g = rr.g[rr.n++];
            for (JsonVariantConst cv : conds) {
                if (g.n >= RULE_MAX_CONDS) return "Demasiadas condiciones en un grupo";
                if (!cv.is<JsonObjectConst>()) return "Formato de reglas no válido";
                RuleCond& c = g.c[g.n++];
                c.a = cv["a"] | -1;
                c.b = cv.containsKey("b") ? (int)(cv["b"] | -2) : -1;
                const char* op = cv["op"] | "";
                if (strcmp(op, ">") == 0) c.op = RULE_OP_GT;
                else if (strcmp(op, "<") == 0) c.op = RULE_OP_LT;
                else if (strcmp(op, "missing") == 0) c.op = RULE_OP_MISSING;
                else return "Operador de regla no válido (usa >, < o missing)";
                if (c.op != RULE_OP_MISSING && !cv["v"].is<float>() && !cv["v"].is<int>()) return "Falta el valor de una regla";
                c.value = c.op == RULE_OP_MISSING ? 0.0f : cv["v"].as<float>();
                c.hyst  = cv["h"] | 0.0f;
            }
        }
        next.relay[r] = rr;
    }
    if (const char* err = ruleValidate(next)) return err;
    out = next;
    return nullptr;
}

inline void rulesToJson(const RuleSet& rs, JsonObject obj) {
    for (int r = 0; r < RULE_RELAYS; r++) {
        JsonArray groups = obj.createNestedArray(ruleRelayKey(r));
        for (uint8_t gi = 0; gi < rs.relay[r].n; gi++) {
            JsonArray conds = groups.createNestedArray();
            const RuleGroup& g = rs.relay[r].g[gi];
            for (uint8_t ci = 0; ci < g.n; ci++) {
                const RuleCond& c = g.c[ci];
                JsonObject o = conds.createNestedObject();
                o["a"] = c.a;
                if (c.b >= 0) o["b"] = c.b;
                o["op"] = c.op == RULE_OP_MISSING ? "missing" : (c.op == RULE_OP_GT ? ">" : "<");
                if (c.op != RULE_OP_MISSING) { o["v"] = c.value; o["h"] = c.hyst; }
            }
        }
    }
}

#ifndef RULES_IO_NO_NVS
#include <Preferences.h>
// Loads the saved rules; false when none are stored (or they are corrupt)
inline bool rulesLoad(RuleSet& out) {
    Preferences prefs;
    if (!prefs.begin(RULES_PREFS_NS, true)) return false;
    bool ok = false;
    if (prefs.isKey(RULES_PREFS_KEY)) {
        String s = prefs.getString(RULES_PREFS_KEY, "");
        DynamicJsonDocument doc(4096);
        RuleSet tmp;
        if (!deserializeJson(doc, s) && !rulesFromJson(doc.as<JsonObjectConst>(), tmp)) { out = tmp; ok = true; }
    }
    prefs.end();
    return ok;
}
inline bool rulesSave(const RuleSet& rs) {
    DynamicJsonDocument doc(4096);
    rulesToJson(rs, doc.to<JsonObject>());
    String s; serializeJson(doc, s);
    Preferences prefs;
    if (!prefs.begin(RULES_PREFS_NS, false)) return false;
    bool ok = prefs.putString(RULES_PREFS_KEY, s) > 0;
    prefs.end();
    return ok;
}
inline void rulesClearSaved() {
    Preferences prefs;
    if (prefs.begin(RULES_PREFS_NS, false)) { prefs.clear(); prefs.end(); }
}
#endif

#endif // RULES_IO_H
