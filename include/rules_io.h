#ifndef RULES_IO_H
#define RULES_IO_H

// JSON <-> RuleSet and NVS persistence for the relay rules.
// JSON shape (also used by GET/POST /api/rules):
//   {"boiler_pump":[[{"a":10,"op":">","v":0.5,"h":0}], ...], "heating_pump":[...], "fans":[...]}
//   a = variable, b = optional second temperature (a - b), op = ">", "<", "missing" (no reading),
//   "true" / "false" (boolean variables 9 and 10); v = value, h = hysteresis (absent for the last three)
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
                else if (strcmp(op, "true") == 0)    c.op = RULE_OP_TRUE;
                else if (strcmp(op, "false") == 0)   c.op = RULE_OP_FALSE;
                else return "Operador de regla no válido (usa >, <, missing, true o false)";
                bool noValue = c.op >= RULE_OP_MISSING;      // missing / true / false take no number
                if (!noValue && !cv["v"].is<float>() && !cv["v"].is<int>()) return "Falta el valor de una regla";
                c.value = noValue ? 0.0f : cv["v"].as<float>();
                c.hyst  = noValue ? 0.0f : (cv["h"] | 0.0f);
                // Rules saved by an earlier version compared the 1/0 variables with "> 0.5" / "< 0.5"
                if (c.a >= RULE_TEMP_VARS && (c.op == RULE_OP_GT || c.op == RULE_OP_LT)) {
                    c.op = c.op == RULE_OP_GT ? RULE_OP_TRUE : RULE_OP_FALSE; c.value = 0; c.hyst = 0;
                }
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
                static const char* const opName[] = { ">", "<", "missing", "true", "false" };
                o["op"] = opName[c.op <= RULE_OP_FALSE ? c.op : 0];
                if (c.op < RULE_OP_MISSING) { o["v"] = c.value; o["h"] = c.hyst; }
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
