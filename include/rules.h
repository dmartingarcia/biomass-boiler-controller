#ifndef RULES_H
#define RULES_H

// Hand-editable relay rules. Hardware-independent (unit-tested on the host).
//
// Each relay (boiler pump, heating pump, fans) has up to RULE_MAX_GROUPS groups.
// The relay wants to be ON when ANY group is satisfied (OR); a group is satisfied
// when ALL its conditions hold (AND). A relay with no groups stays OFF.
// A condition compares a variable (or the difference a - b of two temperatures)
// against a value, with a hysteresis so it does not chatter:
//   ">" turns true above value, and false again below value - hyst
//   "<" turns true below value, and false again above value + hyst
// Boolean variables (9 "boiler pump on", 10 "burning") use the operators "true" / "false"
// instead of a number. A missing/invalid reading makes ">" and "<" false; the "missing" operator is true exactly then
// (a - b is missing if either side is). Readings are held by sensor_hold.h before they get here.

#include <Arduino.h>
#include <math.h>
#include <stdint.h>
#include <initializer_list>
#include "config.h"

#define RULE_RELAYS        3     // boiler pump, heating pump, fans
#define RULE_MAX_GROUPS    4
#define RULE_MAX_CONDS     4
#define RULE_TEMP_VARS     9     // 0-8: same indices as the history channels
#define RULE_VAR_BOILER_PUMP 9   // 1 when the boiler pump rule is satisfied (evaluated first)
#define RULE_VAR_BURNING   10    // 1 while combustion is detected
#define RULE_VAR_COUNT     11

enum RuleOp : uint8_t { RULE_OP_GT = 0, RULE_OP_LT = 1, RULE_OP_MISSING = 2, RULE_OP_TRUE = 3, RULE_OP_FALSE = 4 };
enum RuleRelay { RULE_BOILER_PUMP = 0, RULE_HEATING_PUMP = 1, RULE_FANS = 2 };

struct RuleCond {
    int8_t a = 0;             // variable index 0..RULE_VAR_COUNT-1
    int8_t b = -1;            // -1: use a alone; 0..8: use (a - b)
    uint8_t op = RULE_OP_GT;
    float  value = 0;
    float  hyst = 0;
    bool   state = false;     // runtime latch (not persisted)
};
struct RuleGroup { uint8_t n = 0; RuleCond c[RULE_MAX_CONDS]; };
struct RelayRule { uint8_t n = 0; RuleGroup g[RULE_MAX_GROUPS]; };
struct RuleSet   { RelayRule relay[RULE_RELAYS]; };

inline const char* ruleRelayKey(int r) {
    static const char* const k[RULE_RELAYS] = { "boiler_pump", "heating_pump", "fans" };
    return (r >= 0 && r < RULE_RELAYS) ? k[r] : "";
}

// Evaluates one condition against vars[RULE_VAR_COUNT] (NAN = missing), updating its latch
inline bool ruleEvalCond(RuleCond& c, const float* vars) {
    float x = vars[c.a];
    if (c.b >= 0) x -= vars[c.b];
    bool missing = isnan(x) || isinf(x);
    if (c.op == RULE_OP_TRUE || c.op == RULE_OP_FALSE) {      // boolean variable: stored as 1 / 0
        c.state = !missing && ((x != 0) == (c.op == RULE_OP_TRUE));
        return c.state;
    }
    if (c.op == RULE_OP_MISSING) { c.state = missing; return missing; }
    if (missing) { c.state = false; return false; }
    if (c.op == RULE_OP_GT) {
        if (x > c.value) c.state = true;
        else if (x < c.value - c.hyst) c.state = false;
    } else {
        if (x < c.value) c.state = true;
        else if (x > c.value + c.hyst) c.state = false;
    }
    return c.state;
}

// Every condition is evaluated (no short-circuit) so all latches keep tracking
inline bool ruleEvalRelay(RelayRule& r, const float* vars) {
    bool any = false;
    for (uint8_t gi = 0; gi < r.n; gi++) {
        RuleGroup& g = r.g[gi];
        bool all = g.n > 0;
        for (uint8_t ci = 0; ci < g.n; ci++) if (!ruleEvalCond(g.c[ci], vars)) all = false;
        if (all) any = true;
    }
    return any;
}

// nullptr when coherent, otherwise a message for the user
inline const char* ruleValidate(const RuleSet& rs) {
    for (int r = 0; r < RULE_RELAYS; r++) {
        const RelayRule& rr = rs.relay[r];
        if (rr.n > RULE_MAX_GROUPS) return "Demasiados grupos de reglas";
        for (uint8_t gi = 0; gi < rr.n; gi++) {
            const RuleGroup& g = rr.g[gi];
            if (g.n == 0) return "Hay un grupo sin condiciones";
            if (g.n > RULE_MAX_CONDS) return "Demasiadas condiciones en un grupo";
            for (uint8_t ci = 0; ci < g.n; ci++) {
                const RuleCond& c = g.c[ci];
                if (c.a < 0 || c.a >= RULE_VAR_COUNT) return "Variable de regla no válida";
                if (c.b != -1 && (c.b < 0 || c.b >= RULE_TEMP_VARS)) return "Segunda variable de regla no válida";
                if (c.b >= 0 && c.a >= RULE_TEMP_VARS) return "Solo se pueden restar temperaturas";
                if (c.op > RULE_OP_FALSE) return "Operador de regla no válido";
                bool boolVar = c.a >= RULE_TEMP_VARS;
                bool boolOp  = c.op == RULE_OP_TRUE || c.op == RULE_OP_FALSE;
                if (boolVar && !boolOp) return "«Bomba encendida» y «hay combustión» solo admiten sí / no";
                if (!boolVar && boolOp) return "Sí / no solo vale para «bomba encendida» y «hay combustión»";
                if (isnan(c.value) || isinf(c.value) || c.value < -100 || c.value > 1000) return "Valor de regla fuera de rango (-100 a 1000)";
                if (isnan(c.hyst) || isinf(c.hyst) || c.hyst < 0 || c.hyst > 100) return "Histéresis de regla fuera de rango (0 a 100)";
            }
        }
    }
    return nullptr;
}

inline void ruleAddGroup(RelayRule& r, std::initializer_list<RuleCond> conds) {
    if (r.n >= RULE_MAX_GROUPS) return;
    RuleGroup& g = r.g[r.n++];
    g.n = 0;
    for (const RuleCond& c : conds) if (g.n < RULE_MAX_CONDS) g.c[g.n++] = c;
}
inline RuleCond ruleCond(int a, bool greater, float value, float hyst = 0, int b = -1) {
    RuleCond c; c.a = a; c.b = b; c.op = greater ? RULE_OP_GT : RULE_OP_LT; c.value = value; c.hyst = hyst; return c;
}
inline RuleCond ruleCondBool(int a, bool wanted) {
    RuleCond c; c.a = a; c.op = wanted ? RULE_OP_TRUE : RULE_OP_FALSE; return c;
}
inline RuleCond ruleCondMissing(int a) {
    RuleCond c; c.a = a; c.op = RULE_OP_MISSING; return c;
}

// Default rules. Variables: 0 boiler supply, 2 boiler iron, 3 floor supply, 4 floor return,
// 9 boiler pump on, 10 burning.
//
// Boiler pump (circulates while there is heat to move):
//   - combustion detected, OR
//   - boiler iron above 45 (off below 40): keeps extracting residual heat, OR
//   - boiler supply above 40 (off below 37).
// Heating pump (sends water to the radiant floor) needs ALL of:
//   - boiler pump running,
//   - boiler water above 45 °C to start (it should sit above 50; stops below 40: not worth it),
//   - boiler supply - floor return above 5 °C (stops when the difference falls under 2 °C: the
//     floor would get no heat; restarts when the boiler climbs again). With no floor-return
//     probe the difference cannot be checked, so it is ignored,
//   - floor supply below 35 °C: the radiant floor must never be fed above 35 (cuts above 35,
//     back on below 33). Fail-safe: with no floor-supply reading the pump stays off.
// Fans: while there is combustion, OR boiler iron above 40 (off below 35), OR the upper ambient
//   (7) above 35 (off below 32) — keeps the boiler room ventilated while heat builds up.
inline RuleSet ruleDefaults() {
    RuleSet rs;
    RelayRule& bp = rs.relay[RULE_BOILER_PUMP];
    ruleAddGroup(bp, { ruleCondBool(RULE_VAR_BURNING, true) });
    ruleAddGroup(bp, { ruleCond(2, true, 45, 5) });
    ruleAddGroup(bp, { ruleCond(0, true, 40, 3) });

    RelayRule& hp = rs.relay[RULE_HEATING_PUMP];
    ruleAddGroup(hp, { ruleCondBool(RULE_VAR_BOILER_PUMP, true), ruleCond(0, true, 45, 5),
                       ruleCond(0, true, 5, 3, 4),  ruleCond(3, false, 33, 2) });
    ruleAddGroup(hp, { ruleCondBool(RULE_VAR_BOILER_PUMP, true), ruleCond(0, true, 45, 5),
                       ruleCondMissing(4),          ruleCond(3, false, 33, 2) });

    ruleAddGroup(rs.relay[RULE_FANS], { ruleCondBool(RULE_VAR_BURNING, true) });
    ruleAddGroup(rs.relay[RULE_FANS], { ruleCond(2, true, 40, 5) });
    ruleAddGroup(rs.relay[RULE_FANS], { ruleCond(7, true, 35, 3) });
    return rs;
}

#endif // RULES_H
