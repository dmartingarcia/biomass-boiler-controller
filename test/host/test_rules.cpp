// Host-side tests for the relay rule engine (rules.h) and its JSON format (rules_io.h).
#include "Arduino.h"
unsigned long fakeMs = 0;
#include "config.h"
#include "rules.h"
#include "sensor_hold.h"
#ifndef NO_JSON_TESTS
#define RULES_IO_NO_NVS
#include <ArduinoJson.h>
#include <string>
#include "rules_io.h"
#endif

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); failures++; } else printf("  ok:   %s\n", msg); } while (0)

static void vars(float* v, float supply, float ret, float iron, float floorSup, float floorRet, float mix, float amb1, float amb2, float gas, bool burning) {
    float t[9] = { supply, ret, iron, floorSup, floorRet, mix, amb1, amb2, gas };
    for (int i = 0; i < 9; i++) v[i] = t[i];
    v[RULE_VAR_BOILER_PUMP] = 0; v[RULE_VAR_BURNING] = burning ? 1 : 0;
}

static void test_condition() {
    printf("Condition: thresholds and hysteresis\n");
    float v[RULE_VAR_COUNT]; vars(v, 20, 20, 20, 20, 20, 20, 20, 20, 20, false);
    RuleCond gt = ruleCond(8, true, 100, 5);
    v[8] = 100;  CHECK(!ruleEvalCond(gt, v), "'>' is strict: equal to the value is false");
    v[8] = 101;  CHECK(ruleEvalCond(gt, v),  "'>' turns on above the value");
    v[8] = 97;   CHECK(ruleEvalCond(gt, v),  "'>' stays on inside the hysteresis band");
    v[8] = 95;   CHECK(ruleEvalCond(gt, v),  "'>' stays on exactly at value - hysteresis");
    v[8] = 94.9f; CHECK(!ruleEvalCond(gt, v), "'>' turns off below value - hysteresis");
    v[8] = 98;   CHECK(!ruleEvalCond(gt, v), "'>' does not turn back on inside the band");
    RuleCond lt = ruleCond(8, false, 30, 4);
    v[8] = 29;   CHECK(ruleEvalCond(lt, v),  "'<' turns on below the value");
    v[8] = 33;   CHECK(ruleEvalCond(lt, v),  "'<' stays on inside the band");
    v[8] = 34.5f; CHECK(!ruleEvalCond(lt, v), "'<' turns off above value + hysteresis");
    RuleCond nohyst = ruleCond(0, true, 50, 0);
    v[0] = 50.1f; bool on = ruleEvalCond(nohyst, v); v[0] = 49.9f; bool off = ruleEvalCond(nohyst, v);
    CHECK(on && !off, "hysteresis 0 switches exactly at the value");
}

static void test_missing_and_diff() {
    printf("Condition: missing readings and differences\n");
    float v[RULE_VAR_COUNT]; vars(v, 80, 50, 40, 30, 60, 45, 20, 22, 150, true);
    RuleCond diff = ruleCond(0, true, 5, 0, 4);          // supply - floor return
    CHECK(ruleEvalCond(diff, v), "a - b: 80 - 60 = 20 > 5");
    v[4] = 76; CHECK(!ruleEvalCond(diff, v), "a - b: 80 - 76 = 4 is not > 5");
    v[4] = NAN; RuleCond d2 = ruleCond(0, true, 5, 0, 4);
    CHECK(!ruleEvalCond(d2, v), "missing b makes the difference false");
    v[0] = NAN; v[4] = 10; RuleCond d3 = ruleCond(0, true, 5, 0, 4);
    CHECK(!ruleEvalCond(d3, v), "missing a makes the difference false");
    RuleCond latched = ruleCond(8, true, 100, 50);
    v[8] = 120; ruleEvalCond(latched, v); v[8] = NAN;
    CHECK(!ruleEvalCond(latched, v) && !latched.state, "a lost reading turns a latched condition off");
    v[8] = 90;
    CHECK(!ruleEvalCond(latched, v), "after a lost reading it must cross the threshold again (no stale latch)");
    RuleCond lt = ruleCond(RULE_VAR_BURNING, false, 0.5f);
    v[RULE_VAR_BURNING] = 0; CHECK(ruleEvalCond(lt, v), "pseudo-variable 'burning' = 0 satisfies '< 0.5'");
}

static void test_groups() {
    printf("Relay: AND inside a group, OR between groups\n");
    float v[RULE_VAR_COUNT];
    RelayRule r;
    CHECK(!ruleEvalRelay(r, v) , "a relay with no groups is OFF");
    ruleAddGroup(r, { ruleCond(0, true, 40), ruleCond(2, true, 30) });          // supply>40 AND iron>30
    vars(v, 50, 0, 35, 0, 0, 0, 0, 0, 0, false);  CHECK(ruleEvalRelay(r, v),  "AND: both true -> ON");
    vars(v, 50, 0, 25, 0, 0, 0, 0, 0, 0, false);  CHECK(!ruleEvalRelay(r, v), "AND: one false -> OFF");
    ruleAddGroup(r, { ruleCond(0, true, 70) });                                 // OR supply>70
    vars(v, 75, 0, 25, 0, 0, 0, 0, 0, 0, false);  CHECK(ruleEvalRelay(r, v),  "OR: second group alone -> ON");
    vars(v, 60, 0, 25, 0, 0, 0, 0, 0, 0, false);  CHECK(!ruleEvalRelay(r, v), "OR: neither group -> OFF");

    // No short-circuit: every latch must keep tracking even when an earlier cond is false
    RelayRule s;
    ruleAddGroup(s, { ruleCond(0, true, 100), ruleCond(2, true, 30, 10) });
    vars(v, 50, 0, 35, 0, 0, 0, 0, 0, 0, false); ruleEvalRelay(s, v);
    CHECK(s.g[0].c[1].state, "later conditions are evaluated even if an earlier one is false");

    RelayRule e;
    e.n = 1; e.g[0].n = 0;
    CHECK(!ruleEvalRelay(e, v), "an empty group never turns the relay on");
}

static void test_missing_operator() {
    printf("Operator 'missing' (no reading)\n");
    float v[RULE_VAR_COUNT]; vars(v, 60, 30, 40, 25, 35, 40, 20, 22, 90, false);
    RuleCond m = ruleCondMissing(4);
    CHECK(!ruleEvalCond(m, v), "reading present -> 'missing' is false");
    v[4] = NAN; CHECK(ruleEvalCond(m, v), "reading lost -> 'missing' is true");
    v[4] = 35;  CHECK(!ruleEvalCond(m, v), "reading back -> false again");
    RuleCond md = ruleCondMissing(0); md.b = 4;
    v[4] = NAN; CHECK(ruleEvalCond(md, v), "a - b is missing if b is missing");
    v[4] = 35; v[0] = NAN; CHECK(ruleEvalCond(md, v), "a - b is missing if a is missing");
    v[0] = 60; CHECK(!ruleEvalCond(md, v), "a - b present -> false");
    RuleSet ok; ruleAddGroup(ok.relay[0], { ruleCondMissing(4) });
    CHECK(!ruleValidate(ok), "'missing' on a temperature validates");
    RuleSet bad; ruleAddGroup(bad.relay[0], { ruleCondMissing(RULE_VAR_BURNING) });
    CHECK(ruleValidate(bad), "'missing' on a pseudo-variable rejected (it always has a value)");
    // Heating pump default: no floor probe -> runs by the water-hot group, ignoring the delta
    RuleSet rs = ruleDefaults();
    vars(v, 55, 30, 20, 25, NAN, 40, 20, 22, 120, true); v[RULE_VAR_BOILER_PUMP] = 1;
    CHECK(ruleEvalRelay(rs.relay[RULE_HEATING_PUMP], v), "default heating rule: return probe missing + boiler pump + hot water -> ON");
    vars(v, 35, 30, 20, 25, NAN, 40, 20, 22, 120, true); v[RULE_VAR_BOILER_PUMP] = 1;
    CHECK(!ruleEvalRelay(rs.relay[RULE_HEATING_PUMP], v), "...but not while the water is cold");
}

static void test_sensor_hold() {
    printf("Sensor hold: keep the last good reading for up to 60 s\n");
    SensorHold h; unsigned long t = 1000;
    CHECK(isnan(h.update(NAN, false, t)), "never had a reading -> missing");
    CHECK(h.update(55.5f, true, t) == 55.5f, "valid reading passes through");
    t += 1000;  CHECK(h.update(NAN, false, t) == 55.5f, "1 s after failing: held");
    t += 30000; CHECK(h.update(NAN, false, t) == 55.5f, "31 s after: still held");
    t += 28999; CHECK(h.update(-127, false, t) == 55.5f, "59.999 s after: still held");
    t += 1;     CHECK(isnan(h.update(NAN, false, t)), "60 s after: reported missing");
    t += 5000;  CHECK(isnan(h.update(NAN, false, t)), "stays missing");
    CHECK(h.update(60, true, t) == 60, "recovers as soon as a valid reading returns");
    t += 20000; h.update(NAN, false, t);
    t += 30000; CHECK(h.update(NAN, false, t) == 60, "a new failure restarts the 60 s from the last good reading");
    t += 40000; CHECK(isnan(h.update(NAN, false, t)), "...and expires 60 s after that reading, not before");
    SensorHold nn; nn.update(50, true, t);
    CHECK(nn.update(NAN, true, t + 90000) != nn.update(NAN, true, t + 90000), "a NAN flagged valid is never accepted as a reading");
    SensorHold w; w.update(40, true, 4294967000UL);
    CHECK(w.update(NAN, false, 4294967000UL + 30000UL) == 40, "millis() wrap-around: still held after the counter rolls over");
    CHECK(isnan(w.update(NAN, false, 4294967000UL + 61000UL)), "millis() wrap-around: expires correctly");
    // Effect on relay behaviour: a 20 s glitch does not stop the fans, a 70 s one does
    SensorHold g; RuleSet rs; ruleAddGroup(rs.relay[RULE_FANS], { ruleCond(8, true, 100, 5) }); float v[RULE_VAR_COUNT];
    unsigned long n = 0; bool fans = false; bool glitchOk = true;
    for (int sec = 0; sec < 200; sec++) {
        n += 1000;
        bool failing = (sec >= 20 && sec < 40) || (sec >= 100 && sec < 170);
        float gas = g.update(failing ? NAN : 130, !failing, n);
        vars(v, 50, 30, 20, 25, 30, 40, 20, 22, gas, false);
        fans = ruleEvalRelay(rs.relay[RULE_FANS], v);
        if (sec == 39 && !fans) glitchOk = false;               // 20 s glitch: held
        if (sec == 100 + 58 && !fans) glitchOk = false;          // 59 s after the last good reading: still held
        if (sec == 100 + 61 && fans) glitchOk = false;           // >60 s: off
        if (sec == 175 && !fans) glitchOk = false;               // recovered: on again
    }
    CHECK(glitchOk, "fans ride through a 20 s thermocouple glitch, stop after 60 s without reading, resume on recovery");
}

static void test_validate() {
    printf("Validation\n");
    RuleSet ok; ruleAddGroup(ok.relay[0], { ruleCond(0, true, 50, 2) });
    CHECK(!ruleValidate(ok), "a sane rule passes");
    RuleSet bad = ok; bad.relay[0].g[0].c[0].a = 11;     CHECK(ruleValidate(bad), "variable out of range rejected");
    bad = ok; bad.relay[0].g[0].c[0].a = -1;             CHECK(ruleValidate(bad), "negative variable rejected");
    bad = ok; bad.relay[0].g[0].c[0].b = 9;              CHECK(ruleValidate(bad), "second variable must be a temperature (0-8)");
    bad = ok; bad.relay[0].g[0].c[0].a = 10; bad.relay[0].g[0].c[0].b = 0; CHECK(ruleValidate(bad), "cannot subtract from a pseudo-variable");
    bad = ok; bad.relay[0].g[0].c[0].value = NAN;        CHECK(ruleValidate(bad), "NaN value rejected");
    bad = ok; bad.relay[0].g[0].c[0].value = 5000;       CHECK(ruleValidate(bad), "huge value rejected");
    bad = ok; bad.relay[0].g[0].c[0].hyst = -1;          CHECK(ruleValidate(bad), "negative hysteresis rejected");
    bad = ok; bad.relay[0].g[0].n = 0;                   CHECK(ruleValidate(bad), "group without conditions rejected");
    bad = ok; bad.relay[1].n = RULE_MAX_GROUPS + 1;      CHECK(ruleValidate(bad), "too many groups rejected");
    bad = ok; bad.relay[0].g[0].n = RULE_MAX_CONDS + 1;  CHECK(ruleValidate(bad), "too many conditions rejected");
}

// Evaluates the default rules once with fresh latches. Boiler supply, iron, floor supply, floor return, burning.
struct Out { bool bp, hp, fans; };
static Out defaults(float supply, float iron, float floorSup, float floorRet, bool burning, float ambHigh = 22) {
    RuleSet rs = ruleDefaults();
    float v[RULE_VAR_COUNT]; vars(v, supply, 30, iron, floorSup, floorRet, 40, 20, ambHigh, 90, burning);
    Out o; o.bp = ruleEvalRelay(rs.relay[RULE_BOILER_PUMP], v);
    v[RULE_VAR_BOILER_PUMP] = o.bp;
    o.hp = ruleEvalRelay(rs.relay[RULE_HEATING_PUMP], v);
    o.fans = ruleEvalRelay(rs.relay[RULE_FANS], v);
    return o;
}

static void test_default_rules() {
    printf("Default rules (example set)\n");
    CHECK(!ruleValidate(ruleDefaults()), "the defaults are valid");
    // Boiler pump
    CHECK(!defaults(30, 20, 25, 25, false).bp, "cold boiler, no fire: boiler pump off");
    CHECK(defaults(30, 20, 25, 25, true).bp,   "fire lit: boiler pump on");
    CHECK(defaults(30, 50, 25, 25, false).bp,  "hot iron (>45) after the fire is out: boiler pump on");
    CHECK(defaults(42, 20, 25, 25, false).bp,  "boiler supply above 40: boiler pump on");
    // Fans
    CHECK(defaults(60, 20, 25, 25, true).fans,   "fans on while burning");
    CHECK(!defaults(60, 20, 25, 25, false).fans, "fans off without combustion, cool iron and room");
    CHECK(defaults(60, 41, 25, 25, false).fans,  "hot iron (>40) after the fire is out: fans on");
    CHECK(!defaults(60, 39, 25, 25, false).fans, "iron 39: fans stay off");
    CHECK(defaults(60, 20, 25, 25, false, 36).fans, "upper ambient above 35: fans on");
    CHECK(!defaults(60, 20, 25, 25, false, 34).fans, "upper ambient 34: fans stay off");
    RuleSet fr = ruleDefaults(); float fv[RULE_VAR_COUNT]; bool fo = true;
    float iron[] = { 41, 38, 36, 34.9f, 41 }; bool fanExp[] = { 1, 1, 1, 0, 1 };
    for (int i = 0; i < 5; i++) { vars(fv, 50, 30, iron[i], 25, 25, 40, 20, 22, 90, false); if (ruleEvalRelay(fr.relay[RULE_FANS], fv) != fanExp[i]) fo = false; }
    CHECK(fo, "fans on iron: on above 40, off below 35");
    // Heating pump: nominal
    CHECK(defaults(60, 20, 30, 40, true).hp,  "water 60, floor supply 30, return 40 (delta 20): heating on");
    CHECK(!defaults(44, 20, 30, 30, true).hp, "water 44 (<45): not worth sending yet");
    CHECK(defaults(46, 20, 30, 30, true).hp,  "water 46 (>45), delta 16: starts");
    CHECK(!defaults(60, 20, 30, 40, false).hp || defaults(60, 20, 30, 40, false).bp, "heating never runs without the boiler pump");
    CHECK(!defaults(30, 20, 25, 20, false).hp, "cold boiler: heating off");
    // Floor limit: never fed above 35
    CHECK(!defaults(60, 20, 34, 40, true).hp,  "a start with floor supply at 34: waits until it is below 33 (restart level)");
    CHECK(defaults(60, 20, 32, 40, true).hp,   "floor supply 32: starts");
    CHECK(!defaults(60, 20, 35.1f, 40, true).hp, "floor supply above 35: heating cut");
    CHECK(!defaults(80, 20, 40, 40, true).hp,  "floor supply 40: cut even with a very hot boiler");
    CHECK(!defaults(60, 20, NAN, 40, true).hp, "floor-supply probe lost: fail-safe, heating off");
    // Delta (supply - floor return): starts above 5, stops below 2, restarts above 5
    RuleSet rs = ruleDefaults();
    bool seqOk = true;
    float v[RULE_VAR_COUNT];
    auto run = [&](float sup, float ret) { vars(v, sup, 30, 20, 30, ret, 40, 20, 22, 90, true); v[RULE_VAR_BOILER_PUMP] = 1; return ruleEvalRelay(rs.relay[RULE_HEATING_PUMP], v); };
    seqOk &= run(60, 50) == true;      // delta 10: on
    seqOk &= run(60, 57) == true;      // delta 3: inside the band, stays on
    seqOk &= run(60, 58.1f) == false;  // delta 1.9: below 2 -> off
    seqOk &= run(60, 56) == false;     // delta 4: below the 5 restart level -> stays off
    seqOk &= run(60, 54.9f) == true;   // delta 5.1: boiler climbed again -> back on
    CHECK(seqOk, "delta: on above 5, off below 2, restarts only when it climbs back above 5");
    // Water: starts above 45, stops below 40
    RuleSet w = ruleDefaults();
    bool wOk = true;
    float temps[] = { 44, 46, 42, 39.9f, 43, 46 }; bool exp[] = { 0, 1, 1, 0, 0, 1 };
    for (int i = 0; i < 6; i++) { vars(v, temps[i], 30, 20, 25, 20, 40, 20, 22, 90, true); v[RULE_VAR_BOILER_PUMP] = 1; if (ruleEvalRelay(w.relay[RULE_HEATING_PUMP], v) != exp[i]) { wOk = false; printf("        water step %d\n", i); } }
    CHECK(wOk, "water: starts above 45, keeps going down to 40, then stops until it passes 45 again");
    // Floor limit hysteresis: cut above 35, back on below 33
    RuleSet f = ruleDefaults(); bool fOk = true;
    float fs[] = { 30, 35, 35.5f, 34, 32.9f, 34 }; bool fe[] = { 1, 1, 0, 0, 1, 1 };
    for (int i = 0; i < 6; i++) { vars(v, 60, 30, 20, fs[i], 40, 40, 20, 22, 90, true); v[RULE_VAR_BOILER_PUMP] = 1; if (ruleEvalRelay(f.relay[RULE_HEATING_PUMP], v) != fe[i]) { fOk = false; printf("        floor step %d\n", i); } }
    CHECK(fOk, "floor limit: cut above 35, back on only below 33");
    // No floor-return probe: delta cannot be checked, ignored
    CHECK(defaults(60, 20, 30, NAN, true).hp,   "no floor-return probe: delta ignored, heating runs");
    CHECK(!defaults(44, 20, 30, NAN, true).hp,  "...but still not below 45");
    CHECK(!defaults(60, 20, 36, NAN, true).hp,  "...and the 35 limit still applies");
}

#ifndef NO_JSON_TESTS
static std::string toStr(const RuleSet& rs) {
    DynamicJsonDocument d(4096); rulesToJson(rs, d.to<JsonObject>());
    std::string s; serializeJson(d, s); return s;
}
static const char* parse(const std::string& s, RuleSet& out) {
    DynamicJsonDocument d(6144);
    if (deserializeJson(d, s)) return "json";
    return rulesFromJson(d.as<JsonObjectConst>(), out);
}
static void test_json() {
    printf("JSON format and persistence round-trip\n");
    RuleSet a = ruleDefaults();
    std::string js = toStr(a);
    RuleSet b;
    CHECK(!parse(js, b), "serialised defaults parse back");
    CHECK(toStr(b) == js, "round-trip is lossless (what NVS stores is what comes back)");
    CHECK(b.relay[RULE_HEATING_PUMP].n == 2 && b.relay[RULE_HEATING_PUMP].g[0].n == 4 && b.relay[RULE_HEATING_PUMP].g[0].c[2].b == 4,
          "structure preserved (2 groups, 4 conds, a-b)");
    CHECK(js.size() < 3500, "worst realistic payload fits the NVS string and request limits");

    RuleSet full;
    for (int r = 0; r < RULE_RELAYS; r++) for (int g = 0; g < RULE_MAX_GROUPS; g++)
        ruleAddGroup(full.relay[r], { ruleCond(0, true, 123.5f, 2.5f, 4), ruleCond(1, false, -12.25f, 1), ruleCond(2, true, 100, 0), ruleCond(8, true, 999, 5) });
    std::string fs = toStr(full);
    RuleSet fb; CHECK(!parse(fs, fb) && toStr(fb) == fs, "a completely full rule set round-trips");
    CHECK(fs.size() < 4000 - 100, "a completely full rule set fits the 4000-byte NVS string limit");

    RuleSet ms; CHECK(!parse("{\"fans\":[[{\"a\":4,\"op\":\"missing\"}]]}", ms) && ms.relay[RULE_FANS].g[0].c[0].op == RULE_OP_MISSING, "'missing' operator parses (no value needed)");
    CHECK(toStr(ms).find("\"fans\":[[{\"a\":4,\"op\":\"missing\"}]]") != std::string::npos, "'missing' serialises without value/hysteresis");
    CHECK(js.find("\"op\":\"missing\"") != std::string::npos, "default rules include the 'missing' group");

    // Partial update: only the relays present are replaced
    RuleSet p = a;
    CHECK(!parse("{\"fans\":[[{\"a\":0,\"op\":\">\",\"v\":60,\"h\":3}]]}", p), "partial POST with only 'fans' accepted");
    CHECK(p.relay[RULE_FANS].g[0].c[0].a == 0 && p.relay[RULE_FANS].g[0].c[0].value == 60, "fans replaced");
    CHECK(toStr(p).find("\"boiler_pump\":[[{\"a\":10") != std::string::npos && p.relay[RULE_BOILER_PUMP].n == 3, "other relays untouched");
    RuleSet cleared = a;
    CHECK(!parse("{\"fans\":[]}", cleared) && cleared.relay[RULE_FANS].n == 0, "empty list clears a relay (always OFF)");

    // Rejections leave the target untouched
    const char* bads[] = {
        "{\"fans\":[[{\"a\":0,\"op\":\"=\",\"v\":1}]]}",              // bad operator
        "{\"fans\":[[{\"a\":0,\"op\":\">\"}]]}",                      // no value
        "{\"fans\":[[{\"a\":9,\"op\":\"missing\"}]]}",              // missing on a pseudo-variable
        "{\"fans\":[[{\"a\":99,\"op\":\">\",\"v\":1}]]}",             // bad variable
        "{\"fans\":[[{\"a\":0,\"b\":77,\"op\":\">\",\"v\":1}]]}",     // bad second variable
        "{\"fans\":[[{\"op\":\">\",\"v\":1}]]}",                      // no variable
        "{\"fans\":[[]]}",                                            // empty group
        "{\"fans\":\"x\"}",                                           // wrong type
        "{\"fans\":[3]}",                                             // group is not a list
        "{\"fans\":[[7]]}",                                           // condition is not an object
        "{\"fans\":[[{\"a\":0,\"op\":\">\",\"v\":99999}]]}",          // value out of range
        "{\"fans\":[[{\"a\":0,\"op\":\">\",\"v\":1,\"h\":-5}]]}",     // negative hysteresis
        "{\"fans\":[[{\"a\":0,\"op\":\">\",\"v\":1}],[{\"a\":0,\"op\":\">\",\"v\":1}],[{\"a\":0,\"op\":\">\",\"v\":1}],[{\"a\":0,\"op\":\">\",\"v\":1}],[{\"a\":0,\"op\":\">\",\"v\":1}]]}", // 5 groups
        "{\"fans\":[[{\"a\":0,\"op\":\">\",\"v\":1},{\"a\":0,\"op\":\">\",\"v\":1},{\"a\":0,\"op\":\">\",\"v\":1},{\"a\":0,\"op\":\">\",\"v\":1},{\"a\":0,\"op\":\">\",\"v\":1}]]}", // 5 conds
    };
    bool allRejected = true, untouched = true;
    for (const char* bj : bads) {
        RuleSet t = a; std::string before = toStr(t);
        if (!parse(bj, t)) { allRejected = false; printf("        accepted: %s\n", bj); }
        if (toStr(t) != before) { untouched = false; printf("        modified: %s\n", bj); }
    }
    CHECK(allRejected, "all malformed / out-of-range payloads are rejected");
    CHECK(untouched, "a rejected payload never leaves a half-applied rule set");
    RuleSet unknown = a; CHECK(!parse("{\"other\":[[{\"a\":0}]]}", unknown) && toStr(unknown) == js, "unknown keys are ignored");
    RuleSet corrupt; CHECK(parse("{not json", corrupt), "corrupt stored JSON is detected (falls back to defaults at boot)");
}
#endif

int main() {
    test_condition(); test_missing_operator(); test_sensor_hold(); test_missing_and_diff(); test_groups(); test_validate();
    test_default_rules();
#ifndef NO_JSON_TESTS
    test_json();
#else
    printf("(JSON tests skipped: ArduinoJson not found — run `pio run` once to fetch it)\n");
#endif
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
