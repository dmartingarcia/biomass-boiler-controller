// Host-side unit tests for the hardware-independent logic (no ESP32 needed).
// Run with:  test/host/run.sh
#include "Arduino.h"
unsigned long fakeMs = 0;
#include "Preferences.h"
#include "config.h"
#include "pid_controller.h"
#include "pid_autotune.h"
#include "history.h"
#include "relay_config.h"

static int failures = 0;
#define CHECK(cond, msg) do { if (!(cond)) { printf("  FAIL: %s\n", msg); failures++; } else printf("  ok:   %s\n", msg); } while (0)

// Captures what history.writeJson() prints
struct Capture : public Print {
    std::string s;
    size_t write(const uint8_t* b, size_t n) override { s.append((const char*)b, n); return n; }
};

static void test_pid() {
    printf("PID controller\n");
    PIDController pid(2.0, 0.1, 1.0, 0, 100, 1000);
    double temp = 20, sp = 85, out = 0;
    for (int i = 0; i < 900; i++) { fakeMs += 1000; out = pid.compute(temp, sp); temp += (out * 1.2 - (temp - 20) * 0.8) * 0.02; }
    CHECK(fabs(temp - sp) < 1.0, "converges to the setpoint without steady-state error");
    CHECK(out >= 0 && out <= 100, "output stays inside 0..100");

    pid.reset(40); fakeMs += 1000;
    CHECK(fabs(pid.compute(85, 85) - 40) < 1e-6, "reset(x) is bumpless: first output continues from x");

    PIDController sat(5.0, 5.0, 0, 0, 100, 1000);
    for (int i = 0; i < 200; i++) { fakeMs += 1000; sat.compute(0, 100); }     // long saturation at 100
    fakeMs += 1000;
    CHECK(sat.compute(200, 100) < 100, "no integral wind-up: leaves saturation as soon as error flips");
}

static void test_history() {
    printf("Temperature history (3 tiers)\n");
    TemperatureHistory h; h.begin();
    int16_t t[HISTORY_CHANNELS];
    const int N = 2200;                                   // 2200 * 5 s ≈ 3 h 03 min
    for (int i = 1; i <= N; i++) {
        for (int c = 0; c < HISTORY_CHANNELS; c++) t[c] = 100 + c;
        t[0] = i;                                         // ramp, so the averages are predictable
        fakeMs += HISTORY_SAMPLE_INTERVAL_MS; h.record(t, i % 101, i & 3);
    }
    Capture u, f, c; h.writeTierJson(u, "ultra"); h.writeTierJson(f, "fine"); h.writeTierJson(c, "coarse");
    CHECK(u.s.find("\"ultra\":{\"step\":5,\"age\":0,\"n\":360") != std::string::npos, "ultra: 360 samples every 5 s (30 min), newest age 0");
    CHECK(u.s.find("\"boiler_supply\":[1841,1842") != std::string::npos, "ultra keeps only the newest 360 (oldest is #1841)");
    CHECK(f.s.find("\"fine\":{\"step\":30,\"age\":20,\"n\":360") != std::string::npos, "fine: 360 samples every 30 s (3 h)");
    CHECK(f.s.find("\"boiler_supply\":[39,45,51") != std::string::npos, "fine = average of 6 ultra samples (oldest kept is the mean of #37..#42)");
    CHECK(c.s.find("\"coarse\":{\"step\":300,\"age\":200,\"n\":36") != std::string::npos, "coarse: one sample per 5 min (average of 10 fine)");
    CHECK(c.s.find("\"boiler_supply\":[30,90,150") != std::string::npos, "coarse values are the mean of the right fine samples");
    CHECK(!h.writeTierJson(u, "bogus"), "unknown tier rejected");

    // invalid readings are skipped when averaging
    TemperatureHistory g; g.begin();
    for (int i = 1; i <= 6; i++) { for (int k = 0; k < HISTORY_CHANNELS; k++) t[k] = 100; t[1] = (i == 3) ? HISTORY_INVALID : 500; fakeMs += 5000; g.record(t, 0, 0); }
    Capture gc; g.writeTierJson(gc, "fine");
    CHECK(gc.s.find("\"boiler_return\":[500]") != std::string::npos, "invalid samples are ignored when averaging");
    CHECK(TemperatureHistory::encode(NAN, true) == HISTORY_INVALID, "NaN is stored as invalid");

    // worst-case JSON (4-digit and negative temperatures, all flags) must fit the static buffer
    TemperatureHistory w; w.begin();
    for (int i = 0; i < 6 * 10 * 288 + 50; i++) {
        for (int k = 0; k < HISTORY_CHANNELS; k++) t[k] = (k & 1) ? -1234 : 1234;
        fakeMs += 5000; w.record(t, 100, 31);
    }
    size_t worst = 0;
    for (const char* name : { "ultra", "fine", "coarse" }) { Capture cp; w.writeTierJson(cp, name); if (cp.s.size() > worst) worst = cp.s.size(); }
    printf("        (largest tier JSON: %zu bytes, buffer %d)\n", worst, HISTORY_JSON_BUFFER);
    CHECK(worst < (size_t)HISTORY_JSON_BUFFER, "largest tier JSON fits HISTORY_JSON_BUFFER");
}

static void test_relay_config() {
    printf("Relay channel mapping\n");
    RelayConfig c;
    auto perm = [&]() { bool u[4] = {}; for (int r = 0; r < 4; r++) { int x = c.channelFor(r); if (x < 0 || x > 3 || u[x]) return false; u[x] = true; } return true; };
    CHECK(c.channelFor(ROLE_IDX_BOILER_PUMP) == 1 && c.channelFor(ROLE_IDX_OTHER) == 0, "defaults reproduce the original wiring");
    c.assign(ROLE_IDX_BOILER_PUMP, 3);
    CHECK(c.channelFor(ROLE_IDX_BOILER_PUMP) == 3 && c.channelFor(ROLE_IDX_FANS) == 1, "taking a used channel swaps the two roles");
    RelayConfig d; d.begin();
    CHECK(d.channelFor(ROLE_IDX_BOILER_PUMP) == 3, "assignment is persisted");
    srand(1); bool ok = true;
    for (int i = 0; i < 1000; i++) { c.assign(rand() % 4, rand() % 4); ok &= perm(); }
    CHECK(ok, "mapping is always a valid permutation");
    CHECK(!c.assign(4, 0) && !c.assign(0, 4), "bad role/channel rejected");
    Preferences::db()["boiler_pump"] = 1; Preferences::db()["heating_pump"] = 1;
    RelayConfig e; e.begin();
    CHECK(e.channelFor(ROLE_IDX_BOILER_PUMP) == 1 && e.channelFor(ROLE_IDX_HEATING_PUMP) == 2, "corrupt stored mapping falls back to defaults");
}

static void test_autotune() {
    printf("PID auto-tune (relay method)\n");
    // Plant: first-order process with a transport delay, driven by the tuner's output
    double input = 20, output = 0, setpoint = 85;
    PIDAutoTune at; at.init(&input, &output, setpoint, 50, 1.0, PIDAutoTune::PI_TYPE); at.start();
    double delayed[6] = {0};
    bool done = false; int ticks = 0;
    for (; ticks < 20000 && !done; ticks++) {
        fakeMs += 500;
        done = at.compute();
        for (int i = 5; i > 0; i--) delayed[i] = delayed[i - 1]; delayed[0] = output;
        input += (delayed[5] * 1.5 - (input - 20)) * 0.01;
    }
    CHECK(done, "tuner completes on a simulated process");
    CHECK(at.getKp() > 0 && at.getKp() <= 50 && isfinite(at.getKp()), "Kp positive, finite and within the sanity clamp");
    CHECK(at.getKi() >= 0 && at.getKi() <= 5 && isfinite(at.getKi()), "Ki within the sanity clamp");
    printf("        (Kp=%.3f Ki=%.4f after %d ticks)\n", at.getKp(), at.getKi(), ticks);
}

int main() {
    test_pid(); test_history(); test_relay_config(); test_autotune();
    printf("\n%s (%d failure%s)\n", failures ? "FAILED" : "ALL PASSED", failures, failures == 1 ? "" : "s");
    return failures ? 1 : 0;
}
