#ifndef PID_CONTROLLER_H
#define PID_CONTROLLER_H

#include <Arduino.h>

// PID controller for the air-intake servo.
//
// Positive error (measurement below setpoint) -> output rises (more air).
//  - Derivative on measurement (no kick when the setpoint changes)
//  - Integral clamped to the output range (anti-windup)
//  - reset(output) gives bumpless transfer: the next compute() continues from
//    `output` instead of jumping, e.g. when leaving manual mode.
class PIDController {
public:
    PIDController(double kp, double ki, double kd,
                  double outMin = 0.0, double outMax = 100.0,
                  unsigned long sampleMs = 1000)
        : kp(kp), ki(ki), kd(kd), outMin(outMin), outMax(outMax),
          sampleMs(sampleMs), iTerm(0), lastInput(0), lastOutput(0),
          lastTime(0), primed(false) {}

    void setTunings(double newKp, double newKi, double newKd) {
        kp = newKp; ki = newKi; kd = newKd;
        iTerm = clamp(iTerm);
    }

    void setOutputLimits(double lo, double hi) {
        if (lo >= hi) return;
        outMin = lo; outMax = hi;
        iTerm      = clamp(iTerm);
        lastOutput = clamp(lastOutput);
    }

    void setSampleTime(unsigned long ms) { if (ms > 0) sampleMs = ms; }

    // Restart from a known output value (integral is pre-loaded with it).
    void reset(double output = 0.0) {
        lastOutput = clamp(output);
        iTerm      = lastOutput;
        primed     = false;
    }

    // Returns the current output. Recomputes only when the sample time elapsed;
    // otherwise returns the previous output unchanged.
    double compute(double input, double setpoint) {
        unsigned long now = millis();
        if (primed && (now - lastTime) < sampleMs) return lastOutput;

        double dt = primed ? (now - lastTime) / 1000.0 : sampleMs / 1000.0;
        double error = setpoint - input;

        iTerm = clamp(iTerm + ki * error * dt);
        double dInput = primed ? (input - lastInput) / dt : 0.0;

        lastOutput = clamp(kp * error + iTerm - kd * dInput);
        lastInput  = input;
        lastTime   = now;
        primed     = true;
        return lastOutput;
    }

    double getKp() const { return kp; }
    double getKi() const { return ki; }
    double getKd() const { return kd; }
    double getOutput() const { return lastOutput; }

private:
    double clamp(double v) const { return v < outMin ? outMin : (v > outMax ? outMax : v); }

    double kp, ki, kd;
    double outMin, outMax;
    unsigned long sampleMs;
    double iTerm, lastInput, lastOutput;
    unsigned long lastTime;
    bool primed;
};

#endif // PID_CONTROLLER_H
