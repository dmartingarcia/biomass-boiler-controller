#ifndef AIR_INTAKE_H
#define AIR_INTAKE_H

#include <Arduino.h>
#include <ESP32Servo.h>
#include "pid_controller.h"
#include "pid_autotune.h" // Our own auto-tuning implementation
#include "config.h"

class AirIntake {
public:
    AirIntake() : pid(PID_KP, PID_KI, PID_KD, 0, 100, PID_SAMPLE_TIME) {
        setpoint = DEFAULT_TARGET_BURNING_TEMP;
        servoMin = DEFAULT_SERVO_MIN;
        servoMax = DEFAULT_SERVO_MAX;
    }

    void begin() {
        // Attach servo to pin
        servo.attach(SERVO_AIR_INTAKE);

        // Configure PID
        pid.setSampleTime(PID_SAMPLE_TIME);
        pid.setOutputLimits(0, 100);  // Output in percentage (0-100%)
        pid.reset(0);

        // Initialize servo in closed position
        setServoPosition(0);  // 0% = air intake closed
    }

    void update(float currentBurningTemperature) {
        input = currentBurningTemperature;

        if (tuningInProgress) {
            unsigned long now = millis();

            if (now - lastTuneTime > PID_SAMPLE_TIME) {
                lastTuneTime = now;

                // Execute one step of auto-tuning
                bool tuningResult = autoTune.compute();

                // If auto-tuning is complete
                if (tuningResult) {
                    tuningInProgress = false;

                    // Get the calculated PID parameters
                    double kp = autoTune.getKp();
                    double ki = autoTune.getKi();
                    double kd = autoTune.getKd();

                    // Update the PID controller with new parameters
                    pid.setTunings(kp, ki, kd);

                    // Save parameters for reference
                    currentKp = kp;
                    currentKi = ki;
                    currentKd = kd;

                    // Restore normal control, continuing from the current position
                    pid.reset(currentPosition);
                    tuneFinished = true;
                }

                // Update servo position
                setServoPosition(output);
            }
        } else {
            // Normal PID operation
            output = pid.compute(input, setpoint);
            setServoPosition(output);
        }
    }

    void setTargetTemperature(float temperature) {
        setpoint = temperature;
    }

    float getTargetTemperature() const {
        return setpoint;
    }

    float getCurrentOutput() const {
        return currentPosition;
    }

    // Start the PID auto-tuning process
    bool startAutoTune() {
        // Only start if there's no auto-tuning in progress
        if (!tuningInProgress) {
            output = currentPosition;

            // Configure auto-tuning with our implementation
            PIDAutoTune::ControlType controlType = PID_CONTROL_TYPE == 0 ? PIDAutoTune::PID_TYPE :
                                                 (PID_CONTROL_TYPE == 1 ? PIDAutoTune::PI_TYPE :
                                                                         PIDAutoTune::P_TYPE);

            autoTune.init(&input, &output, setpoint, PID_OUTPUT_STEP, PID_NOISE_BAND, controlType);
            autoTune.start();

            tuningInProgress = true;
            lastTuneTime = millis();

            return true;
        }
        return false;
    }

    // Cancel an auto-tuning in progress
    void cancelAutoTune() {
        if (tuningInProgress) {
            tuningInProgress = false;
            autoTune.cancel();

            // Restore normal PID control, continuing from the current position
            pid.reset(currentPosition);
        }
    }

    // True once after an auto-tune completes; gives the gains so the caller can persist them
    bool takeTuneResult(double& kp, double& ki, double& kd) {
        if (!tuneFinished) return false;
        tuneFinished = false;
        kp = currentKp; ki = currentKi; kd = currentKd;
        return true;
    }

    // Check if there's an auto-tuning in progress
    bool isAutoTuning() const {
        return tuningInProgress;
    }

    // Get current PID parameters
    double getKp() const { return currentKp; }
    double getKi() const { return currentKi; }
    double getKd() const { return currentKd; }

    // Set PID parameters at runtime
    void setTunings(double kp, double ki, double kd) {
        currentKp = kp; currentKi = ki; currentKd = kd;
        pid.setTunings(kp, ki, kd);
    }

    // Directly set servo position (for emergency control from outside)
    void setPosition(int percentage) {
        setServoPosition(percentage);
    }

    // Restart the PID from `startPercent` (bumpless). Use 0 when combustion
    // restarts, or the current servo position when leaving manual mode.
    void resetPid(double startPercent) {
        output = startPercent;
        pid.reset(startPercent);
    }

    // Convenience: resume automatic control from wherever the servo is now
    void resumeFromCurrent() { resetPid(currentPosition); }

    // Set both limits at once (setServoMin/Max alone can reject a valid pair depending on order)
    bool setServoRange(int minPos, int maxPos) {
        if (minPos < 0 || maxPos > 180 || minPos >= maxPos) return false;
        servoMin = minPos;
        servoMax = maxPos;
        return true;
    }

    // Set minimum servo position (0% air)
    void setServoMin(int minPos) {
        if (minPos >= 0 && minPos < servoMax) {
            servoMin = minPos;
        }
    }

    // Set maximum servo position (100% air)
    void setServoMax(int maxPos) {
        if (maxPos > servoMin && maxPos <= 180) {
            servoMax = maxPos;
        }
    }

    // Get current minimum servo position
    int getServoMin() const {
        return servoMin;
    }

    // Get current maximum servo position
    int getServoMax() const {
        return servoMax;
    }

private:
    // Convert percentage (0-100%) to actual servo angle (servoMin-servoMax)
    void setServoPosition(int percentage) {
        // Ensure percentage is within bounds
        percentage = constrain(percentage, 0, 100);

        // Map percentage to servo position
        int position = map(percentage, 0, 100, servoMin, servoMax);

        // Set servo to the calculated position
        servo.write(position);
        currentPosition = percentage;
    }

    Servo servo;
    double input = 0;     // Current combustion temperature
    double output = 0;    // Air intake percentage (0-100%)
    double setpoint = 0;  // Target combustion temperature
    PIDController pid;
    PIDAutoTune autoTune;

    bool tuningInProgress = false;
    bool tuneFinished = false;
    unsigned long lastTuneTime = 0;
    int currentPosition = 0;  // Current percentage position

    // Servo range limits
    int servoMin;  // Servo position at 0% air
    int servoMax;  // Servo position at 100% air

    // Current PID parameters
    double currentKp = PID_KP;
    double currentKi = PID_KI;
    double currentKd = PID_KD;
};

#endif // AIR_INTAKE_H