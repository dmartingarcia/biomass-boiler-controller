#ifndef PID_AUTOTUNE_H
#define PID_AUTOTUNE_H

#include <Arduino.h>

class PIDAutoTune {
public:
    enum ControlType {
        PID_TYPE = 0,
        PI_TYPE = 1,
        P_TYPE = 2
    };

    PIDAutoTune() {
        reset();
    }

    // Initialize auto-tuning
    void init(double* input, double* output, double setpoint, double outputStep, double noiseband, ControlType controlType) {
        _input = input;
        _output = output;
        _setpoint = setpoint;
        _outputStep = outputStep;
        _noiseband = noiseband;
        _controlType = controlType;

        // Initial values
        _state = 0;
        _lastTime = millis();
        _peak1 = 0;
        _peak2 = 0;
        _lastInputs[0] = *_input;
        _peakType = 0;
        _peakCount = 0;
        _initialized = true;
        _running = false;
    }

    // Start auto-tuning
    void start() {
        if (_initialized) {
            _peak1 = 0;
            _peak2 = 0;
            _peakType = 0;
            _peakCount = 0;
            _running = true;
            _state = 0;
            _lastTime = millis();
        }
    }

    // Stop auto-tuning
    void cancel() {
        _running = false;
    }

    // Run one auto-tuning cycle
    bool compute() {
        if (!_running) return false;

        unsigned long now = millis();
        if (now - _lastTime < 500) return false;  // Update every 500ms

        _lastTime = now;

        // Read the current input value
        double refVal = *_input;

        // Check if we are within the noise band
        if (_state == 0) {
            // Waiting for the signal to stabilize
            *_output = _setpoint < refVal ? 0 : _outputStep;

            // If we are within the noise band, move to the next state
            if (abs(refVal - _setpoint) < _noiseband) {
                _state = 1;
                // Apply output step in the opposite direction
                *_output = _setpoint < refVal ? _outputStep : 0;
            }
        }
        else {
            // Look for peaks and valleys

            // Shift previous values
            for (int i = 9; i >= 1; i--) {
                _lastInputs[i] = _lastInputs[i - 1];
            }
            _lastInputs[0] = refVal;

            // Check if we have a peak or valley
            int isMax = 1;
            int isMin = 1;

            for (int i = 1; i < 10; i++) {
                if (_lastInputs[0] < _lastInputs[i]) isMax = 0;
                if (_lastInputs[0] > _lastInputs[i]) isMin = 0;
            }

            // If we have a peak or valley, record it
            if (isMax) {
                if (_peakType == -1) {
                    _peakCount++;
                    _peak2 = _peak1;
                    _peak1 = refVal;
                }
                _peakType = 1;
            }
            else if (isMin) {
                if (_peakType == 1) {
                    _peakCount++;
                    _peak2 = _peak1;
                    _peak1 = refVal;
                }
                _peakType = -1;
            }

            // If we have enough peaks, calculate PID parameters
            if (_peakCount >= 4) {
                // Calculate Ku and Tu values
                double Ku = 4.0 * _outputStep / ((0.5 * (_peak1 - _peak2)) * 3.14159);
                double Tu = (double)(now - _peaks[0]) / 1000.0;

                // Calculate PID parameters using Ziegler-Nichols
                if (_controlType == PID_TYPE) {
                    _kp = 0.6 * Ku;
                    _ki = 1.2 * Ku / Tu;
                    _kd = 0.075 * Ku * Tu;
                }
                else if (_controlType == PI_TYPE) {
                    _kp = 0.45 * Ku;
                    _ki = 0.54 * Ku / Tu;
                    _kd = 0;
                }
                else {  // P_TYPE
                    _kp = 0.5 * Ku;
                    _ki = 0;
                    _kd = 0;
                }

                _running = false;
                return true;  // Auto-tuning complete
            }
            else {
                // Change output based on PID direction
                if (refVal > _setpoint + _noiseband && *_output > 0) {
                    *_output = 0;
                }
                else if (refVal < _setpoint - _noiseband && *_output < _outputStep) {
                    *_output = _outputStep;
                }

                // Record peak times
                if (_peakCount > 0 && _peakCount < 4) {
                    _peaks[_peakCount - 1] = now;
                }
            }
        }

        return false;  // Auto-tuning in progress
    }

    // Get calculated parameters
    double getKp() const { return _kp; }
    double getKi() const { return _ki; }
    double getKd() const { return _kd; }

    // Check if auto-tuning is running
    bool isRunning() const { return _running; }

private:
    void reset() {
        _input = NULL;
        _output = NULL;
        _setpoint = 0;
        _outputStep = 10;
        _noiseband = 0.5;
        _controlType = PI_TYPE;
        _state = 0;
        _peakType = 0;
        _peakCount = 0;
        _initialized = false;
        _running = false;
        _kp = _ki = _kd = 0;

        for (int i = 0; i < 10; i++) {
            _lastInputs[i] = 0;
        }

        for (int i = 0; i < 4; i++) {
            _peaks[i] = 0;
        }
    }

    // Pointers to PID variables
    double* _input;
    double* _output;

    // Configuration parameters
    double _setpoint;
    double _outputStep;
    double _noiseband;
    ControlType _controlType;

    // Internal variables for the algorithm
    int _state;
    int _peakType;
    int _peakCount;
    unsigned long _lastTime;
    unsigned long _peaks[4];
    double _lastInputs[10];
    double _peak1;
    double _peak2;

    // Calculated PID parameters
    double _kp;
    double _ki;
    double _kd;

    // State
    bool _initialized;
    bool _running;
};

#endif // PID_AUTOTUNE_H