/**
 * NixiSat — PID Controller
 *
 * Standard parallel-form PID with:
 *   - Anti-windup integral clamp
 *   - Output saturation
 *   - Derivative-on-measurement (uses gyro rate directly, not error-derivative)
 *     to avoid the "derivative kick" when the setpoint changes.
 *
 * Used for attitude (roll/pitch/yaw) and vertical-speed control.
 */

#ifndef NIXISAT_PID_H
#define NIXISAT_PID_H

#include <Arduino.h>

class PIDController {
public:
    PIDController(float kp, float ki, float kd,
                  float outputLimit, float integralLimit)
        : _kp(kp), _ki(ki), _kd(kd),
          _outputLimit(outputLimit), _integralLimit(integralLimit),
          _integral(0.0f), _lastMeasured(0.0f), _firstRun(true) {}

    /**
     * Update with derivative-on-measurement (rate provided externally — e.g. gyro).
     *
     * @param setpoint   Desired value
     * @param measured   Current sensor value
     * @param rate       Rate of change of `measured` (e.g. gyro reading in deg/s).
     *                   The D term acts on this directly to suppress noise spikes.
     * @param dt         Loop time in seconds
     * @return           Saturated controller output
     */
    float update(float setpoint, float measured, float rate, float dt) {
        float error = setpoint - measured;

        // Proportional
        float p = _kp * error;

        // Integral with clamp
        _integral += error * dt;
        if (_integral >  _integralLimit) _integral =  _integralLimit;
        if (_integral < -_integralLimit) _integral = -_integralLimit;
        float i = _ki * _integral;

        // Derivative on measurement (negate — falling measurement = +derivative)
        float d = -_kd * rate;

        float output = p + i + d;
        if (output >  _outputLimit) output =  _outputLimit;
        if (output < -_outputLimit) output = -_outputLimit;

        _lastMeasured = measured;
        _firstRun = false;
        return output;
    }

    /**
     * Update with derivative-on-error (computes its own derivative).
     * Use when there is no separate rate sensor (e.g. vertical speed).
     */
    float updateDerivOnError(float setpoint, float measured, float dt) {
        float error = setpoint - measured;

        float p = _kp * error;

        _integral += error * dt;
        if (_integral >  _integralLimit) _integral =  _integralLimit;
        if (_integral < -_integralLimit) _integral = -_integralLimit;
        float i = _ki * _integral;

        float d = 0.0f;
        if (!_firstRun && dt > 0.0f) {
            // Note: derivative on -measurement avoids derivative kick on
            // setpoint changes (equivalent for constant setpoints).
            d = -_kd * (measured - _lastMeasured) / dt;
        }

        float output = p + i + d;
        if (output >  _outputLimit) output =  _outputLimit;
        if (output < -_outputLimit) output = -_outputLimit;

        _lastMeasured = measured;
        _firstRun = false;
        return output;
    }

    void reset() {
        _integral = 0.0f;
        _firstRun = true;
        _lastMeasured = 0.0f;
    }

    void setGains(float kp, float ki, float kd) {
        _kp = kp; _ki = ki; _kd = kd;
    }

    float integral() const { return _integral; }

private:
    float _kp, _ki, _kd;
    float _outputLimit;
    float _integralLimit;
    float _integral;
    float _lastMeasured;
    bool  _firstRun;
};

#endif // NIXISAT_PID_H
