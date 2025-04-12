#include "PID.h"
#include <Arduino.h>

PID::PID(float kp, float ki, float kd) : _kp(kp), _ki(ki), _kd(kd) {}

void PID::setOutputLimit(float limit) {
    _output_limit = limit;
}

float PID::compute(float target, float actual) {
    float error = target - actual;
    _integral += error;
    
    // Anti-windup
    if (_output_limit > 0) {
        _integral = constrain(_integral, -_output_limit/_ki, _output_limit/_ki);
    }
    
    float derivative = error - _prev_error;
    _prev_error = error;
    
    float output = _kp * error + _ki * _integral + _kd * derivative;
    
    if (_output_limit > 0) {
        output = constrain(output, -_output_limit, _output_limit);
    }
    
    return output;
}