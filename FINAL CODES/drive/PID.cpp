#include "PID.h"

PID::PID(float kp, float ki, float kd, float minOut, float maxOut) {
    Kp = kp;
    Ki = ki;
    Kd = kd;
    minOutput = minOut;
    maxOutput = maxOut;
    prevError = 0;
    integral = 0;
}

float PID::compute(float target, float actual) {
    float error = target - actual;
    integral += error;
    float derivative = error - prevError;
    prevError = error;

    float output = (Kp * error) + (Ki * integral) + (Kd * derivative);
    
    // // Clamping output to min/max range
    if (output > maxOutput) output = maxOutput;
    if (output < minOutput) output = minOutput;

    return output;
}
