#ifndef PID_H
#define PID_H

class PID {
private:
    float Kp, Ki, Kd;
    float prevError, integral;
    float minOutput, maxOutput;

public:
    PID(float kp, float ki, float kd, float minOut = -180, float maxOut = 180);
    float compute(float target, float actual);
};

#endif
