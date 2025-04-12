#ifndef PID_H
#define PID_H

class PID {
private:
    float _kp, _ki, _kd;
    float _integral = 0;
    float _prev_error = 0;
    float _output_limit = 0;
    
public:
    PID(float kp, float ki, float kd);
    float compute(float target, float actual);
    void setOutputLimit(float limit);  // New method
};

#endif