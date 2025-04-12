#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>

class Motor {
private:
    int dirPins[4];  // Direction pins
    int pwmPins[4];  // PWM pins

public:
    Motor(int d1, int d2, int d3, int d4, int p1, int p2, int p3, int p4);
    void drive(int dir1, int dir2, int dir3, int dir4, int pwm1, int pwm2, int pwm3, int pwm4);
};

#endif
