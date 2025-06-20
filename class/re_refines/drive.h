#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>

class Drive {
public:
    Drive(int p1, int d1, int p2, int d2, int p3, int d3, int p4, int d4);
    void begin();
    void move(int axisY, int axisRX, int l2, int r2);

private:
    int pwm1, dir1, pwm2, dir2, pwm3, dir3, pwm4, dir4;

    const int max_pwm = 150;
    const int rampStep = 5;

    int currentM1 = 0, currentM2 = 0, currentM3 = 0, currentM4 = 0;

    int ramp(int current, int target);
};

#endif
