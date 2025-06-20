#ifndef DRIVE_H
#define DRIVE_H

#include <Arduino.h>

class Drive {
  public:
    Drive(int p1, int d1, int p2, int d2, int p3, int d3, int p4, int d4);
    void begin();
    void move(int axisY, int axisRX, int l2, int r2);

  private:
    int pwm1, dir1, pwm2, dir2, pwm3, dir3, pwm4, dir4; // motor pins
    const int max_pwm = 120; // maximum PWM value

    // direction and pwm values for each motor
    bool m1, m2, m3, m4;
    int M1, M2, M3, M4;

    int prevM1 = 0, prevM2 = 0, prevM3 = 0, prevM4 = 0;
    const int ramp_step = 5;
};

#endif