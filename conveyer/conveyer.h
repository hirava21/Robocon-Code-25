#ifndef CONVEYER_H
#define CONVEYER_H

#include <Arduino.h>


class Conveyer {
private:
    const int PWM;
    const int DIR;

public:
    Conveyer(int pwm, int dir);
    void begin();
    void up();
    void stop();
    void down();
};

#endif
