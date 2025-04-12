#ifndef PS_H
#define PS_H

#include <Arduino.h>

class PS {
public:
    bool buttonRight = false;
    bool buttonLeft = false;
    bool buttonUp = false;
    bool buttonDown = false;

    void update(String command);
    void reset();
};

#endif
