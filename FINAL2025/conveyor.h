#ifndef CONVEYOR_H
#define CONVEYOR_H

#include <Arduino.h>

class Conveyor {
public:
    Conveyor(uint8_t dirPin, uint8_t pwmPin);
    void updateFrom(const String& command);

private:
    uint8_t motorDirPin;
    uint8_t motorPwmPin;

    bool forwardRunning;
    bool reverseRunning;
};

#endif