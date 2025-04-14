#ifndef PS_H
#define PS_H

#include <Arduino.h>

class PS {
public:
    // Button & target flags
    bool dPadRight = false;
    bool dPadLeft = false;
    bool dPadUp = false;
    bool dPadDown = false;

    bool buttonUp = false;
    bool buttonDown = false;
    bool buttonLeft = false;
    bool buttonRight = false;

    // Analog values
    int l2 = 0;
    int r2 = 0;

    // Joysticks
    int axisX = 0;
    int axisY = 0;
    int axisRX = 0;
    int axisRY = 0;

    // Timestamps
    unsigned long lastL2Update = 0;
    unsigned long lastR2Update = 0;

    void resetButtons();
    void updateFromSerial(HardwareSerial& serial);
    void handleTimeouts();  // New function to handle stuck values
};

#endif

