#ifndef PS_H
#define PS_H

#include <Arduino.h>

class PS {
  public:
    bool buttonUp = false;
    bool buttonDown = false;
    bool buttonLeft = false;
    bool buttonRight = false;

    bool dpadUp = false;
    bool dpadDown = false;
    bool dpadLeft = false;
    bool dpadRight = false;

    bool l1 = false;
    bool r1 = false;

    int throttle = 0; // L2
    int brake = 0;    // R2

    int axisX = 0;
    int axisY = 0;
    int axisRX = 0;
    int axisRY = 0;

    void update();  // Call this in loop()

  private:
    void resetInputs();  // Clears momentary button/dpad states
    void parseJoystick(String data, int &x, int &y);
};

#endif
