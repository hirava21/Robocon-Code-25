#ifndef PS_H
#define PS_H

#include <Arduino.h>

class PS {
public:
  void update();      // call each loop
  void resetPulses(); // clears one-shot flags

  // D-Pad single (1) / double (2) press counts
  uint8_t dpadUpCount    = 0;
  uint8_t dpadRightCount = 0;
  uint8_t dpadLeftCount  = 0;
  uint8_t dpadDownCount  = 0;

  // Face buttons (momentary)
  bool buttonUp    = false;
  bool buttonDown  = false;
  bool buttonLeft  = false;
  bool buttonRight = false;

  // Shoulder buttons
  bool l1 = false;
  bool r1 = false;

  // Triggers (analog)
  int throttle = 0;  // L2
  int brake    = 0;  // R2

  // Joysticks
  int axisX  = 0;
  int axisY  = 0;
  int axisRX = 0;
  int axisRY = 0;

private:
  void parseJoystick(const String& data, int& x, int& y);
};

#endif  // PS_H
