#ifndef BLDC_H
#define BLDC_H

#include <Arduino.h>

class BLDC {
  private:
    int escPin;
    void writePulse(int pulseWidth);

  public:
    BLDC(int pin);
    void setSpeed(int pulseWidth);
    void up();
    void down();
};

#endif
