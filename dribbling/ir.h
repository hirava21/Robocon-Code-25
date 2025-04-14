#ifndef IR_H
#define IR_H

#include <Arduino.h>

class IR {
  private:
    int irPins[3];
    int relayPin;
    int detectionThreshold;
    int detectionCount;
    bool objectPreviouslyDetected;

  public:
    IR(int ir1, int ir2, int ir3, int relay, int threshold = 500);

    void begin();
    void update();
};

#endif
