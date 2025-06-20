#include "locking.h"
#include "ps.h"
#include <Wire.h>

Locking locking;
PS ps;

const float TARGET_ANGLE = 90; // Your desired angle
float accumulatedTargetAngle = 0;
bool prevButtonRightState = false;


void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  Wire1.begin();
  locking.begin();
}

void loop() {
  ps.update();       // Update controller input
  locking.update();  // Update locking system

  // Edge detection: only react to the button **press**, not hold
  if (ps.buttonRight && !prevButtonRightState) {
    accumulatedTargetAngle += 180;
    locking.target(accumulatedTargetAngle);
    Serial.print("New Target Angle Set: ");
    Serial.println(accumulatedTargetAngle);
  }
  prevButtonRightState = ps.buttonRight;
}
