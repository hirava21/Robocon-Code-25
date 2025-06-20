#include "PS.h"

PS ps;

void setup() {
  Serial.begin(115200);      // debug
  Serial5.begin(115200);     // from ESP32’s Serial2
  Serial.println("PS receiver up");
}

void loop() {
  ps.update();

  // // D-Pad (now correct single vs double)
  // if (ps.dpadUpCount == 1)    Serial.println("T1");  // TARGET_1
  // if (ps.dpadUpCount == 2)    Serial.println("T2");  // TARGET_2

  // if (ps.dpadDownCount == 1)  Serial.println("T7");  // TARGET_7
  // if (ps.dpadDownCount == 2)  Serial.println("T8");  // TARGET_8

  // if (ps.dpadLeftCount == 1)  Serial.println("T5");  // TARGET_5
  // if (ps.dpadLeftCount == 2)  Serial.println("T6");  // TARGET_6

  // if (ps.dpadRightCount == 1) Serial.println("T3");  // TARGET_3
  // if (ps.dpadRightCount == 2) Serial.println("T4");  // TARGET_4

  // // Face buttons
  // if (ps.buttonUp)    Serial.println("BUTTON UP");
  // if (ps.buttonDown)  Serial.println("BUTTON DOWN");
  // if (ps.buttonLeft)  Serial.println("BUTTON LEFT");
  // if (ps.buttonRight) Serial.println("BUTTON RIGHT");

  // // Shoulder
  // if (ps.l1) Serial.println("L1 HELD");
  // if (ps.r1) Serial.println("R1 HELD");

  // Triggers
  // Serial.printf("Throttle=%d  Brake=%d\n", ps.throttle, ps.brake);

  // // Joysticks
  // Serial.printf("LX=%d LY=%d  RX=%d RY=%d\n",
  //               ps.axisX, ps.axisY, ps.axisRX, ps.axisRY);

  // delay(10);

}
