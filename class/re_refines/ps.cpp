#include "PS.h"

void PS::update() {
  // 1) clear last-frame one-shots
  resetPulses();

  // 2) read every incoming line
  while (Serial5.available()) {
    String cmd = Serial5.readStringUntil('\n');
    cmd.trim();

    // — D-Pad (must match ESP32 single/double mapping)
    if (cmd == "TARGET_1") dpadUpCount    = 1;  // single ↑
    if (cmd == "TARGET_2") dpadUpCount    = 2;  // double ↑

    if (cmd == "TARGET_3") dpadRightCount = 1;  // single →
    if (cmd == "TARGET_4") dpadRightCount = 2;  // double →

    if (cmd == "TARGET_5") dpadLeftCount  = 1;  // single ←
    if (cmd == "TARGET_6") dpadLeftCount  = 2;  // double ←

    if (cmd == "TARGET_7") dpadDownCount  = 1;  // single ↓
    if (cmd == "TARGET_8") dpadDownCount  = 2;  // double ↓

    // — Face buttons
    if (cmd == "BUTTON_1") buttonUp    = true;
    if (cmd == "BUTTON_2") buttonDown  = true;
    if (cmd == "BUTTON_3") buttonLeft  = true;
    if (cmd == "BUTTON_4") buttonRight = true;

    // — L1 / R1
    if (cmd == "BUTTON_L1 PRESSED")  l1 = true;
    if (cmd == "BUTTON_L1 RELEASED") l1 = false;
    if (cmd == "BUTTON_R1 PRESSED")  r1 = true;
    if (cmd == "BUTTON_R1 RELEASED") r1 = false;

    // — Triggers
    if (cmd.startsWith("THROTTLE:")) throttle = cmd.substring(9).toInt();
    if (cmd.startsWith("BRAKE:"))    brake    = cmd.substring(6).toInt();

    // — Joysticks
    if (cmd.startsWith("LEFT_JOYSTICK:"))  parseJoystick(cmd.substring(15), axisX,  axisY);
    if (cmd.startsWith("RIGHT_JOYSTICK:")) parseJoystick(cmd.substring(16), axisRX, axisRY);
  }
}

void PS::resetPulses() {
  // clear D-pad counts
  dpadUpCount = dpadRightCount = dpadLeftCount = dpadDownCount = 0;
  // clear face-button one-shots
  buttonUp = buttonDown = buttonLeft = buttonRight = false;
  l1 = r1 = false;
}

void PS::parseJoystick(const String& data, int& x, int& y) {
  int idxX = data.indexOf("X=");
  int idxY = data.indexOf("Y=");

  if (idxX >= 0 && idxY >= 0) {
    String xVal = data.substring(idxX + 2, data.indexOf(' ', idxX + 2));
    String yVal = data.substring(idxY + 2);
    x = xVal.toInt();
    y = yVal.toInt();
  }
}