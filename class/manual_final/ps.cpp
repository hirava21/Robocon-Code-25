#include "PS.h"

void PS::update() {
  while (Serial5.available()) {
    String command = Serial5.readStringUntil('\n');
    command.trim();

    resetInputs();  // Optional per loop (clears momentary inputs)

    if (command == "TARGET_1") dpadRight = true;
    else if (command == "TARGET_2") dpadLeft = true;
    else if (command == "TARGET_3") dpadUp = true;
    else if (command == "TARGET_4") dpadDown = true;

    else if (command == "BUTTON_1") buttonUp = true;
    else if (command == "BUTTON_2") buttonDown = true;
    else if (command == "BUTTON_3") buttonLeft = true;
    else if (command == "BUTTON_4") buttonRight = true;

    else if (command == "BUTTON_L1 PRESSED") l1 = true;
    else if (command == "BUTTON_L1 RELEASED") l1 = false;

    else if (command == "BUTTON_R1 PRESSED") r1 = true;
    else if (command == "BUTTON_R1 RELEASED") r1 = false;

    else if (command.startsWith("THROTTLE:")) throttle = command.substring(9).toInt();
    else if (command.startsWith("BRAKE:")) brake = command.substring(6).toInt();

    else if (command.startsWith("LEFT_JOYSTICK:")) {
      parseJoystick(command.substring(15), axisX, axisY);
    } else if (command.startsWith("RIGHT_JOYSTICK:")) {
      parseJoystick(command.substring(16), axisRX, axisRY);
    }
  }
}

void PS::parseJoystick(String data, int &x, int &y) {
  int idxX = data.indexOf("X=");
  int idxY = data.indexOf("Y=");

  if (idxX >= 0 && idxY >= 0) {
    String xVal = data.substring(idxX + 2, data.indexOf(' ', idxX + 2));
    String yVal = data.substring(idxY + 2);
    x = xVal.toInt();
    y = yVal.toInt();
  }
}

void PS::resetInputs() {
  dpadUp = dpadDown = dpadLeft = dpadRight = false;
  buttonUp = buttonDown = buttonLeft = buttonRight = false;
  l1 = r1 = false; 
}
