#include <Wire.h>
#include <Bluepad32.h>

const int I2C_ADDRESS = 8; // Address of the Mega

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      myControllers[i] = ctl;
      break;
    }
  }
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      myControllers[i] = nullptr;
      break;
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire.begin(); // Initialize I2C as master
  BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
  BP32.update();

  if (myControllers[0] != nullptr && myControllers[0]->isConnected()) {
    int axisX = myControllers[0]->axisX(); // Left joystick horizontal axis
    int axisY = myControllers[0]->axisY(); // Left joystick vertical axis
    int axisRX = myControllers[0]->axisRX(); // Right joystick horizontal axis (rotation)

    // Determine the movement command
    String command;
    if (abs(axisRX) > 200) { 
      command = (axisRX > 0) ? "CLOCKWISE" : "ANTICLOCKWISE";
    } else if (abs(axisY) > 200) {
      command = (axisY < 0) ? "FORWARD" : "REVERSE";
    } else if (abs(axisX) > 200) {
      command = (axisX > 0) ? "RIGHT" : "LEFT";
    } else {
      command = "STOP";
    }

    // Send command to Mega via I2C
    sendCommandToMega(command);
    Serial.println("Command sent to Mega: " + command);
  }
}

void sendCommandToMega(String command) {
  Wire.beginTransmission(I2C_ADDRESS);
  Wire.write(command.c_str());
  Wire.endTransmission();
}
