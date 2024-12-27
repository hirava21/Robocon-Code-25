// if 360 drive cwit rx rotation control works then this will be the code uploaded in the esp for givin commands to the mega
#include <Bluepad32.h>

// Constants
const int baudRate = 9600;

// Serial communication
HardwareSerial MegaSerial(2); // Use Serial2 for communication with Mega

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
  MegaSerial.begin(baudRate, SERIAL_8N1, 16, 17); // TX=17, RX=16 (customizable pins)
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

    // Send command to Mega
    MegaSerial.println(command);
    Serial.println("Command sent to Mega: " + command);
  }
}
