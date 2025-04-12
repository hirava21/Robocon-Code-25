// this code is to control the bot for clw and aclw using the right joystick and 4 dir(F,B,R,L) using left JS. Hopefully the ps4 will get connected not sure
#include <Bluepad32.h>

// Constants
const int max_pwm = 220;
const int rot_pwm = 50;
const int deadZone = 200;

// Variables
int clkpwm, antclkpwm;
int M1, M2, M3, M4;
bool m1, m2, m3, m4;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
  bool foundEmptySlot = false;
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == nullptr) {
      Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
      ControllerProperties properties = ctl->getProperties();
      Serial.printf("Controller model: %s, VID=0x%04x, PID=0x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
      myControllers[i] = ctl;
      foundEmptySlot = true;
      break;
    }
  }
  if (!foundEmptySlot) {
    Serial.println("CALLBACK: Controller connected, but could not find an empty slot");
  }
  Serial.println("Controller connected!");
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      break;
    }
  }
  Serial.println("Controller disconnected!");
}

void setup() {
  Serial.begin(9600);
  BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
  BP32.update();

  // Process all connected controllers
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] != nullptr && myControllers[i]->isConnected()) {
      // You can add any controller-specific logic here in the future
    }
  }

  // Get joystick input
  float joystickX = myControllers[0]->axisX();  // Left joystick X-axis value
  float joystickY = myControllers[0]->axisY();  // Left joystick Y-axis value
  float joystickRX = myControllers[0]->axisRX(); // Right joystick X-axis value

  // Apply dead zone
  if (abs(joystickX) < deadZone) joystickX = 0;
  if (abs(joystickY) < deadZone) joystickY = 0;
  if (abs(joystickRX) < deadZone) joystickRX = 0;

  // Determine clockwise and counterclockwise rotation
  clkpwm = (joystickRX > 0) ? map(joystickRX, 0, 512, 0, rot_pwm) : 0;
  antclkpwm = (joystickRX < 0) ? map(joystickRX, 0, -512, 0, rot_pwm) : 0;

  // Apply new mapping logic
  int uppwm = map(joystickX, 0, -512, 0, max_pwm);
  int downpwm = map(joystickX, 0, 512, 0, max_pwm);
  int rightpwm = map(joystickY, 0, -512, 0, max_pwm);
  int leftpwm = map(joystickY, 0, 512, 0, max_pwm);

  // Calculate motor PWMs
  int m1pwm = uppwm - rightpwm + clkpwm - antclkpwm; // Motor 1 (FL)
  int m2pwm = uppwm + rightpwm - clkpwm + antclkpwm; // Motor 2 (FR)
  int m3pwm = downpwm - leftpwm + clkpwm - antclkpwm; // Motor 3 (RR)
  int m4pwm = downpwm + leftpwm - clkpwm + antclkpwm; // Motor 4 (RL)

  // Determine motor direction
  m1 = (m1pwm > 0);
  m2 = (m2pwm > 0);
  m3 = (m3pwm < 0);
  m4 = (m4pwm < 0);

  // Limit PWM values
  M1 = constrain(abs(m1pwm), 0, max_pwm);
  M2 = constrain(abs(m2pwm), 0, max_pwm);
  M3 = constrain(abs(m3pwm), 0, max_pwm);
  M4 = constrain(abs(m4pwm), 0, max_pwm);

  // Prepare and send data over serial
  String dataToSend = String(M1) + "," + String(M2) + "," + String(M3) + "," +
                      String(M4) + "," + String(m1) + "," + String(m2) + "," +
                      String(m3) + "," + String(m4);

  Serial.println("Data to send: " + dataToSend);

  delay(50); // Adjust delay as needed to avoid buffer overflow and ensure stability
}

void dumpGamepad(ControllerPtr ctl) {
  Serial.printf(
    "idx=%d, dpad: 0x%02x, buttons: 0x%04x, axis L: %4d, %4d, axis R: %4d, %4d, brake: %4d, throttle: %4d, "
    "misc: 0x%02x, gyro x:%6d y:%6d z:%6d, accel x:%6d y:%6d z:%6d\n",
    ctl->index(),        // Controller Index
    ctl->dpad(),         // D-pad
    ctl->buttons(),      // bitmask of pressed buttons
    ctl->axisX(),        // (-511 - 512) left X Axis
    ctl->axisY(),        // (-511 - 512) left Y axis
    ctl->axisRX(),       // (-511 - 512) right X axis
    ctl->axisRY(),       // (-511 - 512) right Y axis
    ctl->brake(),        // (0 - 1023): brake button
    ctl->throttle(),     // (0 - 1023): throttle (AKA gas) button
    ctl->miscButtons(),  // bitmask of pressed "misc" buttons
    ctl->gyroX(),        // Gyro X
    ctl->gyroY(),        // Gyro Y
    ctl->gyroZ(),        // Gyro Z
    ctl->accelX(),       // Accelerometer X
    ctl->accelY(),       // Accelerometer Y
    ctl->accelZ()        // Accelerometer Z
  );
}
