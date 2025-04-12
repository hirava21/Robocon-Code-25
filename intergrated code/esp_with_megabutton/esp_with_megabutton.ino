// #include <ESP32Servo.h>
#include <Bluepad32.h>
// PIN CONNECTIONS
int ledPin = 2;
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

void processGamepad(ControllerPtr ctl) {
  int axisY = ctl->axisY();
  int axisRX = ctl->axisRX();
  uint8_t dpad = ctl->dpad();
  int throttle = ctl->throttle(); // Read throttle value
  int brake = ctl->brake();       // Read brake value
  int buttons = ctl->buttons();
  // Send joystick, D-pad, throttle, and brake values to Arduino Mega
  Serial2.print(axisY);
  Serial2.print(",");
  Serial2.print(axisRX);
  Serial2.print(",");
  Serial2.print(dpad);
  Serial2.print(",");
  Serial2.print(throttle);
  Serial2.print(",");
  Serial2.println(brake);
  // Debug output
  Serial.print("axisY: ");
  Serial.print(axisY);
  Serial.print(", axisRX: ");
  Serial.print(axisRX);
  Serial.print(", dpad: ");
  Serial.print(dpad);
  Serial.print(", throttle: ");
  Serial.print(throttle);
  Serial.print(", brake: ");
  Serial.print(brake);
  Serial.print(", buttons:");
  Serial.println(buttons);
}
void setup() {  
  Serial.begin(115200);
  Serial2.begin(9600, SERIAL_8N1, 16, 17); // RX=16, TX=17
  pinMode(ledPin, OUTPUT);
  // xServo.attach(13);
  // yServo.attach(12);
  BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
  BP32.update();
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] != nullptr && myControllers[i]->isConnected()) {
      processGamepad(myControllers[i]);
    }
  }
}