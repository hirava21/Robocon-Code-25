#include <ESP32Servo.h>
#include <Bluepad32.h>

// PIN CONNECTIONS
int ledPin = 2;

// Motor 1
int PWM1 = 14; // Motor 1 speed
int DIR1 = 27; // Motor 1 direction

// Motor 2
int PWM2 = 32; // Motor 2 speed
int DIR2 = 33; // Motor 2 direction

// Motor 3
int PWM3 = 25; // Motor 3 speed
int DIR3 = 26; // Motor 3 direction

// Motor 4
int PWM4 = 4;  // Motor 4 speed
int DIR4 = 5;  // Motor 4 direction

Servo xServo;
Servo yServo;

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
}

void onDisconnectedController(ControllerPtr ctl) {
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] == ctl) {
      Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
      myControllers[i] = nullptr;
      break;
    }
  }
}
void processGamepad(ControllerPtr ctl) {
  int motorSpeed = 0;

  // LEFT JOYSTICK - UP/DOWN for forward/reverse
  if (ctl->axisY() <= -25) {  // FORWARD
    motorSpeed = map(ctl->axisY(), -50, -508, 70, 255);
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, motorSpeed);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, motorSpeed);
    digitalWrite(DIR3, HIGH);
    analogWrite(PWM3, motorSpeed);
    digitalWrite(DIR4, HIGH);
    analogWrite(PWM4, motorSpeed);
    Serial.println("Motors moving FORWARD");
  } else if (ctl->axisY() >= 25) {  // REVERSE
    motorSpeed = map(ctl->axisY(), 50, 512, 70, 255);
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, motorSpeed);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, motorSpeed);
    digitalWrite(DIR3, LOW);
    analogWrite(PWM3, motorSpeed);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM4, motorSpeed);
    Serial.println("Motors moving REVERSE");
  }

  // RIGHT JOYSTICK - LEFT/RIGHT for turning
  if (ctl->axisRX() <= -25) {  // LEFT TURN
    motorSpeed = map(ctl->axisRX(), -50, -508, 70, 255);
    digitalWrite(DIR1, LOW);
    analogWrite(PWM1, motorSpeed);
    digitalWrite(DIR2, HIGH);
    analogWrite(PWM2, motorSpeed);
    digitalWrite(DIR3, LOW);
    analogWrite(PWM3, motorSpeed);
    digitalWrite(DIR4, HIGH);
    analogWrite(PWM4, motorSpeed);
    Serial.println("Motors turning LEFT");
  } else if (ctl->axisRX() >= 25) {  // RIGHT TURN
    motorSpeed = map(ctl->axisRX(), 50, 512, 70, 255);
    digitalWrite(DIR1, HIGH);
    analogWrite(PWM1, motorSpeed);
    digitalWrite(DIR2, LOW);
    analogWrite(PWM2, motorSpeed);
    digitalWrite(DIR3, HIGH);
    analogWrite(PWM3, motorSpeed);
    digitalWrite(DIR4, LOW);
    analogWrite(PWM4, motorSpeed);
    Serial.println("Motors turning RIGHT");
  }

  // STOP MOTORS if no input
  if (ctl->axisY() > -25 && ctl->axisY() < 25 && ctl->axisRX() > -25 && ctl->axisRX() < 25) {
    analogWrite(PWM1, 0);
    analogWrite(PWM2, 0);
    analogWrite(PWM3, 0);
    analogWrite(PWM4, 0);
    Serial.println("Motors STOPPED");
  }
}
void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // Motor pins
  pinMode(PWM1, OUTPUT);
  pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT);
  pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT);
  pinMode(DIR3, OUTPUT);
  pinMode(PWM4, OUTPUT);
  pinMode(DIR4, OUTPUT);

  // Servo setup
  xServo.attach(13); // Attach servo to pin 13
  yServo.attach(12); // Attach servo to pin 12

  // Bluepad32 setup
  BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
  BP32.update();

  // Process all connected controllers
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] != nullptr && myControllers[i]->isConnected()) {
      processGamepad(myControllers[i]);
    }
  }
}
