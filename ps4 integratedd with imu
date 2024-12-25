#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <ESP32Servo.h>
#include <Bluepad32.h>

// PIN CONNECTIONS
int ledPin = 2;

// Motor Pins
int PWM1 = 14, DIR1 = 27;
int PWM2 = 32, DIR2 = 33;
int PWM3 = 25, DIR3 = 26;
int PWM4 = 4, DIR4 = 5;

// Servo Pins
Servo xServo, yServo;

// BNO08x IMU
Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;
float rotationThreshold = 0.05;
float angle = 0.0;
unsigned long previousTime = 0;

// PS4 Controller
ControllerPtr myControllers[BP32_MAX_GAMEPADS];

// Callbacks for Controller Connection
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

// Initialize IMU
void initializeIMU() {
  Wire.begin(21, 22); // SDA: GPIO21, SCL: GPIO22
  if (!bno.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1);
  }
  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope data reporting");
    while (1);
  }
}

// Process IMU Data
void processIMUData() {
  if (bno.getSensorEvent(&sensorValue) && sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    float zRotation = sensorValue.un.gyroscope.z;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    angle += zRotation * deltaTime;
    previousTime = currentTime;

    Serial.print("Angle displaced: ");
    Serial.print(angle);
    Serial.print(" rad, ");
    if (abs(zRotation) > rotationThreshold) {
      if (zRotation > 0) Serial.println("Anticlockwise rotation");
      else Serial.println("Clockwise rotation");
    } else {
      Serial.println("No significant rotation detected.");
    }
  }
}

// Process PS4 Controller Data
void processGamepad(ControllerPtr ctl) {
  // PS4 X button to control LED
  if (ctl->buttons() == 0x0001) digitalWrite(ledPin, HIGH);
  else digitalWrite(ledPin, LOW);

  // Left Joystick - Motor Control
  int axisY = ctl->axisY(), axisX = ctl->axisX();
  if (axisY <= -25) { // Forward
    int speed = map(axisY, -25, -508, 70, 255);
    digitalWrite(DIR1, HIGH); analogWrite(PWM1, speed);
    digitalWrite(DIR2, HIGH); analogWrite(PWM2, speed);
    digitalWrite(DIR3, HIGH); analogWrite(PWM3, speed);
    digitalWrite(DIR4, HIGH); analogWrite(PWM4, speed);
  } else if (axisY >= 25) { // Reverse
    int speed = map(axisY, 25, 512, 70, 255);
    digitalWrite(DIR1, LOW); analogWrite(PWM1, speed);
    digitalWrite(DIR2, LOW); analogWrite(PWM2, speed);
    digitalWrite(DIR3, LOW); analogWrite(PWM3, speed);
    digitalWrite(DIR4, LOW); analogWrite(PWM4, speed);
  } else if (axisX <= -25) { // Left
    int speed = map(axisX, -25, -508, 70, 255);
    digitalWrite(DIR1, LOW); analogWrite(PWM1, speed);
    digitalWrite(DIR2, HIGH); analogWrite(PWM2, speed);
    digitalWrite(DIR3, LOW); analogWrite(PWM3, speed);
    digitalWrite(DIR4, HIGH); analogWrite(PWM4, speed);
  } else if (axisX >= 25) { // Right
    int speed = map(axisX, 25, 512, 70, 255);
    digitalWrite(DIR1, HIGH); analogWrite(PWM1, speed);
    digitalWrite(DIR2, LOW); analogWrite(PWM2, speed);
    digitalWrite(DIR3, HIGH); analogWrite(PWM3, speed);
    digitalWrite(DIR4, LOW); analogWrite(PWM4, speed);
  } else { // Stop
    analogWrite(PWM1, 0); analogWrite(PWM2, 0);
    analogWrite(PWM3, 0); analogWrite(PWM4, 0);
  }

  // Right Joystick - Servo Control
  int axisRX = ctl->axisRX(), axisRY = ctl->axisRY();
  if (axisRX) xServo.write(map(axisRX, -508, 512, 0, 180));
  if (axisRY) yServo.write(map(axisRY, -508, 512, 0, 180));
}

void setup() {
  Serial.begin(115200);
  pinMode(ledPin, OUTPUT);

  // Motor Pins Setup
  pinMode(PWM1, OUTPUT); pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT); pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT); pinMode(DIR3, OUTPUT);
  pinMode(PWM4, OUTPUT); pinMode(DIR4, OUTPUT);

  // Servo Setup
  // xServo.attach(13); yServo.attach(12);

  // Initialize Bluepad32 and IMU
  BP32.setup(&onConnectedController, &onDisconnectedController);
  initializeIMU();
}

void loop() {
  BP32.update();

  // Process all connected controllers
  for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
    if (myControllers[i] != nullptr && myControllers[i]->isConnected()) {
      processGamepad(myControllers[i]);
    }
  }

  // Process IMU data
  processIMUData();
}
