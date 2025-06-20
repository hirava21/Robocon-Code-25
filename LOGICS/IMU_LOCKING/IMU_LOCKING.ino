#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

// Motor pin definitions
#define M1_PWM 14
#define M1_DIR 30
#define M2_PWM 15
#define M2_DIR 31
#define M3_PWM 18
#define M3_DIR 32
#define M4_PWM 19
#define M4_DIR 33

// IMU and control variables
Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;
float angle = 0.0;
unsigned long previousTime = 0;

const float rotationThreshold = 0.05;  // rad/s
const float turnTolerance = 5.0;        // degrees
const int turnMotorSpeed = 50;
float targetAngle = 180.0; // in degrees

// Helper Functions
void setMotor(int pwmPin, int dirPin, int speed, bool forward) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, speed);
}

void stopMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);
}

void clockwise() {
  digitalWrite(M1_DIR, HIGH);  analogWrite(M1_PWM, turnMotorSpeed);
  digitalWrite(M2_DIR, LOW);   analogWrite(M2_PWM, turnMotorSpeed);
  digitalWrite(M3_DIR, LOW);   analogWrite(M3_PWM, turnMotorSpeed);
  digitalWrite(M4_DIR, HIGH);  analogWrite(M4_PWM, turnMotorSpeed);
}

void anticlockwise() {
  digitalWrite(M1_DIR, LOW);   analogWrite(M1_PWM, turnMotorSpeed);
  digitalWrite(M2_DIR, HIGH);  analogWrite(M2_PWM, turnMotorSpeed);
  digitalWrite(M3_DIR, HIGH);  analogWrite(M3_PWM, turnMotorSpeed);
  digitalWrite(M4_DIR, LOW);   analogWrite(M4_PWM, turnMotorSpeed);
}

void initializeSensor() {
  Wire1.begin();

  if (!bno.begin_I2C(0x4A, &Wire1)) {
    Serial.println("Failed to find BNO08x chip");
    while (1);
  }
  Serial.println("BNO08x Found!");

  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope data reporting");
    while (1);
  }
}

void processGyroscopeData() {
  if (!bno.getSensorEvent(&sensorValue)) {
    Serial.println("Failed to read sensor data");
    return;
  }

  if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    float zRotation = sensorValue.un.gyroscope.z; // rad/s
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    if (abs(zRotation) > rotationThreshold) {
      angle += zRotation * deltaTime;  // theta = omega * deltaTime
    }
  }
}

void setup() {
  Serial.begin(115200);
  Wire1.begin();

  // Motor setup
  pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M3_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT); pinMode(M4_DIR, OUTPUT);
  stopMotors();

  // IMU setup
  initializeSensor();
  previousTime = millis();
}

void loop() {
  processGyroscopeData();
  
  float deg = angle * 180.0 / PI;
  if (deg > 180) deg -= 360;
  else if (deg < -180) deg += 360;

  float error = targetAngle - deg;
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;

  if (abs(error) < turnTolerance) {
    stopMotors();
    Serial.println("Reached target angle.");
  } else if (error > 0) {
    clockwise();
    Serial.println("clock");
  } else {
    anticlockwise();
    Serial.println("anticlockwise");
  }

  delay(50);
}