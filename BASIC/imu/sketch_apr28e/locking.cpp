#include "locking.h"
#include <Arduino.h>

// Motor pin definitions
#define M1_PWM 14
#define M1_DIR 30
#define M2_PWM 15
#define M2_DIR 31
#define M3_PWM 18
#define M3_DIR 32
#define M4_PWM 19
#define M4_DIR 33

Locking::Locking() {
  targetAngle = 0.0;
}

void Locking::begin() {
  // Initialize motors
  pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M3_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT); pinMode(M4_DIR, OUTPUT);
  stopMotors();

  // Initialize IMU
  imu.initializeSensor();
}

  void Locking::target(float angle) {
  // Normalize angle to [-180, 180] for consistency
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  
  targetAngle = angle;
  lockingActive = true;
}

void Locking::update() {
  imu.processGyroscopeData();

  float currentDeg = imu.getAngleInDeg();
  if (currentDeg > 180) currentDeg -= 360;
  else if (currentDeg < -180) currentDeg += 360;

  float error = targetAngle - currentDeg;
  if (error > 180) error -= 360;
  else if (error < -180) error += 360;

  Serial.print("Current Angle: ");
  Serial.print(currentDeg);
  Serial.print(" | Target Angle: ");
  Serial.print(targetAngle);
  Serial.print(" | Error: ");
  Serial.println(error);

  if (!lockingActive) {
    stopMotors();
    return;
  }

  if (abs(error) < turnTolerance) {
    stopMotors();
    Serial.println("Locked at target angle!");
    lockingActive = false;
  } 
  else if (error > 0) {
    clockwise();
    Serial.println("Turning Clockwise");
  } 
  else {
    anticlockwise();
    Serial.println("Turning Anticlockwise");
  }
}

// Motor Control Functions
void Locking::setMotor(int pwmPin, int dirPin, int speed, bool forward) {
  digitalWrite(dirPin, forward ? HIGH : LOW);
  analogWrite(pwmPin, speed);
}

void Locking::stopMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);
}

void Locking::clockwise() {
  digitalWrite(M1_DIR, HIGH);  analogWrite(M1_PWM, turnMotorSpeed);
  digitalWrite(M2_DIR, LOW);   analogWrite(M2_PWM, turnMotorSpeed);
  digitalWrite(M3_DIR, LOW);   analogWrite(M3_PWM, turnMotorSpeed);
  digitalWrite(M4_DIR, HIGH);  analogWrite(M4_PWM, turnMotorSpeed);
}

void Locking::anticlockwise() {
  digitalWrite(M1_DIR, LOW);   analogWrite(M1_PWM, turnMotorSpeed);
  digitalWrite(M2_DIR, HIGH);  analogWrite(M2_PWM, turnMotorSpeed);
  digitalWrite(M3_DIR, HIGH);  analogWrite(M3_PWM, turnMotorSpeed);
  digitalWrite(M4_DIR, LOW);   analogWrite(M4_PWM, turnMotorSpeed);
}