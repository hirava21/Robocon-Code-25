#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

#define X_ENCODER_A 3  
#define X_ENCODER_B 2
#define Y_ENCODER_A 19
#define Y_ENCODER_B 18

// Motor Pins
int FL_pwm = 8, FR_pwm = 10, RL_pwm = 6, RR_pwm = 12;
int FL_dir = 9, FR_dir = 11, RL_dir = 7, RR_dir = 13;

int motorSpeed = 100;    // Base speed
int correctionFactor = 20; // Speed reduction for correction
const float PULSES_PER_REVOLUTION = 600.0;
const float WHEEL_DIAMETER = 10; // cm
const float CM_PER_PULSE = (PI * WHEEL_DIAMETER) / PULSES_PER_REVOLUTION;

volatile long x_encoder_count = 0;
volatile long y_encoder_count = 0;
float x_position = 0.0;
float y_position = 0.0;
float x_calibration = 1.0;
float y_calibration = 1.0;

// Movement flags
bool forwardCompleted = false;
bool rightCompleted = false;

// BNO080 Variables
BNO080 myIMU;
float initialYaw = 0;

void setup() {
  Serial.begin(115200);
  
  // Encoder Setup
  pinMode(X_ENCODER_A, INPUT_PULLUP);
  pinMode(X_ENCODER_B, INPUT_PULLUP);
  pinMode(Y_ENCODER_A, INPUT_PULLUP);
  pinMode(Y_ENCODER_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(X_ENCODER_A), x_encoder_isr, CHANGE);
  attachInterrupt(digitalPinToInterrupt(Y_ENCODER_A), y_encoder_isr, CHANGE);
  
  // Motor Setup
  pinMode(FL_dir, OUTPUT); pinMode(FL_pwm, OUTPUT);
  pinMode(FR_dir, OUTPUT); pinMode(FR_pwm, OUTPUT);
  pinMode(RR_dir, OUTPUT); pinMode(RR_pwm, OUTPUT);
  pinMode(RL_dir, OUTPUT); pinMode(RL_pwm, OUTPUT);

  // IMU Setup
  Wire.begin();
  if (!myIMU.begin(0x4A)) {
    Serial.println("BNO080 not detected. Check wiring!");
    while (1);
  }
  
  myIMU.enableRotationVector(100);
  delay(1000);
  
  // Calibrate initial yaw
  float sumYaw = 0;
  int validReadings = 0;
  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable()) {
      sumYaw += getYaw();
      validReadings++;
    }
    delay(10);
  }
  initialYaw = validReadings > 0 ? sumYaw / validReadings : 0;
  
  Serial.println("Setup Complete!");
}

void loop() {
  // Update position
  x_position = x_encoder_count * CM_PER_PULSE * x_calibration;
  y_position = y_encoder_count * CM_PER_PULSE * y_calibration;

  Serial.print("( X: "); Serial.print(x_position, 2);
  Serial.print(", Y: "); Serial.print(y_position, 2);
  Serial.println(")");

  if (!forwardCompleted) {
    if (x_position < 30) {
      moveForwardWithYawCorrection();
    } else {
      stopMotors();
      forwardCompleted = true;
      delay(1000);
    }
  }

  if (forwardCompleted && !rightCompleted) {
    if (y_position < 30) {
      moveRightWithYawCorrection();
    } else {
      stopMotors();
      rightCompleted = true;
      Serial.println("Movement Completed");
    }
  }

  delay(100);
}

// ISR for X encoder
void x_encoder_isr() {
  int a = digitalRead(X_ENCODER_A);
  int b = digitalRead(X_ENCODER_B);
  x_encoder_count += (a == b) ? 1 : -1;
}

// ISR for Y encoder
void y_encoder_isr() {
  int a = digitalRead(Y_ENCODER_A);
  int b = digitalRead(Y_ENCODER_B);
  y_encoder_count += (a == b) ? 1 : -1;
}

// Get Yaw angle from BNO080
float getYaw() {
  float qw = myIMU.getQuatReal();
  float qx = myIMU.getQuatI();
  float qy = myIMU.getQuatJ();
  float qz = myIMU.getQuatK();
  
  float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  yaw = yaw * 180.0 / PI;
  float adjustedYaw = yaw - initialYaw;
  
  if (adjustedYaw > 180) adjustedYaw -= 360;
  else if (adjustedYaw < -180) adjustedYaw += 360;
  
  return adjustedYaw;
}

// Move forward with yaw correction
void moveForwardWithYawCorrection() {
  float adjustedYaw = getYaw();
  
  if (adjustedYaw > -5 && adjustedYaw < 5) {
    forward(motorSpeed, motorSpeed);
  } 
  else if (adjustedYaw >= 5 && adjustedYaw < 10) {
    forward(motorSpeed, motorSpeed - correctionFactor);
  } 
  else if (adjustedYaw <= -5 && adjustedYaw > -10) {
    forward(motorSpeed - correctionFactor, motorSpeed);
  } 
  else if (adjustedYaw >= 10) {
    rotateClockwise();
  } 
  else if (adjustedYaw <= -10) {
    rotateAnticlockwise();
  }
}

// Move right with yaw correction
void moveRightWithYawCorrection() {
  float adjustedYaw = getYaw();

  if (adjustedYaw > -5 && adjustedYaw < 5) {
    right(motorSpeed);
  } 
  else if (adjustedYaw >= 5 && adjustedYaw < 10) {
    right(motorSpeed - correctionFactor);
  } 
  else if (adjustedYaw <= -5 && adjustedYaw > -10) {
    right(motorSpeed + correctionFactor);
  } 
  else if (adjustedYaw >= 10) {
    rotateClockwise();
  } 
  else if (adjustedYaw <= -10) {
    rotateAnticlockwise();
  }
}

// Motor Control Functions
void forward(int LS, int RS) {
  digitalWrite(FL_dir, 1); analogWrite(FL_pwm, LS);
  digitalWrite(FR_dir, 1); analogWrite(FR_pwm, RS);
  digitalWrite(RR_dir, 1); analogWrite(RR_pwm, RS);
  digitalWrite(RL_dir, 1); analogWrite(RL_pwm, LS);
}

void right(int speed) {
  digitalWrite(FL_dir, 1); analogWrite(FL_pwm, speed);
  digitalWrite(FR_dir, 0); analogWrite(FR_pwm, speed);
  digitalWrite(RR_dir, 1); analogWrite(RR_pwm, speed);
  digitalWrite(RL_dir, 0); analogWrite(RL_pwm, speed);
}

void rotateClockwise() {
  digitalWrite(FL_dir, 1); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 0); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1); analogWrite(RL_pwm, motorSpeed);
}

void rotateAnticlockwise() {
  digitalWrite(FL_dir, 0); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0); analogWrite(RL_pwm, motorSpeed);
}

void stopMotors() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
}
