#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

// Motor pins
#define M1_PWM 4
#define M1_DIR 22
#define M2_PWM 5
#define M2_DIR 24
#define M3_PWM 6
#define M3_DIR 26
#define M4_PWM 7
#define M4_DIR 28

// Encoder pins
const int encoderXPinA = 18;
const int encoderXPinB = 19;
const int encoderYPinA = 3;
const int encoderYPinB = 2;

// IMU
Adafruit_BNO08x bno;
sh2_SensorValue_t sensorValue;

// Constants
const float wheelCircumference = 18.2212;
const float distancePerPulse = wheelCircumference / 520.0;
const int bs = 150;
const int correctionFactor = 15;

// IMU control parameters
const float rotationThreshold = 0.03;  
const float angleTolerance = 0.05;
float targetAngle = 0.0;  // We'll maintain this angle
float currentAngle = 0.0;
unsigned long previousTime = 0;

// Movement tracking
volatile long pulseCountX = 0;
volatile long pulseCountY = 0;
float distanceX = 0.0;
float distanceY = 0.0;

// Motor control
int currentLeftSpeed = 0;
int currentRightSpeed = 0;
const int stepSize = 5;
const int delayTime = 50;

void setMotors(int leftSpeed, int rightSpeed, bool forward);
void stopMotors();
void countPulseX();
void countPulseY();
void initializeSensor();
void processGyroscopeData();
void maintainOrientation(int &leftSpeed, int &rightSpeed);

void setup() {
  Serial.begin(115200);
  
  // Initialize motors
  pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M3_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT); pinMode(M4_DIR, OUTPUT);
  stopMotors();
  
  // Initialize encoders
  pinMode(encoderXPinA, INPUT_PULLUP);
  pinMode(encoderXPinB, INPUT_PULLUP);
  pinMode(encoderYPinA, INPUT_PULLUP);
  pinMode(encoderYPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderXPinA), countPulseX, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderYPinA), countPulseY, RISING);
  
  initializeSensor();
  previousTime = micros();
}

void loop() {
  // Example movement - replace with your desired coordinates
  moveToTarget(0.0, 600.0);
  while(true); // Stop after reaching target
}

void moveToTarget(float targetX, float targetY) {
  // Reset counters
  noInterrupts();
  pulseCountX = 0;
  pulseCountY = 0;
  interrupts();
  distanceX = 0.0;
  distanceY = 0.0;
  
  // Movement in X direction
  if(abs(targetX) > 0.1) {
    Serial.println("Moving in X direction");
    
    while(abs(distanceX) < abs(targetX)) {
      // Update distances
      noInterrupts();
      distanceX = pulseCountX * distancePerPulse;
      distanceY = pulseCountY * distancePerPulse;
      interrupts();
      
      // Check Y deviation
      if(abs(distanceY) > 5.0) {
        // Correct Y deviation
        int leftSpeed = bs;
        int rightSpeed = bs;
        
        if(distanceY > 0) {
          // Need to correct negative Y (move opposite)
          leftSpeed -= correctionFactor;
          rightSpeed += correctionFactor;
        } else {
          leftSpeed += correctionFactor;
          rightSpeed -= correctionFactor;
        }
        
        setMotors(leftSpeed, rightSpeed, targetX > 0);
      } else {
        // Maintain orientation while moving X
        int leftSpeed = bs;
        int rightSpeed = bs;
        maintainOrientation(leftSpeed, rightSpeed);
        setMotors(leftSpeed, rightSpeed, targetX > 0);
      }
      
      delay(50);
    }
    
    stopMotors();
    delay(500);
  }
  
  // Movement in Y direction
  if(abs(targetY) > 0.1) {
    Serial.println("Moving in Y direction");
    
    while(abs(distanceY) < abs(targetY)) {
      // Update distances
      noInterrupts();
      distanceX = pulseCountX * distancePerPulse;
      distanceY = pulseCountY * distancePerPulse;
      interrupts();
      
      // Check X deviation
      if(abs(distanceX) > 5.0) {
        // Correct X deviation
        int leftSpeed = bs;
        int rightSpeed = bs;
        
        if(distanceX > 0) {
          // Need to correct negative X (move opposite)
          leftSpeed += correctionFactor;
          rightSpeed -= correctionFactor;
        } else {
          leftSpeed -= correctionFactor;
          rightSpeed += correctionFactor;
        }
        
        setMotors(leftSpeed, rightSpeed, targetY > 0);
      } else {
        // Maintain orientation while moving Y
        int leftSpeed = bs;
        int rightSpeed = bs;
        maintainOrientation(leftSpeed, rightSpeed);
        setMotors(leftSpeed, rightSpeed, targetY > 0);
      }
      
      delay(50);
    }
    
    stopMotors();
    delay(500);
  }
}

void setMotors(int leftSpeed, int rightSpeed, bool forward) {
  // Constrain speeds to valid PWM range
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);
  
  // Gradually adjust motor speeds
  while(currentLeftSpeed != leftSpeed || currentRightSpeed != rightSpeed) {
    if(currentLeftSpeed < leftSpeed) {
      currentLeftSpeed = min(currentLeftSpeed + stepSize, leftSpeed);
    } else if(currentLeftSpeed > leftSpeed) {
      currentLeftSpeed = max(currentLeftSpeed - stepSize, leftSpeed);
    }
    
    if(currentRightSpeed < rightSpeed) {
      currentRightSpeed = min(currentRightSpeed + stepSize, rightSpeed);
    } else if(currentRightSpeed > rightSpeed) {
      currentRightSpeed = max(currentRightSpeed - stepSize, rightSpeed);
    }
    
    // Set left motors (M1 and M3)
    digitalWrite(M1_DIR, forward ? HIGH : LOW);
    digitalWrite(M3_DIR, forward ? HIGH : LOW);
    analogWrite(M1_PWM, currentLeftSpeed);
    analogWrite(M3_PWM, currentLeftSpeed);
    
    // Set right motors (M2 and M4)
    digitalWrite(M2_DIR, forward ? HIGH : LOW);
    digitalWrite(M4_DIR, forward ? HIGH : LOW);
    analogWrite(M2_PWM, currentRightSpeed);
    analogWrite(M4_PWM, currentRightSpeed);
    
    delay(delayTime);
  }
}
void stopMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);
  currentLeftSpeed = 0;
  currentRightSpeed = 0;
}

void countPulseX() {
  if (digitalRead(encoderXPinB) == HIGH) {
    pulseCountX++;
  } else {
    pulseCountX--;
  }
}

void countPulseY() {
  if (digitalRead(encoderYPinB) == HIGH) {
    pulseCountY++;
  } else {
    pulseCountY--;
  }
}

void initializeSensor() {
  Wire.begin();
  if (!bno.begin_I2C()) {
    Serial.println("Failed to find BNO08x chip");
    while (1);
  }
  Serial.println("BNO08x Found!");

  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope data reporting");
    while (1);
  }
}

void maintainOrientation(int &leftSpeed, int &rightSpeed) {
  if (!bno.getSensorEvent(&sensorValue)) {
    Serial.println("Failed to read sensor data");
    return;
  }

  if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    float zRotation = sensorValue.un.gyroscope.z;
    unsigned long currentTime = millis();
    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    if (abs(zRotation) > rotationThreshold) {
      currentAngle += zRotation * deltaTime;
    }

    if (abs(currentAngle - targetAngle) > angleTolerance) {
      if (currentAngle > targetAngle) {
        // Need to rotate clockwise (reduce right speed)
        // rightSpeed -= correctionFactor;
        clockwise();
      } else {
        // Need to rotate counter-clockwise (reduce left speed)
        // leftSpeed -= correctionFactor;
        anticlockwise();
      }
    }
  }
}
void clockwise() {
  digitalWrite(M1_DIR, 1);
  analogWrite(M1_PWM, bs);
  digitalWrite(M2_DIR, 0);
  analogWrite(M2_PWM, bs);
  digitalWrite(M3_DIR, 0);
  analogWrite(M3_PWM, bs);
  digitalWrite(M4_DIR, 1);
  analogWrite(M4_PWM, bs);
  Serial.println("Turning Clockwise");
}
void anticlockwise() {
  digitalWrite(M1_DIR, 0);
  analogWrite(M1_PWM, bs);
  digitalWrite(M2_DIR, 1);
  analogWrite(M2_PWM, bs);
  digitalWrite(M3_DIR, 1);
  analogWrite(M3_PWM, bs);
  digitalWrite(M4_DIR, 0);
  analogWrite(M4_DIR, bs);
  Serial.println("Turning Anticlockwise");
}