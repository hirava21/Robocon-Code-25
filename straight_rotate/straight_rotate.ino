#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

#define M1_PWM 8    // (FL)
#define M1_DIR 9
#define M2_PWM 10   // (FR)
#define M2_DIR 11
#define M3_PWM 12   // (RL)
#define M3_DIR 13
#define M4_PWM 6    // (RR)
#define M4_DIR 7

const int encoderAPin = 2;  
const int encoderBPin = 3;   

volatile long pulseCount = 0; 
float distance = 0.0; 

const float wheelCircumference = 31.4159;     
const float distancePerPulse = wheelCircumference / 550.0; 

const float targetDistance = 100.0;   
const float targetAngle = 90.0;     
const float turnTolerance = 5.0;    

const int driveMotorSpeed = 30;       
const int turnMotorSpeed = 50;       

BNO080 myIMU;
float initialYaw = 0.0;    
float adjustedYaw = 0.0;   

enum State {
  DRIVING,  
  TURNING,  
  DONE     
};

State currentState = DRIVING;

void setMotor(int pwmPin, int dirPin, int speed, bool forward);
void stopMotors();
void clockwise();     
void anticlockwise();  
void countPulse();   

void setup() {
  Serial.begin(9600);
  while (!Serial);  

  pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M3_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT); pinMode(M4_DIR, OUTPUT);
  stopMotors();  

  pinMode(encoderAPin, INPUT_PULLUP);
  pinMode(encoderBPin, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderAPin), countPulse, RISING);

  Wire.begin();
  if (myIMU.begin(0x4A) == false) {
    Serial.println("BNO085 not detected. Check wiring or I2C address.");
    while (1);
  }
  Serial.println("BNO085 detected!");
  
  myIMU.enableRotationVector(100);
  delay(1000);

  float sumYaw = 0.0;
  int validReadings = 0;
  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable()) {
      float qw = myIMU.getQuatReal();
      float qx = myIMU.getQuatI();
      float qy = myIMU.getQuatJ();
      float qz = myIMU.getQuatK();
      float yaw = atan2(2.0 * (qw * qz + qx * qy),
                        1.0 - 2.0 * (qy * qy + qz * qz));
      yaw = yaw * 180.0 / PI;
      sumYaw += yaw;
      validReadings++;
    }
    delay(10);
  }
  if (validReadings > 0) {
    initialYaw = sumYaw / validReadings;
    Serial.print("Initial Yaw: ");
    Serial.println(initialYaw, 2);
  } else {
    Serial.println("Failed to obtain initial yaw.");
  }
  
  Serial.println("Setup complete. Beginning operations...");
}

void loop() {
  switch (currentState) {
    
    case DRIVING: {
      setMotor(M1_PWM, M1_DIR, driveMotorSpeed, true);
      setMotor(M2_PWM, M2_DIR, driveMotorSpeed, true);
      setMotor(M3_PWM, M3_DIR, driveMotorSpeed, true);
      setMotor(M4_PWM, M4_DIR, driveMotorSpeed, true);

      noInterrupts();
      long pulses = pulseCount;
      interrupts();
      distance = pulses * distancePerPulse;  

      Serial.print("Distance: ");
      Serial.print(distance, 2);
      Serial.println(" cm");

      if (distance >= targetDistance) {
        stopMotors();
        Serial.println("Target distance reached.");

        delay(1000); // Small pause before rotating

        noInterrupts();
        pulseCount = 0;
        interrupts();
        distance = 0.0;

        currentState = TURNING;
      }
      break;
    }

    case TURNING: {
      if (myIMU.dataAvailable()) {
        float qw = myIMU.getQuatReal();
        float qx = myIMU.getQuatI();
        float qy = myIMU.getQuatJ();
        float qz = myIMU.getQuatK();
        float yaw = atan2(2.0 * (qw * qz + qx * qy),
                          1.0 - 2.0 * (qy * qy + qz * qz));
        yaw = yaw * 180.0 / PI;
        adjustedYaw = yaw - initialYaw;
        
        if (adjustedYaw > 180) {
          adjustedYaw -= 360;
        } else if (adjustedYaw < -180) {
          adjustedYaw += 360;
        }
        
        Serial.print("Adjusted Yaw: ");
        Serial.println(adjustedYaw, 2);

        if (abs(adjustedYaw - targetAngle) < turnTolerance) {
          stopMotors();
          Serial.println("Target angle reached.");
          currentState = DONE;
        }

        else if (adjustedYaw < targetAngle) {
          anticlockwise();
        }

        else if (adjustedYaw > targetAngle) {
          clockwise();
        }
      }
      break;
    }

    case DONE:
      Serial.println("Task Completed.");
      stopMotors();
      while (1); // Halt program
  }

  delay(50);
}

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
  digitalWrite(M1_DIR, HIGH);
  analogWrite(M1_PWM, turnMotorSpeed);
  digitalWrite(M2_DIR, LOW);
  analogWrite(M2_PWM, turnMotorSpeed);
  digitalWrite(M3_DIR, LOW);
  analogWrite(M3_PWM, turnMotorSpeed);
  digitalWrite(M4_DIR, HIGH);
  analogWrite(M4_PWM, turnMotorSpeed);
  Serial.println("Turning clockwise...");
}

void anticlockwise() {
  digitalWrite(M1_DIR, LOW);
  analogWrite(M1_PWM, turnMotorSpeed);
  digitalWrite(M2_DIR, HIGH);
  analogWrite(M2_PWM, turnMotorSpeed);
  digitalWrite(M3_DIR, HIGH);
  analogWrite(M3_PWM, turnMotorSpeed);
  digitalWrite(M4_DIR, LOW);
  analogWrite(M4_PWM, turnMotorSpeed);
  Serial.println("Turning anticlockwise...");
}

void countPulse() {
  if (digitalRead(encoderBPin) == HIGH) {
    pulseCount++;
  } else {
    pulseCount--;
  }
}
