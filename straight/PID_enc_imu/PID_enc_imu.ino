#include <Arduino.h>
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

#define DEBUG true

#define M1_PWM 4  // FL
#define M1_DIR 22
#define M2_PWM 5  // FR
#define M2_DIR 24
#define M3_PWM 6  // RL
#define M3_DIR 26
#define M4_PWM 7  // RR
#define M4_DIR 28


const int encoderXPinA = 19;
const int encoderXPinB = 18;
const int encoderYPinA = 3;
const int encoderYPinB = 2;


volatile long pulseCountX = 0;
volatile long pulseCountY = 0;
float distanceX = 0.0;
float distanceY = 0.0;


const float wheelCircumference = 18.2212;
const float distancePerPulse = wheelCircumference / 535.0;
const int motorSpeed = 150;  


float Kp_pos = 1.0;  
float Ki_pos = 0.0;  
float Kd_pos = 0.0;  


float Kp_orient = 0.6;  
float Ki_orient = 0.3;  
float Kd_orient = 0.3;  


const float yawDeadband = 1.0;


BNO080 myIMU;
float initialYaw = 0.0;  

float currentYaw = 0.0;
float targetYaw = 0.0;   

int currentTarget = 0;
struct Target {
  float x;
  float y;
};

Target targets[] = {
  {0, 500},
  {0, -500},
  {0, 600},
  {0, -600},
  {0, 500},
  {0, -500},
  {0, 400},
  {0, -400}
};

const int totalTargets = sizeof(targets) / sizeof(targets[0]);

// PID state variables for position
float errorSumPos = 0.0;
float lastErrorPos = 0.0;
unsigned long lastTimePos = 0;

// PID state variables for orientation
float errorSumOrient = 0.0;
float lastErrorOrient = 0.0;
unsigned long lastTimeOrient = 0;

// State machine

enum MovementState { IDLE, MOVING_X, MOVING_Y };
MovementState movementState = IDLE;
Target currentTargetObj;
float baselineY = 0.0;
float baselineX = 0.0;
unsigned long lastUpdateTime = 0;
const unsigned long updateInterval = 10;  


void setMotor(int pwmPin, int dirPin, int speed, bool forward);
void stopMotors();
void countPulseX();
void countPulseY();
float calculatePID(float error, float &errorSum, float &lastError, unsigned long &lastTime, float Kp, float Ki, float Kd);
void updateMovement();
float getYawCorrection();
float normalizeAngle(float angle);

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);


  pinMode(M1_PWM, OUTPUT); pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT); pinMode(M2_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT); pinMode(M3_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT); pinMode(M4_DIR, OUTPUT);
  stopMotors();


  pinMode(encoderXPinA, INPUT_PULLUP);
  pinMode(encoderXPinB, INPUT_PULLUP);
  pinMode(encoderYPinA, INPUT_PULLUP);
  pinMode(encoderYPinB, INPUT_PULLUP);

  attachInterrupt(digitalPinToInterrupt(encoderXPinA), countPulseX, RISING);
  attachInterrupt(digitalPinToInterrupt(encoderYPinA), countPulseY, RISING);


  Wire.begin();
  if (myIMU.begin(0x4A) == false) {
    Serial.println("BNO080 not detected. Check your wiring or I2C address.");
    while (1);
  }
  
  Serial.println("BNO080 detected!");
  myIMU.enableRotationVector(100);
  delay(1000);

  // calibrate initial yaw by averaging several readings
  float sumYaw = 0.0;
  int validReadings = 0;
  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable()) {
      float qw = myIMU.getQuatReal();
      float qx = myIMU.getQuatI();
      float qy = myIMU.getQuatJ();
      float qz = myIMU.getQuatK();
      float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      yaw = yaw * 180.0 / PI;
      sumYaw += yaw;
      validReadings++;
    }
    delay(10);
  }
  if(validReadings > 0) {
    initialYaw = sumYaw / validReadings;
  }
  targetYaw = 0.0;  // We want the bot to always face 0 degrees
  Serial.println("Initial Yaw and Target Yaw set to 0.0 degrees (relative to calibration)");
  
  // initialize PID timing
  lastTimePos = millis();
  lastTimeOrient = millis();
  
  Serial.println("Setup complete!");
}

void loop() {

  if (Serial3.available()) {
    String command = Serial3.readStringUntil('\n');
    command.trim();
    if (command == "NEXT_TARGET") {
      Serial.println("Command Received: NEXT_TARGET");
      if (currentTarget < totalTargets) {

        // reset PID state variables
        errorSumPos = 0.0;
        lastErrorPos = 0.0;
        lastTimePos = millis();
        errorSumOrient = 0.0;
        lastErrorOrient = 0.0;
        lastTimeOrient = millis();


        noInterrupts();
        pulseCountX = 0;
        pulseCountY = 0;
        interrupts();
        distanceX = 0.0;
        distanceY = 0.0;


        currentTargetObj = targets[currentTarget];
        currentTarget++;
        movementState = MOVING_X;
        

        noInterrupts();
        baselineY = pulseCountY * distancePerPulse;
        interrupts();
        

        targetYaw = 0.0;
        Serial.println("Target Yaw set to 0.0 degrees");
        
        lastUpdateTime = millis();
        Serial.print("Moving to Target: ");
        Serial.print(currentTargetObj.x);
        Serial.print(", ");
        Serial.println(currentTargetObj.y);
      }
      else {
        Serial.println("All targets reached.");
      }
    }
  }


  if (myIMU.dataAvailable()) {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();
    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw = yaw * 180.0 / PI;

    // normalize and calculate current yaw relative to the calibrated initial yaw
    
    currentYaw = normalizeAngle(yaw - initialYaw);
  
    
    if (DEBUG && (movementState != IDLE) && (millis() % 500 == 0)) {
      Serial.print("Current Yaw: ");
      Serial.print(currentYaw);
      Serial.print(", Target Yaw: ");
      Serial.println(targetYaw);
    }
  }

  // update movement logic
  updateMovement();
}

void updateMovement() {
  if (movementState == IDLE) return;  // no active movement
  
  unsigned long currentTime = millis();
  if (currentTime - lastUpdateTime < updateInterval)
    return;  
  lastUpdateTime = currentTime;

  // check if we need to reset PID integral terms every 2 seconds if errors persist

  static unsigned long lastResetTime = 0;
  if (currentTime - lastResetTime > 2000) {
    if (movementState != IDLE && 
        ((movementState == MOVING_X && abs(distanceX) < 1.0) ||
         (movementState == MOVING_Y && abs(distanceY) < 1.0))) {
      errorSumPos = 0.0;
      errorSumOrient = 0.0;
      Serial.println("Resetting PID integral terms");
    }
    lastResetTime = currentTime;
  }

  // get orientation correction

  float orientationCorrection = getYawCorrection();

  if (movementState == MOVING_X) {

    // handle X-axis movement with both position and orientation correction

    noInterrupts();
    long pulsesX = pulseCountX;
    long pulsesY = pulseCountY;
    interrupts();
    distanceX = pulsesX * distancePerPulse;
    float currentY = pulsesY * distancePerPulse;
    float errorY = currentY - baselineY;
    
    // deadband for small errors in Y drift correction
    
    if (abs(errorY) < 0.5) errorY = 0;
    
    float correction_vy = -calculatePID(errorY, errorSumPos, lastErrorPos, lastTimePos, Kp_pos, Ki_pos, Kd_pos);
    float desired_vx = (currentTargetObj.x >= 0) ? motorSpeed : -motorSpeed;
    
    if (DEBUG) {
      Serial.print("MOVING_X | distX: ");
      Serial.print(distanceX);
      Serial.print(" | errorY: ");
      Serial.print(errorY);
      Serial.print(" | PID correction: ");
      Serial.print(correction_vy);
      Serial.print(" | Yaw correction: ");
      Serial.println(orientationCorrection);
    }
    
    // apply both Y drift correction and orientation correction

    float wheel1 = desired_vx + correction_vy + orientationCorrection;
    float wheel2 = desired_vx - correction_vy - orientationCorrection;
    float wheel3 = desired_vx + correction_vy + orientationCorrection;
    float wheel4 = desired_vx - correction_vy - orientationCorrection;
    
    
    wheel1 = constrain(wheel1, -255, 255);
    wheel2 = constrain(wheel2, -255, 255);
    wheel3 = constrain(wheel3, -255, 255);
    wheel4 = constrain(wheel4, -255, 255);
    
    setMotor(M1_PWM, M1_DIR, abs(wheel1), wheel1 >= 0);
    setMotor(M2_PWM, M2_DIR, abs(wheel2), wheel2 >= 0);
    setMotor(M3_PWM, M3_DIR, abs(wheel3), wheel3 >= 0);
    setMotor(M4_PWM, M4_DIR, abs(wheel4), wheel4 >= 0);

    if (abs(distanceX) >= abs(currentTargetObj.x)) {
      stopMotors();
      if (DEBUG) Serial.println("X-axis target reached. Switching to MOVING_Y phase.");
      delay(100); 
      movementState = MOVING_Y;
      noInterrupts();
      baselineX = pulseCountX * distancePerPulse;
      interrupts();

      // reset PID for the next phase

      errorSumPos = 0.0;
      lastErrorPos = 0.0;
      lastTimePos = millis();
      errorSumOrient = 0.0;
      lastErrorOrient = 0.0;
      lastTimeOrient = millis();
    }
  }
  else if (movementState == MOVING_Y) {

    // handle Y-axis movement with both position and orientation correction

    noInterrupts();
    long pulsesY = pulseCountY;
    long pulsesX = pulseCountX;
    interrupts();
    distanceY = pulsesY * distancePerPulse;
    float currentX = pulsesX * distancePerPulse;
    float errorX = currentX - baselineX;
    
    // apply deadband to small errors in X drift correction

    if (abs(errorX) < 0.5) errorX = 0;
    
    float correction_vx = -calculatePID(errorX, errorSumPos, lastErrorPos, lastTimePos, Kp_pos, Ki_pos, Kd_pos);
    float desired_vy = (currentTargetObj.y >= 0) ? motorSpeed : -motorSpeed;
    
    if (DEBUG) {
      Serial.print("MOVING_Y | distY: ");
      Serial.print(distanceY);
      Serial.print(" | errorX: ");
      Serial.print(errorX);
      Serial.print(" | PID correction: ");
      Serial.print(correction_vx);
      Serial.print(" | Yaw correction: ");
      Serial.println(orientationCorrection);
    }
    
    // apply both X drift correction and orientation correction
    float wheel1 = desired_vy + correction_vx + orientationCorrection;
    float wheel2 = desired_vy - correction_vx - orientationCorrection;
    float wheel3 = desired_vy + correction_vx + orientationCorrection;
    float wheel4 = desired_vy - correction_vx - orientationCorrection;
    

    wheel1 = constrain(wheel1, -255, 255);
    wheel2 = constrain(wheel2, -255, 255);
    wheel3 = constrain(wheel3, -255, 255);
    wheel4 = constrain(wheel4, -255, 255);
    
    setMotor(M1_PWM, M1_DIR, abs(wheel1), wheel1 >= 0);
    setMotor(M2_PWM, M2_DIR, abs(wheel2), wheel2 >= 0);
    setMotor(M3_PWM, M3_DIR, abs(wheel3), wheel3 >= 0);
    setMotor(M4_PWM, M4_DIR, abs(wheel4), wheel4 >= 0);
    
    //if target Y distance is reached
    if (abs(distanceY) >= abs(currentTargetObj.y)) {
      stopMotors();
      if (DEBUG) Serial.println("Y-axis target reached. Movement complete.");
      Serial.println("Reached Target!");
      movementState = IDLE;
      delay(100); //pause before accepting next command
    }
  }
}


float calculatePID(float error, float &errorSum, float &lastError, unsigned long &lastTime, float Kp, float Ki, float Kd) {
  unsigned long now = millis();
  float dt = (now - lastTime) / 1000.0;
  if (dt <= 0) dt = 0.001;  
  
  // accumulate integral term if error is above a minimum threshold
  if (abs(error) > 0.1 && Ki != 0) {
    errorSum += error * dt;
    const float maxErrorSum = 5.0 / (Ki > 0 ? Ki : 1.0);
    errorSum = constrain(errorSum, -maxErrorSum, maxErrorSum);
  }
  
  float dError = (error - lastError) / dt;
  if (abs(dError) < 0.5) dError = 0;
  
  float output = Kp * error + Ki * errorSum + Kd * dError;
  lastError = error;
  lastTime = now;
  
  return output;
}


float getYawCorrection() {
  // yaw error relative to target yaw (which is 0)
  float yawError = normalizeAngle(currentYaw - targetYaw);

  if (abs(yawError) < yawDeadband) {
    return 0.0;
  }
  // PID function for orientation correction
  return calculatePID(yawError, errorSumOrient, lastErrorOrient, lastTimeOrient, Kp_orient, Ki_orient, Kd_orient);
}

//-180 to 180 degrees
float normalizeAngle(float angle) {
  while (angle > 180) angle -= 360;
  while (angle < -180) angle += 360;
  return angle;
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

void countPulseX() {
  pulseCountX += (digitalRead(encoderXPinB) == HIGH) ? 1 : -1;
}

void countPulseY() {
  pulseCountY += (digitalRead(encoderYPinB) == HIGH) ? 1 : -1;
}