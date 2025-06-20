#include <Wire.h>
#include "ps.h"
#include "drive.h"
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include <Wire.h>

#define M1_PWM 14
#define M1_DIR 30
#define M2_PWM 15
#define M2_DIR 31
#define M3_PWM 18
#define M3_DIR 32
#define M4_PWM 19
#define M4_DIR 33

#define MOTOR1_PWM 23
#define MOTOR1_DIR 35
#define MOTOR2_PWM 22
#define MOTOR2_DIR 34
#define Conveyer_pwm 22
#define Conveyer_dir 34
#define flywheelPwmPin 23
#define flywheelDirPin 35

// Motor 1 encoder pins
#define MOTOR1_ENCODER_A 8
#define MOTOR1_ENCODER_B 9

enum ControlGroup {
  GROUP_MANUAL,
  GROUP_AUTO
};

ControlGroup activeGroup = GROUP_MANUAL;
bool groupSwitchRequested = false;
ControlGroup requestedGroup = GROUP_MANUAL;

PS ps;
Drive drive(14, 30, 15, 31, 18, 32, 19, 33);
const int encoder1PinA = 0;
const int encoder1PinB = 1;
const int encoder2PinA = 2;
const int encoder2PinB = 3;
const int encoder3PinA = 4;
const int encoder3PinB = 5;
const int encoder4PinA = 6;
const int encoder4PinB = 7;

const int PPR = 7*19;
const float WHEEL_DIAMETER = 100.0; 
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float CM_PER_PULSE = WHEEL_CIRCUMFERENCE / (PPR * 10.0); 
const float BASE_SPEED = 80; 
const float KP = 5.0; 

// Motor 1 RPM control constants
const int MOTOR1_PPR = 600;
const float MOTOR1_KP = 2.0;
const float MOTOR1_KI = 0.5;
const float MOTOR1_KD = 0.1;
const unsigned long RPM_UPDATE_INTERVAL = 50; // 20Hz update rate

const float ANGLE_TOLERANCE = 4.0;
const float ANGLE_SLOW_ZONE = 10.0;
const int TURN_BASE_SPEED = 50;
const int TURN_MAX_SPEED = 60;

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
volatile long encoder3Count = 0;
volatile long encoder4Count = 0;

// Motor 1 encoder variables
volatile long motor1EncoderCount = 0;
long prevMotor1EncoderCount = 0;
unsigned long lastRPMUpdateTime = 0;
float currentMotor1RPM = 0;
float targetMotor1RPM = 0;
int motor1PWM = 0;

// PID variables for Motor 1
float motor1ErrorSum = 0;
float motor1LastError = 0;

int currentButtonLeftState;
Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

enum RobotState {
  IDLE,
  MOTOR1_RUNNING,
  MOVING_X_AXIS,
  MOVING_Y_AXIS,
  POST_MOVEMENT_DELAY,
  TURNING_TO_ANGLE,
  MOTOR2_RUNNING,
  STOPPING
};

struct Target {
  float motor1RPM;  // Changed from pwm1 to motor1RPM
  float x;
  float y;
  float angle;
  int pwm2;
};

// Updated target format: (motor1RPM, x, y, angle, motor2PWM)
Target target1 = {1500, 0, 100, -20, 255};    // 1500 RPM, 0cm X, 100cm Y, -20°, 255 PWM
Target target2 = {1200, 0, -100, 0, 255};     // 1200 RPM, 0cm X, -100cm Y, 0°, 255 PWM
Target target3 = {0, 0, 0, 180, 0};           // 0 RPM, 0cm X, 0cm Y, 180°, 0 PWM
Target target4 = {800, 50, 25, 45, 200};      // 800 RPM, 50cm X, 25cm Y, 45°, 200 PWM

RobotState currentState = IDLE;
Target currentTarget;
unsigned long stateStartTime = 0;
unsigned long lastEncoderTime = 0;
float currentX = 0, currentY = 0;
float targetX = 0, targetY = 0;
float intermediateX = 0; 
bool motor1Running = false;
float targetAngle = 0;

float initialYaw = 0;
float currentYaw = 0;
float adjustedYaw = 0;
bool yawCalibrated = false;

long startEncoder1 = 0, startEncoder2 = 0, startEncoder3 = 0, startEncoder4 = 0;
float targetDistance = 0;
bool movementStarted = false;

long prevEncoder1 = 0, prevEncoder2 = 0, prevEncoder3 = 0, prevEncoder4 = 0;
int pwm = 0;
bool lastR1State = false;
bool lastButtonRightState = false;
bool motorOn = false;
bool motorDir = HIGH;

void setup() {
  Serial.begin(115200);
  Serial5.begin(115200);
  delay(1000);
  
  pinMode(M1_PWM, OUTPUT);
  pinMode(M1_DIR, OUTPUT);
  pinMode(M2_PWM, OUTPUT);
  pinMode(M2_DIR, OUTPUT);
  pinMode(M3_PWM, OUTPUT);
  pinMode(M3_DIR, OUTPUT);
  pinMode(M4_PWM, OUTPUT);
  pinMode(M4_DIR, OUTPUT);
  pinMode(Conveyer_pwm, OUTPUT);
  pinMode(Conveyer_dir, OUTPUT);
  
  pinMode(MOTOR1_PWM, OUTPUT);
  pinMode(MOTOR1_DIR, OUTPUT);
  pinMode(MOTOR2_PWM, OUTPUT);
  pinMode(MOTOR2_DIR, OUTPUT);
  
  pinMode(encoder1PinA, INPUT_PULLUP);
  pinMode(encoder1PinB, INPUT_PULLUP);
  pinMode(encoder2PinA, INPUT_PULLUP);
  pinMode(encoder2PinB, INPUT_PULLUP);
  pinMode(encoder3PinA, INPUT_PULLUP);
  pinMode(encoder3PinB, INPUT_PULLUP);
  pinMode(encoder4PinA, INPUT_PULLUP);
  pinMode(encoder4PinB, INPUT_PULLUP);
  
  // Motor 1 encoder setup
  pinMode(MOTOR1_ENCODER_A, INPUT_PULLUP);
  pinMode(MOTOR1_ENCODER_B, INPUT_PULLUP);
  
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3PinA), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4PinA), encoder4ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(MOTOR1_ENCODER_A), motor1EncoderISR, CHANGE);
  
  Wire1.begin();
  Wire1.setClock(100000);
  
  bool imuFound = false;
  
  if (bno08x.begin_I2C(BNO08x_I2CADDR_DEFAULT, &Wire1)) {
    imuFound = true;
  } else {
    if (bno08x.begin_I2C(0x4B, &Wire1)) {
      imuFound = true;
    }
  }
  
  if (imuFound) {
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      yawCalibrated = false;
    } else {
      stopAllMotors();
      delay(1000);
      
      calibrateInitialYaw();
      
      if (!yawCalibrated) {
        delay(2000);
        calibrateInitialYaw();
      }
    }
  } else {
    yawCalibrated = false;
  }
  
  lastRPMUpdateTime = millis();
}

void loop() {
  ps.update();
  
  checkGroupSwitchRequests();
  
  if (groupSwitchRequested) {
    switchControlGroup();
  }
  
  if (activeGroup == GROUP_MANUAL) {
    executeManualGroup();
  } else if (activeGroup == GROUP_AUTO) {
    executeAutoGroup();
  }
  
  updatePosition();
  updateMotor1RPM(); // Update Motor 1 RPM control
}

void updateMotor1RPM() {
  if (!motor1Running) return;
  
  unsigned long currentTime = millis();
  if (currentTime - lastRPMUpdateTime >= RPM_UPDATE_INTERVAL) {
    // Calculate RPM
    long encoderDelta = motor1EncoderCount - prevMotor1EncoderCount;
    float timeInterval = (currentTime - lastRPMUpdateTime) / 1000.0; // Convert to seconds
    
    currentMotor1RPM = (encoderDelta / (float)MOTOR1_PPR) * (60.0 / timeInterval);
    
    // PID control
    float error = targetMotor1RPM - currentMotor1RPM;
    motor1ErrorSum += error * timeInterval;
    float errorDerivative = (error - motor1LastError) / timeInterval;
    
    // Calculate PID output
    float pidOutput = MOTOR1_KP * error + MOTOR1_KI * motor1ErrorSum + MOTOR1_KD * errorDerivative;
    
    // Update PWM
    motor1PWM += (int)pidOutput;
    motor1PWM = constrain(motor1PWM, 0, 255);
    
    // Apply PWM to motor
    analogWrite(MOTOR1_PWM, motor1PWM);
    
    // Update variables for next iteration
    prevMotor1EncoderCount = motor1EncoderCount;
    lastRPMUpdateTime = currentTime;
    motor1LastError = error;
    
    // Optional: Print debug info
    // Serial.print("Target RPM: "); Serial.print(targetMotor1RPM);
    // Serial.print(", Current RPM: "); Serial.print(currentMotor1RPM);
    // Serial.print(", PWM: "); Serial.println(motor1PWM);
  }
}

void checkGroupSwitchRequests() {
  if (ps.dpadUpCount >= 1 || ps.dpadRightCount >= 1) {
    if (activeGroup != GROUP_AUTO) {
      requestedGroup = GROUP_AUTO;
      groupSwitchRequested = true;
      return;
    }
  }
  
  bool manualControlActive = (abs(ps.axisY) > 50 || abs(ps.axisRX) > 50 || 
                             ps.brake || ps.throttle || 
                             ps.buttonUp || ps.buttonDown || 
                             ps.r1);
  
  if (manualControlActive && activeGroup != GROUP_MANUAL) {
    requestedGroup = GROUP_MANUAL;
    groupSwitchRequested = true;
  }
}

void switchControlGroup() {
  if (requestedGroup != activeGroup) {
    if (activeGroup == GROUP_AUTO) {
      stopAutoGroup();
    } else {
      stopManualGroup();
    }
    
    activeGroup = requestedGroup;
    groupSwitchRequested = false;
    
    delay(100);
  }
}

void stopAutoGroup() {
  currentState = STOPPING;
  stopAllMotors();
  if (motor1Running) {
    stopMotor1();
  }
  currentState = IDLE;
}

void stopManualGroup() {
  drive.move(0, 0, false, false);
  stopDriveMotors();
  
  analogWrite(flywheelPwmPin, 0);
  pwm = 0;
}

void executeManualGroup() {
  drive.move(ps.axisY, ps.axisRX, ps.brake, ps.throttle);
  conveyer();
  flywheel();
}

void executeAutoGroup() {
  checkSerialCommands();
  stateMachine();
}

void checkSerialCommands() {
  if (ps.dpadUpCount == 1) {
    executeTarget(target1);
  } else if (ps.dpadUpCount == 2) {
    executeTarget(target2);
  }
  if (ps.dpadRightCount == 1) {
    executeTarget(target3);
  } else if (ps.dpadRightCount == 2) {
    executeTarget(target4);
  }
}

void executeTarget(Target target) {
  if (motor1Running) {
    stopMotor1();
  }
  
  stopDriveMotors();
  delay(500);
  
  currentTarget = target;
  targetX = target.x;
  targetY = target.y;
  intermediateX = targetX; 
  targetAngle = target.angle;
  
  movementStarted = false;
  currentX = 0;
  currentY = 0;
  
  currentState = MOTOR1_RUNNING;
  stateStartTime = millis();
  startMotor1(currentTarget.motor1RPM); // Pass RPM instead of PWM
}

void stateMachine() {
  switch (currentState) {
    case IDLE:
      break;
      
    case MOTOR1_RUNNING:
      movementStarted = false;
      currentState = MOVING_X_AXIS;
      stateStartTime = millis();
      break;
      
    case MOVING_X_AXIS:
      if (moveInXAxis(intermediateX)) {
        movementStarted = false; 
        currentState = MOVING_Y_AXIS;
        stateStartTime = millis();
      }
      break;
      
    case MOVING_Y_AXIS:
      if (moveInYAxis(targetY)) {
        currentState = POST_MOVEMENT_DELAY;
        stateStartTime = millis();
      }
      break;
      
    case POST_MOVEMENT_DELAY:
      if (millis() - stateStartTime >= 500) {
        if (yawCalibrated) {
          currentState = TURNING_TO_ANGLE;
          stateStartTime = millis();
        } else {
          currentState = MOTOR2_RUNNING;
          stateStartTime = millis();
          startMotor2(currentTarget.pwm2);
        }
      }
      break;
      
    case TURNING_TO_ANGLE:
      if (millis() - stateStartTime > 15000) {
        stopDriveMotors();
        currentState = MOTOR2_RUNNING;
        stateStartTime = millis();
        startMotor2(currentTarget.pwm2);
      } else if (turnToAngle(targetAngle)) {
        currentState = MOTOR2_RUNNING;
        stateStartTime = millis();
        startMotor2(currentTarget.pwm2);
      }
      break;
      
    case MOTOR2_RUNNING:
      if (yawCalibrated) {
        maintainAngle(targetAngle);
      }
      
      if (millis() - stateStartTime > 5000) { 
        stopMotor2();
        currentState = IDLE;
      }
      break;
      
    case STOPPING:
      stopAllMotors();
      currentState = IDLE;
      break;
  }
}

bool moveInXAxis(float targetX) {
  if (!movementStarted) {
    startEncoder1 = encoder1Count;
    startEncoder2 = encoder2Count;
    startEncoder3 = encoder3Count;
    startEncoder4 = encoder4Count;
    targetDistance = abs(targetX - currentX);
    movementStarted = true;
  }
  
  long deltaEncoder1 = encoder1Count - startEncoder1;
  long deltaEncoder2 = encoder2Count - startEncoder2;
  long deltaEncoder3 = encoder3Count - startEncoder3;
  long deltaEncoder4 = encoder4Count - startEncoder4;
  
  float distanceMoved = abs((abs(deltaEncoder1) + abs(deltaEncoder3) + abs(deltaEncoder2) + abs(deltaEncoder4)) / 4.0) * CM_PER_PULSE;
  
  if (distanceMoved >= targetDistance - 0.5) {
    stopDriveMotors();
    currentX = targetX; 
    return true;
  }
  
  float deltaX = targetX - currentX;
  float remainingDistance = targetDistance - distanceMoved;
  float rampDistance = min(targetDistance * 0.3, 10.0); 
  
  int speed = BASE_SPEED;
  if (distanceMoved < rampDistance) {
    speed = (int)(BASE_SPEED * 0.3 + (BASE_SPEED * 0.7 * distanceMoved / rampDistance));
  } else if (remainingDistance < rampDistance) {
    speed = (int)(BASE_SPEED * 0.3 + (BASE_SPEED * 0.7 * remainingDistance / rampDistance));
  }
  
  if (deltaX < 0) speed = -speed;
  
  setMotorSpeed(1, speed);
  setMotorSpeed(2, -speed);
  setMotorSpeed(3, speed);
  setMotorSpeed(4, -speed);
  
  return false;
}

bool moveInYAxis(float targetY) {
  if (!movementStarted) {
    startEncoder1 = encoder1Count;
    startEncoder2 = encoder2Count;
    startEncoder3 = encoder3Count;
    startEncoder4 = encoder4Count;
    targetDistance = abs(targetY - currentY);
    movementStarted = true;
  }
  
  long deltaEncoder1 = encoder1Count - startEncoder1;
  long deltaEncoder2 = encoder2Count - startEncoder2;
  long deltaEncoder3 = encoder3Count - startEncoder3;
  long deltaEncoder4 = encoder4Count - startEncoder4;
  
  float distanceMoved = abs((deltaEncoder1 + deltaEncoder2 + deltaEncoder3 + deltaEncoder4) / 4.0) * CM_PER_PULSE;
  
  if (distanceMoved >= targetDistance - 0.5) {
    stopDriveMotors();
    currentY = targetY; 
    return true;
  }
  
  float deltaY = targetY - currentY;
  float remainingDistance = targetDistance - distanceMoved;
  float rampDistance = min(targetDistance * 0.3, 10.0);
  
  int speed = BASE_SPEED;
  if (distanceMoved < rampDistance) {
    speed = (int)(BASE_SPEED * 0.6 + (BASE_SPEED * 0.7 * distanceMoved / rampDistance));
  } else if (remainingDistance < rampDistance) {
    speed = (int)(BASE_SPEED * 0.5 + (BASE_SPEED * 0.7 * remainingDistance / rampDistance));
  }
  
  if (deltaY < 0) speed = -speed;
  
  setMotorSpeed(1, speed);
  setMotorSpeed(2, speed);
  setMotorSpeed(3, speed);
  setMotorSpeed(4, speed);
  
  return false;
}

bool turnToAngle(float targetAngle) {
  if (!yawCalibrated) {
    return true;
  }
  
  if (readIMU()) {
    float error = targetAngle - adjustedYaw;
    
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    float absError = abs(error);
    
    if (absError <= ANGLE_TOLERANCE) {
      stopDriveMotors();
      return true;
    }
    
    int speed = TURN_BASE_SPEED;
    if (absError < ANGLE_SLOW_ZONE) {
      speed = map(absError, ANGLE_TOLERANCE, ANGLE_SLOW_ZONE, TURN_BASE_SPEED/2, TURN_BASE_SPEED);
    } else {
      speed = TURN_MAX_SPEED;
    }
    
    if (error > 0) {
      rotateCounterClockwise(speed);
    } else {
      rotateClockwise(speed);
    }
    
    return false;
  }
  
  return false; 
}

void maintainAngle(float targetAngle) {
  if (!yawCalibrated) return;
  
  if (readIMU()) {
    float error = targetAngle - adjustedYaw;
    
    if (error > 180) error -= 360;
    if (error < -180) error += 360;
    
    float absError = abs(error);
    
    if (absError <= ANGLE_TOLERANCE) {
      stopDriveMotors();
      return;
    }
    
    int speed = TURN_BASE_SPEED / 2; 
    if (absError < ANGLE_SLOW_ZONE) {
      speed = map(absError, ANGLE_TOLERANCE, ANGLE_SLOW_ZONE, TURN_BASE_SPEED/4, TURN_BASE_SPEED/2);
    }
    
    if (error > 0) {
      rotateCounterClockwise(speed);
    } else {
      rotateClockwise(speed);
    }
  }
}

void calibrateInitialYaw() {
  yawCalibrated = false;
  
  float sumYaw = 0;
  int validReadings = 0;
  
  for (int i = 0; i < 50; i++) {
    if (bno08x.wasReset()) {
      if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
        delay(100);
        continue;
      }
      delay(100);
    }
    
    if (bno08x.getSensorEvent(&sensorValue)) {
      if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
        float qr = sensorValue.un.rotationVector.real;
        float qi = sensorValue.un.rotationVector.i;
        float qj = sensorValue.un.rotationVector.j;
        float qk = sensorValue.un.rotationVector.k;
        
        float yaw = atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk));
        yaw = yaw * 180.0 / PI;
        
        sumYaw += yaw;
        validReadings++;
      }
    }
    delay(20);
  }
  
  if (validReadings > 0) {
    initialYaw = sumYaw / validReadings;
    yawCalibrated = true;
  } else {
    yawCalibrated = false;
  }
}

bool readIMU() {
  if (!yawCalibrated) return false;
  
  if (bno08x.wasReset()) {
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
      return false;
    }
    delay(50);
  }
  
  if (bno08x.getSensorEvent(&sensorValue)) {
    if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
      float qr = sensorValue.un.rotationVector.real;
      float qi = sensorValue.un.rotationVector.i;
      float qj = sensorValue.un.rotationVector.j;
      float qk = sensorValue.un.rotationVector.k;
      
      currentYaw = atan2(2.0 * (qr * qk + qi * qj), 1.0 - 2.0 * (qj * qj + qk * qk));
      currentYaw = currentYaw * 180.0 / PI;
      
      adjustedYaw = currentYaw - initialYaw;
      
      if (adjustedYaw > 180) adjustedYaw -= 360;
      if (adjustedYaw < -180) adjustedYaw += 360;
      
      return true;
    }
  }
  
  return false;
}

void rotateClockwise(int speed) {
  digitalWrite(M1_DIR, HIGH);  
  analogWrite(M1_PWM, speed);
  
  digitalWrite(M2_DIR, LOW);  
  analogWrite(M2_PWM, speed);
  
  digitalWrite(M3_DIR, LOW); 
  analogWrite(M3_PWM, speed);
  
  digitalWrite(M4_DIR, HIGH);  
  analogWrite(M4_PWM, speed);
}

void rotateCounterClockwise(int speed) {
  digitalWrite(M1_DIR, LOW);   
  analogWrite(M1_PWM, speed);
  
  digitalWrite(M2_DIR, HIGH);  
  analogWrite(M2_PWM, speed);
  
  digitalWrite(M3_DIR, HIGH);  
  analogWrite(M3_PWM, speed);
  
  digitalWrite(M4_DIR, LOW);  
  analogWrite(M4_PWM, speed);
}

void setMotorSpeed(int motor, int speed) {
  int pwmPin, dirPin;
  switch (motor) {
    case 1: pwmPin = M1_PWM; dirPin = M1_DIR; break;
    case 2: pwmPin = M2_PWM; dirPin = M2_DIR; break;
    case 3: pwmPin = M3_PWM; dirPin = M3_DIR; break;
    case 4: pwmPin = M4_PWM; dirPin = M4_DIR; break;
    default: return;
  }
  
  digitalWrite(dirPin, speed >= 0 ? HIGH : LOW);
  analogWrite(pwmPin, abs(speed));
}

void stopDriveMotors() {
  analogWrite(M1_PWM, 0);
  analogWrite(M2_PWM, 0);
  analogWrite(M3_PWM, 0);
  analogWrite(M4_PWM, 0);
}

void stopAllMotors() {
  stopDriveMotors();
  stopMotor2();
}

// Updated Motor 1 functions for RPM control
void startMotor1(float rpm) {
  targetMotor1RPM = rpm;
  if (rpm > 0) {
    digitalWrite(MOTOR1_DIR, HIGH);
    motor1PWM = 100; // Initial PWM value, will be adjusted by PID
    analogWrite(MOTOR1_PWM, motor1PWM);
    motor1Running = true;
    
    // Reset PID variables
    motor1ErrorSum = 0;
    motor1LastError = 0;
    prevMotor1EncoderCount = motor1EncoderCount;
    lastRPMUpdateTime = millis();
  } else {
    stopMotor1();
  }
}

void stopMotor1() {
  analogWrite(MOTOR1_PWM, 0);
  motor1Running = false;
  targetMotor1RPM = 0;
  motor1PWM = 0;
  motor1ErrorSum = 0;
  motor1LastError = 0;
}

void startMotor2(int pwm) {
  digitalWrite(MOTOR2_DIR, HIGH);
  analogWrite(MOTOR2_PWM, pwm);
}

void stopMotor2() {
  analogWrite(MOTOR2_PWM, 0);
}

void updatePosition() {
  static unsigned long lastPrint = 0;
  if (millis() - lastPrint > 1000) { 
    lastPrint = millis();
  }
}

void encoder1ISR() {
  if (digitalRead(encoder1PinA) == digitalRead(encoder1PinB)) {
    encoder1Count++;
  } else {
    encoder1Count--;
  }
}

void encoder2ISR() {
  if (digitalRead(encoder2PinA) == digitalRead(encoder2PinB)) {
    encoder2Count++;
  } else {
    encoder2Count--;
  }
}

void encoder3ISR() {
  if (digitalRead(encoder3PinA) == digitalRead(encoder3PinB)) {
    encoder3Count++;
  } else {
    encoder3Count--;
  }
}

void encoder4ISR() {
  if (digitalRead(encoder4PinA) == digitalRead(encoder4PinB)) {
    encoder4Count--;
  } else {
    encoder4Count++;
  }
}

// Motor 1 encoder interrupt service routine
void motor1EncoderISR() {
  if (digitalRead(MOTOR1_ENCODER_A) == digitalRead(MOTOR1_ENCODER_B)) {
    motor1EncoderCount++;
  } else {
    motor1EncoderCount--;
  }
}

void conveyer() {
  static bool lastButtonUp = false;
  static bool lastButtonDown = false;
  bool upPressed = ps.buttonUp && !lastButtonUp;
  bool downPressed = ps.buttonDown && !lastButtonDown;
  if (upPressed || downPressed) {
    motorDir = upPressed ? HIGH : LOW;
    motorOn = !(motorOn && ((motorDir == HIGH && upPressed) || (motorDir == LOW && downPressed)));
  }
  digitalWrite(Conveyer_dir, motorDir);
  analogWrite(Conveyer_pwm, motorOn ? 255 : 0);
  lastButtonUp = ps.buttonUp;
  lastButtonDown = ps.buttonDown;
}

void flywheel(){
  if (ps.r1 && !lastR1State) {
    digitalWrite(flywheelDirPin, HIGH);
    if (pwm == 0) {
      pwm = 150;
    } else {
      pwm += 10;
    }
    pwm = constrain(pwm, 0, 255);
    analogWrite(flywheelPwmPin, pwm);
  }
  if (ps.buttonRight && !lastButtonRightState) {
    digitalWrite(flywheelDirPin, HIGH);
    pwm -= 20;
    pwm = constrain(pwm, 0, 255);
    analogWrite(flywheelPwmPin, pwm);
  }

  lastR1State = ps.r1;
  lastButtonRightState = ps.buttonRight;
}