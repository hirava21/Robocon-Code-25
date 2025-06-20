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

#define MOTOR1_PWM 23//flywheel
#define MOTOR1_DIR 35
#define MOTOR2_PWM 22//conveyer
#define MOTOR2_DIR 34
#define Conveyer_pwm 22
#define Conveyer_dir 34
#define flywheelPwmPin 23
#define flywheelDirPin 35
#define LIMIT_SWITCH_PIN 2

// Control Groups
enum ControlGroup {
  GROUP_MANUAL,     // Drive, flywheel, conveyor
  GROUP_AUTO        // Execute target (autonomous tasks)
};
volatile bool limitSwitchTriggered = false;
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

const float ANGLE_TOLERANCE = 4.0;
const float ANGLE_SLOW_ZONE = 10.0;
const int TURN_BASE_SPEED = 50;
const int TURN_MAX_SPEED = 60;

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
volatile long encoder3Count = 0;
volatile long encoder4Count = 0;
//RELAY
const int R1 = 9;  
const int R2 = 17;
bool relay1State = false;
bool relay2State = false;
int lastButtonLeftState = 0; // To detect state changes

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
  float x;
  float y;
  int pwm1;
  float angle;
  int pwm2;
};

Target target1 = {0, 100, 230, -20, 255};
Target target2 = {0, -100, 230, 0, 255};
Target target3 = {0, 0, 0, 180, 0};
Target target4 = {0, 0, 0, 0, 0};

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
//flywheel
int pwm = 0;
bool lastR1State = false;
bool lastButtonRightState = false;
//converyer
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
  pinMode(LIMIT_SWITCH_PIN, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(LIMIT_SWITCH_PIN), limitSwitchISR, FALLING);
  
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
  
  attachInterrupt(digitalPinToInterrupt(encoder1PinA), encoder1ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder2PinA), encoder2ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder3PinA), encoder3ISR, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encoder4PinA), encoder4ISR, CHANGE);

  pinMode(R1, OUTPUT);
  pinMode(R2, OUTPUT);
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
  
  Serial.println("Robot Ready!");
  Serial.println("Active Group: MANUAL");
}

void loop() {
  ps.update();
  
  // Check for group switching requests
  checkGroupSwitchRequests();
  
  // Handle group switching
  if (groupSwitchRequested) {
    switchControlGroup();
  }
  
  // Execute based on active group
  if (activeGroup == GROUP_MANUAL) {
    executeManualGroup();
  } else if (activeGroup == GROUP_AUTO) {
    executeAutoGroup();
  }
  
  updatePosition();
}

void checkGroupSwitchRequests() {
  // Check for AUTO group requests (execute target commands)
  if (ps.dpadUpCount >= 1 || ps.dpadRightCount >= 1) {
    if (activeGroup != GROUP_AUTO) {
      requestedGroup = GROUP_AUTO;
      groupSwitchRequested = true;
      return;
    }
  }
  
  // Check for MANUAL group requests (drive, flywheel, conveyor)
  // Increased threshold to prevent false triggers from joystick noise
  bool manualControlActive = (abs(ps.axisY) > 50 || abs(ps.axisRX) > 50 || 
                             ps.brake || ps.throttle || 
                             ps.buttonUp || ps.buttonDown || 
                             ps.r1);
  
  // Debug: Print when manual control is detected during AUTO mode
  if (activeGroup == GROUP_AUTO && manualControlActive) {
    Serial.print("Manual control detected - axisY: ");
    Serial.print(ps.axisY);
    Serial.print(", axisRX: ");
    Serial.print(ps.axisRX);
    Serial.print(", brake: ");
    Serial.print(ps.brake);
    Serial.print(", throttle: ");
    Serial.print(ps.throttle);
    Serial.print(", buttons: ");
    Serial.print(ps.buttonUp ? "Up " : "");
    Serial.print(ps.buttonDown ? "Down " : "");
    Serial.print(ps.r1 ? "R1 " : "");
    Serial.println();
  }
  
  if (manualControlActive && activeGroup != GROUP_MANUAL) {
    requestedGroup = GROUP_MANUAL;
    groupSwitchRequested = true;
  }
}

void switchControlGroup() {
  if (requestedGroup != activeGroup) {
    Serial.print("Switching from ");
    Serial.print(activeGroup == GROUP_MANUAL ? "MANUAL" : "AUTO");
    Serial.print(" to ");
    Serial.println(requestedGroup == GROUP_MANUAL ? "MANUAL" : "AUTO");
    
    // Stop current group activities
    if (activeGroup == GROUP_AUTO) {
      // Stop autonomous operations
      stopAutoGroup();
    } else {
      // Stop manual operations
      stopManualGroup();
    }
    
    activeGroup = requestedGroup;
    groupSwitchRequested = false;
    
    // Small delay to ensure clean transition
    delay(100);
  }
}

void stopAutoGroup() {
  Serial.println("Stopping AUTO group operations");
  currentState = STOPPING;
  stopAllMotors();
  if (motor1Running) {
    stopMotor1();
    Serial.println("Motor1 stopped due to group switch");
  }
  currentState = IDLE;
}

void stopManualGroup() {
  Serial.println("Stopping MANUAL group operations");
  // Stop drive motors
  drive.move(0, 0, false, false);  // Stop drive system
  stopDriveMotors();
  
  // Stop flywheel
  analogWrite(flywheelPwmPin, 0);
  pwm = 0;
  
  // Keep conveyor state as it might be intentionally running
  // You can uncomment the next two lines if you want to stop conveyor too
  // analogWrite(Conveyer_pwm, 0);
  // motorOn = false;
}

void executeManualGroup() {
  // Manual control: drive, flywheel, conveyor
  drive.move(ps.axisY, ps.axisRX, ps.brake, ps.throttle);
  conveyer();
  flywheel();
}

void executeAutoGroup() {
  // Autonomous control: execute targets and state machine
  checkSerialCommands();
  stateMachine();
}

void checkSerialCommands() {
  if (ps.dpadUpCount == 1) {
    Serial.println("PS: DPad Up pressed → Target1");
    executeTarget(target1);
  } else if (ps.dpadUpCount == 2) {
    Serial.println("PS: DPad Up pressed twice → Target2");
    executeTarget(target2);
  }
  // D-pad Right
  if (ps.dpadRightCount == 1) {
    Serial.println("PS: DPad Right pressed → Target3");
    executeTarget(target3);
  } else if (ps.dpadRightCount == 2) {
    Serial.println("PS: DPad Right pressed twice → Target4");
    executeTarget(target4);
  }
}

void executeTarget(Target target) {
  Serial.println("New target received - stopping for 500ms");
  if (motor1Running) {
    stopMotor1();
    Serial.println("Motor1 stopped due to new target");
  }
  
  stopDriveMotors();  // Stop drive motors for autonomous movement
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
  startMotor1(currentTarget.pwm1);
  
  Serial.print("Target set - X: ");
  Serial.print(targetX);
  Serial.print(", Y: ");
  Serial.print(targetY);
  Serial.print(", Angle: ");
  Serial.println(targetAngle);
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

void startMotor1(int pwm) {
  digitalWrite(MOTOR1_DIR, HIGH);
  analogWrite(MOTOR1_PWM, pwm);
  motor1Running = true;
}

void stopMotor1() {
  analogWrite(MOTOR1_PWM, 0);
  motor1Running = false;
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
    Serial.print("Current Position - X: ");
    Serial.print(currentX);
    Serial.print(" cm, Y: ");
    Serial.print(currentY);
    Serial.print(" cm, Group: ");
    Serial.println(activeGroup == GROUP_MANUAL ? "MANUAL" : "AUTO");
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

void flywheel(){
  if (ps.r1 && !lastR1State) {
    digitalWrite(flywheelDirPin, HIGH);  // Set direction
    if (pwm == 0) {
      pwm = 150;
    } else {
      pwm += 10;
    }
    pwm = constrain(pwm, 0, 255);
    analogWrite(flywheelPwmPin, pwm);
    Serial.print("R1 Pressed - PWM: ");
    Serial.println(pwm);
  }
  if (ps.buttonRight && !lastButtonRightState) {
    digitalWrite(flywheelDirPin, HIGH);  // Still want forward direction
    pwm -= 20;
    pwm = constrain(pwm, 0, 255);
    analogWrite(flywheelPwmPin, pwm);
    Serial.print("ButtonRight Pressed - PWM: ");
    Serial.println(pwm);
  }

  lastR1State = ps.r1;
  lastButtonRightState = ps.buttonRight;
}
void relay(){
  currentButtonLeftState = ps.buttonLeft;

  // Check if button state has changed from last time
  if (currentButtonLeftState != lastButtonLeftState) {
    if (currentButtonLeftState == 1) {
      // Toggle relay 1
      relay1State = !relay1State;
      digitalWrite(R1, relay1State ? HIGH : LOW);
    } else if (currentButtonLeftState == 2) {
      // Toggle relay 2
      relay2State = !relay2State;
      digitalWrite(R2, relay2State ? HIGH : LOW);
    }
    lastButtonLeftState = currentButtonLeftState;
  }
  if (currentButtonLeftState == 0) {
    lastButtonLeftState = 0;
  }
}
void conveyer() {
  static bool lastButtonUp = false;
  static bool lastButtonDown = false;
  static bool isReversing = false;
  static unsigned long reverseStartTime = 0;

  bool upPressed = ps.buttonUp && !lastButtonUp;
  bool downPressed = ps.buttonDown && !lastButtonDown;

  lastButtonUp = ps.buttonUp;
  lastButtonDown = ps.buttonDown;

  // Start motor on button press
  if ((upPressed || downPressed) && !isReversing) {
    motorDir = upPressed ? HIGH : LOW;
    motorOn = !(motorOn && ((motorDir == HIGH && upPressed) || (motorDir == LOW && downPressed)));
  }
  if (limitSwitchTriggered && motorOn && !isReversing) {
    // Start reversing
    motorDir = !motorDir; // reverse direction
    isReversing = true;
    reverseStartTime = millis();
    limitSwitchTriggered = false;
  }
  if (isReversing) {
    if (millis() - reverseStartTime <= 500) {
      analogWrite(Conveyer_pwm, 255);
      digitalWrite(Conveyer_dir, motorDir);
    } else {
      analogWrite(Conveyer_pwm, 0);
      motorOn = false;
      isReversing = false;
    }
  } else {
    // Normal motor control
    analogWrite(Conveyer_pwm, motorOn ? 255 : 0);
    digitalWrite(Conveyer_dir, motorDir);
  }
}
void limitSwitchISR() {
  limitSwitchTriggered = true;
}