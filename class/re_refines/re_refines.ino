#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include "ps.h"
#include "drive.h"

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
#define MOTOR2_PWM 22 //conveyer
#define MOTOR2_DIR 34

#define Conveyer_pwm 22
#define Conveyer_dir 34

PS ps;
Drive drive(14, 30, 15, 31, 18, 32, 19, 33);

// Control Groups
enum ControlGroup {
  GROUP_MANUAL,     // Drive, flywheel, conveyor
  GROUP_AUTO        // Execute target (autonomous tasks)
};

ControlGroup activeGroup = GROUP_MANUAL;
bool groupSwitchRequested = false;
ControlGroup requestedGroup = GROUP_MANUAL;

const int encoder1PinA = 0;
const int encoder1PinB = 1;
const int encoder2PinA = 2;
const int encoder2PinB = 3;
const int encoder3PinA = 4;
const int encoder3PinB = 5;
const int encoder4PinA = 6;
const int encoder4PinB = 7;

const int PPR = 7*18;
const float WHEEL_DIAMETER = 100.0; 
const float WHEEL_CIRCUMFERENCE = PI * WHEEL_DIAMETER;
const float CM_PER_PULSE = WHEEL_CIRCUMFERENCE / (PPR * 10.0); 
const float BASE_SPEED = 80; 
const float KP = 2.0; 

volatile long encoder1Count = 0;
volatile long encoder2Count = 0;
volatile long encoder3Count = 0;
volatile long encoder4Count = 0;

bool motorOn = false;
bool motorDir = HIGH;

Adafruit_BNO08x bno08x;
sh2_SensorValue_t sensorValue;

enum RobotState {
  IDLE,
  MOVING_X_AXIS,
  MOVING_Y_AXIS,
  MOTOR1_RUNNING,
  MOTOR1_DELAY,        
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

Target target1 = {0, 100, 150, 90, 255};
Target target2 = {0, 100, 150, 90, 255};
Target target3 = {0, 100, 150, 90, 255};
Target target4 = {0, 100, 150, 90, 255};
RobotState currentState = IDLE;
Target currentTarget;
unsigned long stateStartTime = 0;
unsigned long lastEncoderTime = 0;
float currentX = 0, currentY = 0;
float targetX = 0, targetY = 0;
float intermediateX = 0; 
bool motor1Running = false;
float targetAngle = 0;
long startEncoder1 = 0, startEncoder2 = 0, startEncoder3 = 0, startEncoder4 = 0;
float targetDistance = 0;
bool movementStarted = false;
long prevEncoder1 = 0, prevEncoder2 = 0, prevEncoder3 = 0, prevEncoder4 = 0;

int motorSpeed = 0;          // Current speed
int targetSpeed = 0;         // Desired speed
const int dirPin = 35;
const int pwmPin = 23;
const int speedStep = 10;
const int minSpeed = 0;
const int maxSpeed = 255;
const int initialSpeed = 80;

void setup() {
    Serial.begin(115200);
    Serial5.begin(115200);
    drive.begin();
    
    pinMode(Conveyer_pwm, OUTPUT);
    pinMode(Conveyer_dir, OUTPUT);

    pinMode(M1_PWM, OUTPUT);
    pinMode(M1_DIR, OUTPUT);
    pinMode(M2_PWM, OUTPUT);
    pinMode(M2_DIR, OUTPUT);
    pinMode(M3_PWM, OUTPUT);
    pinMode(M3_DIR, OUTPUT);
    pinMode(M4_PWM, OUTPUT);
    pinMode(M4_DIR, OUTPUT);

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

    Wire1.begin();
    if (!bno08x.begin_I2C(0x4A, &Wire1)) {
        Serial.println("Failed to find BNO08x chip");
    } else {
        Serial.println("BNO08x Found!");
        setReports();
    }
    
    stopAllMotors();
    Serial.println("X-Drive Robot Ready!");
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
    
    updateIMU();
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
                               ps.buttonRight || ps.buttonLeft);
    
    // Debug: Print joystick values when in AUTO mode to see what's causing the switch
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
        Serial.print(ps.buttonRight ? "Right " : "");
        Serial.print(ps.buttonLeft ? "Left " : "");
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
    analogWrite(pwmPin, 0);
    motorSpeed = 0;
    targetSpeed = 0;
    
    // Keep conveyor state as it might be intentionally running
    // You can uncomment the next two lines if you want to stop conveyor too
    // analogWrite(Conveyer_pwm, 0);
    // motorOn = false;
}

void executeManualGroup() {
    // Manual control: drive, flywheel, conveyor
    drive.move(ps.axisY, ps.axisRX, ps.brake, ps.throttle);
    conveyer();
    flywheel(ps);
}

void executeAutoGroup() {
    // Autonomous control: execute targets and state machine
    checkPSCommands();
    stateMachine();
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
void checkPSCommands() {
    if (ps.dpadUpCount) {
        Serial.println("PS: DPad Up pressed → Target1");
        executeTarget(target1);
    }
    if (ps.dpadDownCount) {
        Serial.println("PS: DPad Down pressed → Target2");
        executeTarget(target2);
    }
    if (ps.dpadRightCount) {
        Serial.println("PS: Button Left pressed → Target3");
        // executeTarget(target3);
    }
    if (ps.dpadLeftCount) {
        Serial.println("PS: Button Right pressed → Target4");
        // executeTarget(target4);
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
    
    currentState = MOVING_X_AXIS;
    stateStartTime = millis();
    Serial.print("Moving first in X axis to: X=");
    Serial.print(targetX);
    Serial.println(" cm");
}

void stateMachine() {
    switch (currentState) {
        case IDLE:
            break;
            
        case MOVING_X_AXIS:
            if (moveInXAxis(intermediateX)) {
                Serial.print("X-axis movement complete, now moving in Y axis to: Y=");
                Serial.print(targetY);
                Serial.println(" cm");
                movementStarted = false;
                currentState = MOVING_Y_AXIS;
                stateStartTime = millis();
            }
            break;
            
        case MOVING_Y_AXIS:
            if (moveInYAxis(targetY)) {
                Serial.println("Position reached, starting Motor1");
                currentState = MOTOR1_RUNNING;
                stateStartTime = millis();
                startMotor1(currentTarget.pwm1);
            }
            break;
            
        case MOTOR1_RUNNING:
            Serial.println("Motor1 started, waiting 3 seconds before turning to angle");
            currentState = MOTOR1_DELAY;
            stateStartTime = millis();
            break;
            
        case MOTOR1_DELAY:
            if (millis() - stateStartTime >= 3000) {
                Serial.println("3-second delay complete, now turning to angle while Motor1 continues");
                currentState = TURNING_TO_ANGLE;
                stateStartTime = millis();
            }
            break;
            
        case TURNING_TO_ANGLE:
            if (turnToAngle(targetAngle)) {
                Serial.println("Angle reached, starting Motor2 (Motor1 still running)");
                currentState = MOTOR2_RUNNING;
                stateStartTime = millis();
                startMotor2(currentTarget.pwm2);
            }
            break;
            
        case MOTOR2_RUNNING:
            if (millis() - stateStartTime > 5000) { 
                stopMotor2();
                Serial.println("Motor2 stopped after 5 seconds, Motor1 still running");
                Serial.println("Task complete - Motor1 will run until new target received");
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
        
        Serial.print("Starting X movement, target distance: ");
        Serial.print(targetDistance);
        Serial.println(" cm");
    }
    
    long deltaEncoder1 = encoder1Count - startEncoder1;
    long deltaEncoder2 = encoder2Count - startEncoder2;
    long deltaEncoder3 = encoder3Count - startEncoder3;
    long deltaEncoder4 = encoder4Count - startEncoder4;
    
    float distanceMoved = abs((abs(deltaEncoder1) + abs(deltaEncoder3) + abs(deltaEncoder2) + abs(deltaEncoder4)) / 4.0) * CM_PER_PULSE;
    
    Serial.print("Distance moved: ");
    Serial.print(distanceMoved);
    Serial.print(" cm, Target: ");
    Serial.print(targetDistance);
    Serial.println(" cm");
    
    if (distanceMoved >= targetDistance - 0.5) {
        stopDriveMotors();
        currentX = targetX; 
        Serial.println("X-axis target reached!");
        return true;
    }
    
    float deltaX = targetX - currentX;
    int speed = BASE_SPEED;
    if (deltaX < 0) speed = -speed;
    
    // X-axis movement
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
        
        Serial.print("Starting Y movement, target distance: ");
        Serial.print(targetDistance);
        Serial.println(" cm");
    }
    
    long deltaEncoder1 = encoder1Count - startEncoder1;
    long deltaEncoder2 = encoder2Count - startEncoder2;
    long deltaEncoder3 = encoder3Count - startEncoder3;
    long deltaEncoder4 = encoder4Count - startEncoder4;
    
    float distanceMoved = abs((deltaEncoder1 + deltaEncoder2 + deltaEncoder3 + deltaEncoder4) / 4.0) * CM_PER_PULSE;
    
    Serial.print("Distance moved: ");
    Serial.print(distanceMoved);
    Serial.print(" cm, Target: ");
    Serial.print(targetDistance);
    Serial.println(" cm");
    
    if (distanceMoved >= targetDistance - 0.5) {
        stopDriveMotors();
        currentY = targetY; 
        Serial.println("Y-axis target reached!");
        return true;
    }
    
    float deltaY = targetY - currentY;
    int speed = BASE_SPEED;
    if (deltaY < 0) speed = -speed;
    
    // Y-axis movement
    setMotorSpeed(1, speed);
    setMotorSpeed(2, speed);
    setMotorSpeed(3, speed);
    setMotorSpeed(4, speed);
    
    return false;
}

bool turnToAngle(float targetAngle) {
    float currentAngle = getCurrentAngle();
    float angleDiff = targetAngle - currentAngle;

    while (angleDiff > 180) angleDiff -= 360;
    while (angleDiff < -180) angleDiff += 360;
    
    if (abs(angleDiff) < 2.0) {
        stopDriveMotors();
        return true;
    }
    
    int turnSpeed = (int)(BASE_SPEED * 0.6);
    if (angleDiff > 0) {
        setMotorSpeed(1, -turnSpeed);
        setMotorSpeed(2, turnSpeed);
        setMotorSpeed(3, turnSpeed);
        setMotorSpeed(4, -turnSpeed);
    } else {
        setMotorSpeed(1, turnSpeed);
        setMotorSpeed(2, -turnSpeed);
        setMotorSpeed(3, -turnSpeed);
        setMotorSpeed(4, turnSpeed);
    }
    
    return false;
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

float getCurrentAngle() {
    if (bno08x.wasReset()) {
        setReports();
    }
    if (bno08x.getSensorEvent(&sensorValue)) {
        if (sensorValue.sensorId == SH2_ROTATION_VECTOR) {
            float quat_real = sensorValue.un.rotationVector.real;
            float quat_i = sensorValue.un.rotationVector.i;
            float quat_j = sensorValue.un.rotationVector.j;
            float quat_k = sensorValue.un.rotationVector.k;
            
            float yaw = atan2(2.0 * (quat_real * quat_k + quat_i * quat_j),
                             1.0 - 2.0 * (quat_j * quat_j + quat_k * quat_k));
            
            return yaw * 180.0 / PI;
        }
    }
    return 0;
}

void updateIMU() {
    if (bno08x.wasReset()) {
        setReports();
    }
}

void setReports(void) {
    if (!bno08x.enableReport(SH2_ROTATION_VECTOR)) {
        Serial.println("Could not enable rotation vector");
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

void flywheel(PS &ps) {
    static bool motorStarted = false;
    if (ps.buttonRight) {
        if (!motorStarted) {
            targetSpeed = initialSpeed;
            motorStarted = true;
        } else {
            targetSpeed = min(targetSpeed + speedStep, maxSpeed);
        }
    }
    if (ps.buttonLeft && motorStarted) {
        targetSpeed = max(targetSpeed - speedStep, minSpeed);
    }

    if (motorSpeed < targetSpeed) {
        motorSpeed++;
    } else if (motorSpeed > targetSpeed) {
        motorSpeed--;
    }

    digitalWrite(dirPin, HIGH);
    analogWrite(pwmPin, motorSpeed);
}