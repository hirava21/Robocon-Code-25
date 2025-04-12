#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 myIMU;

// Constants
const int BASE_SPEED = 100;
const int CORRECTION_FACTOR = 20;
const int YAW_TOLERANCE = 5;
const int UPDATE_INTERVAL = 50;  // Update IMU reading every 50ms

// Motor Pins
const int FL_pwm = 8, FR_pwm = 10, RL_pwm = 6, RR_pwm = 12;
const int FL_dir = 9, FR_dir = 11, RL_dir = 7, RR_dir = 13;

// Global Variables
float initialYaw = 0, adjustedYaw = 0;
unsigned long lastUpdateTime = 0;

// State Machine for Movement
enum MovementState {
  STRAIGHT,
  CORRECT_LEFT,
  CORRECT_RIGHT,
  TURN_CLOCKWISE,
  TURN_ANTICLOCKWISE,
  STOP
};

MovementState currentState = STOP;

void setup() {
  Serial.begin(115200);
  while (!Serial);

  Serial.println("Initializing...");
  Wire.begin();
  
  if (!myIMU.begin(0x4A)) {
    Serial.println("BNO085 not detected. Check wiring.");
    while (1);
  }

  Serial.println("BNO085 detected!");
  myIMU.enableRotationVector(100);
  delay(1000);

  // Get initial yaw
  initialYaw = calibrateYaw();
  Serial.print("Initial Yaw: ");
  Serial.println(initialYaw, 2);

  // Initialize Motor Pins
  int motorPins[] = {FL_pwm, FR_pwm, RL_pwm, RR_pwm, FL_dir, FR_dir, RL_dir, RR_dir};
  for (int pin : motorPins) {
    pinMode(pin, OUTPUT);
  }

  Serial.println("Setup complete. Starting...");
  currentState = STRAIGHT;  // Begin moving forward
}

void loop() {
  unsigned long currentTime = millis();

  // Update IMU reading at intervals
  if (currentTime - lastUpdateTime >= UPDATE_INTERVAL) {
    lastUpdateTime = currentTime;
    updateYaw();
    updateState();
    executeMovement();
  }
}

// Updates the yaw and calculates adjustedYaw
void updateYaw() {
  if (!myIMU.dataAvailable()) return;

  float qw = myIMU.getQuatReal();
  float qx = myIMU.getQuatI();
  float qy = myIMU.getQuatJ();
  float qz = myIMU.getQuatK();

  float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
  adjustedYaw = (yaw * 180.0 / PI) - initialYaw;

  // Normalize yaw to [-180, 180]
  if (adjustedYaw > 180) adjustedYaw -= 360;
  else if (adjustedYaw < -180) adjustedYaw += 360;

  Serial.print("Adjusted Yaw: ");
  Serial.println(adjustedYaw, 2);
}

// Determines the movement state based on yaw error
void updateState() {
  if (abs(adjustedYaw) < YAW_TOLERANCE) {
    currentState = STRAIGHT;
  } else if (adjustedYaw >= YAW_TOLERANCE && adjustedYaw < 10) {
    currentState = CORRECT_RIGHT;
  } else if (adjustedYaw <= -YAW_TOLERANCE && adjustedYaw > -10) {
    currentState = CORRECT_LEFT;
  } else if (adjustedYaw >= 10) {
    currentState = TURN_CLOCKWISE;
  } else if (adjustedYaw <= -10) {
    currentState = TURN_ANTICLOCKWISE;
  }
}

// Executes motor control based on current state
void executeMovement() {
  switch (currentState) {
    case STRAIGHT:
      Serial.println("Moving Straight");
      moveForward(BASE_SPEED, BASE_SPEED);
      break;
    case CORRECT_LEFT:
      Serial.println("Correcting Left (Slow down Left)");
      moveForward(BASE_SPEED - CORRECTION_FACTOR, BASE_SPEED);
      break;
    case CORRECT_RIGHT:
      Serial.println("Correcting Right (Slow down Right)");
      moveForward(BASE_SPEED, BASE_SPEED - CORRECTION_FACTOR);
      break;
    case TURN_CLOCKWISE:
      Serial.println("Turning Clockwise");
      turnClockwise();
      break;
    case TURN_ANTICLOCKWISE:
      Serial.println("Turning Anticlockwise");
      turnAnticlockwise();
      break;
    case STOP:
      Serial.println("Stopping");
      stopMotors();
      break;
  }
}

// Calibrates initial yaw using multiple readings
float calibrateYaw() {
  float sumYaw = 0;
  int validReadings = 0;

  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable()) {
      float qw = myIMU.getQuatReal();
      float qx = myIMU.getQuatI();
      float qy = myIMU.getQuatJ();
      float qz = myIMU.getQuatK();
      float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      sumYaw += yaw * 180.0 / PI;
      validReadings++;
    }
    delay(10);
  }

  return (validReadings > 0) ? sumYaw / validReadings : 0;
}

// Motor Control Functions
void moveForward(int leftSpeed, int rightSpeed) {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, leftSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, rightSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, rightSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, leftSpeed);
}

void turnClockwise() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, BASE_SPEED);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, BASE_SPEED);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, BASE_SPEED);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, BASE_SPEED);
}

void turnAnticlockwise() {
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, BASE_SPEED);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, BASE_SPEED);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, BASE_SPEED);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, BASE_SPEED);
}

void stopMotors() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
}
