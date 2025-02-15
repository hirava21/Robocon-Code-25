#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

// Motor pin configuration
int FL_pwm = 8, FR_pwm = 10, RL_pwm = 6, RR_pwm = 12;
int FL_dir = 9, FR_dir = 11, RL_dir = 7, RR_dir = 13;
int motorSpeed = 50; // Adjust speed as needed

volatile unsigned int counterFL = 0, counterFR = 0, counterRL = 0, counterRR = 0;
const int pulsesPerRevolution = 1200;  // Encoder PPR
const float wheelDiameter = 10;      // Wheel diameter in cm
const float wheelCircumference = 3.14159 * wheelDiameter;

// IMU setup
BNO080 myIMU;
float initialYaw = 0;

// Variables for position tracking
float posX = 0, posY = 0;
float yaw = 0;  // Current orientation

void setup() {
  Serial.begin(115200);

  // Motor pin setup
  pinMode(FL_dir, OUTPUT);
  pinMode(FL_pwm, OUTPUT);
  pinMode(FR_dir, OUTPUT);
  pinMode(FR_pwm, OUTPUT);
  pinMode(RR_dir, OUTPUT);
  pinMode(RR_pwm, OUTPUT);
  pinMode(RL_dir, OUTPUT);
  pinMode(RL_pwm, OUTPUT);

  // Encoder interrupt setup
  attachInterrupt(digitalPinToInterrupt(50), ai0FL, RISING);
  attachInterrupt(digitalPinToInterrupt(40), ai0FR, RISING);
  attachInterrupt(digitalPinToInterrupt(4), ai0RL, RISING);
  attachInterrupt(digitalPinToInterrupt(5), ai0RR, RISING);

  // IMU initialization
  Wire.begin();
  if (myIMU.begin(0x4A) == false) {
    Serial.println("BNO080 not detected.");
    while (1);
  }
  myIMU.enableRotationVector(100);
  delay(1000);

  initialYaw = getIMUYaw();
  Serial.println("Setup Complete");
}

// Interrupt functions for counting encoder pulses
void ai0FL() { counterFL++; }
void ai0FR() { counterFR++; }
void ai0RL() { counterRL++; }
void ai0RR() { counterRR++; }

// Function to get current yaw from IMU
float getIMUYaw() {
  if (myIMU.dataAvailable()) {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();

    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw = yaw * 180.0 / PI;
    yaw -= initialYaw;
    
    if (yaw > 180) yaw -= 360;
    if (yaw < -180) yaw += 360;
    
    return yaw;
  }
  return 0.0;
}

// Function to calculate distance traveled using encoder counts
float getDistanceTraveled(volatile unsigned int &counter) {
  float rotations = (float)counter / pulsesPerRevolution;
  return rotations * wheelCircumference; // Distance in cm
}

// Function to update robot position using encoders and IMU
void updatePosition() {
  float distFL = getDistanceTraveled(counterFL);
  float distFR = getDistanceTraveled(counterFR);
  float distRL = getDistanceTraveled(counterRL);
  float distRR = getDistanceTraveled(counterRR);

  // Reset encoders
  counterFL = counterFR = counterRL = counterRR = 0;

  // Calculate forward and lateral movement
  float deltaX_local = ((distFR + distRL) - (distFL + distRR)) / 4.0;
  float deltaY_local = ((distFL + distFR) + (distRL + distRR)) / 4.0;

  yaw = getIMUYaw();
  float yawRad = yaw * PI / 180.0;

  // Convert local movement to global frame using IMU yaw correction
  float deltaX_global = deltaX_local * cos(yawRad) - deltaY_local * sin(yawRad);
  float deltaY_global = deltaX_local * sin(yawRad) + deltaY_local * cos(yawRad);

  posX += deltaX_global;
  posY += deltaY_global;

  Serial.print("Position X: ");
  Serial.print(posX);
  Serial.print(" cm, Y: ");
  Serial.print(posY);
  Serial.print(" cm, Yaw: ");
  Serial.println(yaw);
}

// Motor movement functions
void forward() {
  digitalWrite(FL_dir, 1); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1); analogWrite(RL_pwm, motorSpeed);
  Serial.println("Moving forward");
}

void reverse() {
  digitalWrite(FL_dir, 0); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 0); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0); analogWrite(RL_pwm, motorSpeed);
  Serial.println("Moving reverse");
}

void right() {
  digitalWrite(FL_dir, 1); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0); analogWrite(RL_pwm, motorSpeed);
  Serial.println("Moving right");
}

void left() {
  digitalWrite(FL_dir, 0); analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1); analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 0); analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1); analogWrite(RL_pwm, motorSpeed);
  Serial.println("Moving left");
}

void stopMotors() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
  Serial.println("Stopping");
}

// Main function to move in a square trajectory
void loop() {
  posX = 0;
  posY = 0;
  moveTo(400, 750); 
  Serial.println("Task completed.");
  while (1);

}
// Function to move the bot to a specific position
void moveTo(float targetX, float targetY) {
  // Reset encoders before movement
  counterFL = counterFR = counterRL = counterRR = 0;
  if (posY < targetY) {
    forward();
    while (posY < targetY) {
      updatePosition();
    }
    stopMotors();
    Serial.println("Reached target Y position.");
  }

  delay(1000);  // Short delay before next movement

  // Move in the X-axis (right)
  if (posX < targetX) {
    right();
    while (posX < targetX) {
      updatePosition();
    }
    stopMotors();
    Serial.println("Reached target X position.");
  }

  Serial.println("Target position reached.");
}
