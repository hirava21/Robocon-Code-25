int FL_pwm = 8; int FR_pwm = 10; int RL_pwm = 6; int RR_pwm = 12;
int FL_dir = 9; int FR_dir = 11; int RL_dir = 7; int RR_dir = 13;
int motorSpeed = 0;

#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 myIMU;
float initialYaw = 0;
float adjustedYaw = 0;

void setup() {
  Serial.begin(9600);
  while (!Serial);
  Serial.println("Starting setup...");
  Wire.begin();

  if (myIMU.begin(0x4A) == false) {
    Serial.println("BNO085 not detected. Check your wiring or I2C address.");
    while (1);
  }

  Serial.println("BNO085 detected!");
  myIMU.enableRotationVector(100);
  delay(1000);

  // Capture the initial yaw as reference (zero point)
  while (!myIMU.dataAvailable());
  initialYaw = getYaw();
  Serial.print("Initial Yaw (Set as Zero): ");
  Serial.println(initialYaw, 2);
}

void loop() {
  rotateToAngle(120);  // Example: Rotate to 90 degrees
  delay(2000);
}

float getYaw() {
  if (myIMU.dataAvailable()) {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();
    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw = yaw * 180.0 / PI;

    // Convert to 0-360 degrees
    adjustedYaw = yaw - initialYaw;
    if (adjustedYaw < 0) adjustedYaw += 360;

    return adjustedYaw;
  }
  return adjustedYaw;
}

void rotateToAngle(float targetAngle) {
  Serial.print("Rotating to angle: ");
  Serial.println(targetAngle);

  while (true) {
    float currentYaw = getYaw();
    float error = targetAngle - currentYaw;

    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    if (abs(error) <= 10) {
      stop();
      Serial.println("Target angle reached!");
      break;
    }

    if (error > 0) {
      clockwise();
    } else {
      anticlockwise();
    }
  }

  // Move Forward After Reaching Target Angle
  moveForward();
  delay(2000);  // Move forward for 2 seconds
  stop();
  delay(5000);
  // Reset Yaw
  initialYaw = getYaw();
  Serial.println("Yaw Reset to Zero.");
}

void moveForward() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("Moving Forward");
}

void clockwise() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("clockwise");
}

void anticlockwise() {
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("anticlockwise");
}

void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
  Serial.println("stop");
}
