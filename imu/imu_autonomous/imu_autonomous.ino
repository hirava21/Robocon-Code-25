int FL_pwm = 8; int FR_pwm = 10; int RL_pwm = 6; int RR_pwm = 12;
int FL_dir = 9; int FR_dir = 11; int RL_dir = 7; int RR_dir = 13;
int motorSpeed = 50;

#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 myIMU;
float initialYaw = 0;
float adjustedYaw = 0;
float target = 45;

void setup() {
  Serial.begin(115200);
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

  float sumYaw = 0;
  int validReadings = 0;

  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable() == true) {
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

  if (validReadings > 0) {
    initialYaw = sumYaw / validReadings;
    Serial.print("Initial Yaw: ");
    Serial.println(initialYaw, 2);
  } else {
    Serial.println("Failed to get initial yaw.");
  }

  Serial.println("Setup complete, starting loop...");
}

void loop() {
  if (myIMU.dataAvailable() == true) {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();
    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw = yaw * 180.0 / PI;
    adjustedYaw = yaw - initialYaw;

    if (adjustedYaw > 180) {
      adjustedYaw -= 360;
    } else if (adjustedYaw < -180) {
      adjustedYaw += 360;
    }

    Serial.print("Adjusted Yaw: ");
    Serial.println(adjustedYaw, 2);
  }

  mapping();
}

void mapping() {
  // Forward
  if (adjustedYaw > (target-5) && adjustedYaw < (target+5)) {
    Serial.println("Forward");
    forward();
  }
  // Reverse
  else if (adjustedYaw >= target) {
    Serial.println("Reverse");
    reverse();
  }
  // Right
  else if (adjustedYaw <= target) {
    Serial.println("Right");
    right();
  }
  // Left
  else {
    Serial.println("Left");
    left();
  }
}

void forward() {
  // Forward direction for all wheels
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  Serial.println("Moving forward");
}

void reverse() {
  // Reverse direction for all wheels
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, motorSpeed);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, motorSpeed);
  Serial.println("Moving reverse");
}

void right() {
  // Right turn: Front wheels rotate in one direction, back wheels in the other
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, motorSpeed);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, motorSpeed);
  Serial.println("Turning right");
}

void left() {
  // Left turn: Front wheels rotate in one direction, back wheels in the other
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  Serial.println("Turning left");
}

void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RL_pwm, 0);
  analogWrite(RR_pwm, 0);
  Serial.println("Stopping");
}
