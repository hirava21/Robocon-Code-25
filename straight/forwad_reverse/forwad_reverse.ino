#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 myIMU;
float initialYaw = 0, adjustedYaw = 0;
int BS = 50;  // Base speed
int correctionFactor = 20; // Speed adjustment for corrections
int FL_pwm = 8, FR_pwm = 10, RL_pwm = 6, RR_pwm = 12;
int FL_dir = 9, FR_dir = 11, RL_dir = 7, RR_dir = 13;
bool isMovingForward = false; // Set to false for reverse movement

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting setup...");

  Wire.begin();
  if (!myIMU.begin(0x4A)) {
    Serial.println("BNO085 not detected. Check wiring.");
    while (1);
  }

  Serial.println("BNO085 detected!");
  myIMU.enableRotationVector(100);
  delay(1000);

  float sumYaw = 0;
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

  if (validReadings > 0) {
    initialYaw = sumYaw / validReadings;
    Serial.print("Initial Yaw: ");
    Serial.println(initialYaw, 2);
  } else {
    Serial.println("Failed to get initial yaw.");
  }

  Serial.println("Setup complete, starting loop...");

  pinMode(FR_pwm, OUTPUT); pinMode(FR_dir, OUTPUT);
  pinMode(RR_pwm, OUTPUT); pinMode(RR_dir, OUTPUT);
  pinMode(RL_pwm, OUTPUT); pinMode(RL_dir, OUTPUT);
  pinMode(FL_pwm, OUTPUT); pinMode(FL_dir, OUTPUT);
}

void loop() {
  if (myIMU.dataAvailable()) {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();
    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw = yaw * 180.0 / PI;
    adjustedYaw = yaw - initialYaw;

    if (adjustedYaw > 180) adjustedYaw -= 360;
    else if (adjustedYaw < -180) adjustedYaw += 360;

    Serial.print("Adjusted Yaw: ");
    Serial.println(adjustedYaw, 2);

    // Movement logic integrated here
    if (adjustedYaw > -5 && adjustedYaw < 5) {
      Serial.println(isMovingForward ? "Moving Forward" : "Moving Backward");
      if (isMovingForward) forward(BS, BS);
      else reverse(BS, BS);
    } 
    else if (adjustedYaw >= 5 && adjustedYaw < 10) {
      Serial.println(isMovingForward ? "Correcting Right (Slow down Right)" : "Correcting Left (Slow down Left)");
      if (isMovingForward) forward(BS, BS - correctionFactor);
      else reverse(BS, BS - correctionFactor);
    } 
    else if (adjustedYaw <= -5 && adjustedYaw > -10) {
      Serial.println(isMovingForward ? "Correcting Left (Slow down Left)" : "Correcting Right (Slow down Right)");
      if (isMovingForward) forward(BS - correctionFactor, BS);
      else reverse(BS - correctionFactor, BS);
    } 
    else if (adjustedYaw >= 10) {
      Serial.println(isMovingForward ? "Turning Clockwise" : "Turning Clockwise in Reverse");
      clockwise();
    } 
    else if (adjustedYaw <= -10) {
      Serial.println(isMovingForward ? "Turning Anticlockwise" : "Turning Anticlockwise in Reverse");
      anticlockwise();
    }
  }
}

// Moves the robot forward
void forward(int LS, int RS) {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, LS);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, RS);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, RS);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, LS);
}

// Moves the robot backward
void reverse(int LS, int RS) {
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, LS);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, RS);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, RS);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, LS);
}

// Rotate clockwise
void clockwise() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, BS);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, BS);
  digitalWrite(RR_dir, 0);
  analogWrite(RR_pwm, BS);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, BS);
  Serial.println("Turning Clockwise");
}

// Rotate anticlockwise
void anticlockwise() {
  digitalWrite(FL_dir, 0);
  analogWrite(FL_pwm, BS);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, BS);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, BS);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, BS);
  Serial.println("Turning Anticlockwise");
}

// Stop the robot
void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
  Serial.println("Stop");
}
