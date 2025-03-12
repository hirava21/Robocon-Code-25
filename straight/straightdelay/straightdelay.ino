#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

// Create an instance of the BNO085 sensor
BNO080 myIMU;
int FL_pwm = 8; int FR_pwm = 10; int RL_pwm = 6; int RR_pwm =12;
int FL_dir = 9; int FR_dir = 11; int RL_dir = 7; int RR_dir =13; int motorSpeed = 100;
float targetYaw = 0;  // Desired yaw angle
float yawTolerance = 2.0;  // Allowable deviation in degrees
unsigned long lastYawUpdate = 0;
const int yawUpdateInterval = 200;  // Time between yaw corrections (ms)

void setup() {
  Serial.begin(115200);
  while (!Serial);  // Wait for Serial to be ready
  Wire.begin();

  // Initialize BNO085
  if (!myIMU.begin(0x4A)) {
    Serial.println("BNO085 not detected. Check wiring.");
    while (1);
  }
  Serial.println("BNO085 detected!");
  myIMU.enableRotationVector(100);

  pinMode(FL_dir, OUTPUT);
  pinMode(FL_pwm, OUTPUT);
  pinMode(FR_dir, OUTPUT);
  pinMode(FR_pwm, OUTPUT);
  pinMode(RR_dir, OUTPUT);
  pinMode(RR_pwm, OUTPUT);
  pinMode(RL_dir, OUTPUT);
  pinMode(RL_pwm, OUTPUT);

  delay(1000);  // Allow IMU to stabilize

  // Get initial yaw
  float sumYaw = 0;
  int validReadings = 0;
  for (int i = 0; i < 100; i++) {
    if (myIMU.dataAvailable()) {
      float qw = myIMU.getQuatReal();
      float qx = myIMU.getQuatI();
      float qy = myIMU.getQuatJ();
      float qz = myIMU.getQuatK();

      float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      yaw = yaw * 180.0 / PI;  // Convert to degrees

      sumYaw += yaw;
      validReadings++;
    }
    delay(10);
  }

  if (validReadings > 0) {
    targetYaw = sumYaw / validReadings;  // Set initial yaw as target
    Serial.print("Target Yaw: ");
    Serial.println(targetYaw, 2);
  } else {
    Serial.println("Failed to get initial yaw.");
  }
}
void forward(){
digitalWrite(FL_dir,1);
analogWrite(FL_pwm, motorSpeed);
digitalWrite(FR_dir,1);
analogWrite(FR_pwm, motorSpeed);
digitalWrite(RR_dir,1);
analogWrite(RR_pwm, motorSpeed);
digitalWrite(RL_dir,1);
analogWrite(RL_pwm, motorSpeed);
Serial.println("forward");
}
void reverse(){
digitalWrite(FL_dir,0);
analogWrite(FL_pwm, motorSpeed);
digitalWrite(FR_dir,0);
analogWrite(FR_pwm, motorSpeed);
digitalWrite(RR_dir,0);
analogWrite(RR_pwm, motorSpeed);
digitalWrite(RL_dir,0);
analogWrite(RL_pwm, motorSpeed);
Serial.println("reverse");
}
void right(){
digitalWrite(FL_dir,1);
analogWrite(FL_pwm, motorSpeed);
digitalWrite(FR_dir,0);
analogWrite(FR_pwm, motorSpeed);
digitalWrite(RR_dir,1);
analogWrite(RR_pwm, motorSpeed);
digitalWrite(RL_dir,0);
analogWrite(RL_pwm, motorSpeed);
Serial.println("right");
}
void left(){
digitalWrite(FL_dir,0);
analogWrite(FL_pwm, motorSpeed);
digitalWrite(FR_dir,1);
analogWrite(FR_pwm, motorSpeed);
digitalWrite(RR_dir,0);
analogWrite(RR_pwm, motorSpeed);
digitalWrite(RL_dir,1);
analogWrite(RL_pwm, motorSpeed);
Serial.println("left");
}
void stop(){
analogWrite(FL_pwm, 0);
analogWrite(FR_pwm, 0);
analogWrite(RR_pwm, 0);
analogWrite(RL_pwm, 0);
Serial.println("stop");
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
void loop() {
  forward();  // Keep moving forward

  // Check yaw correction at fixed intervals
  if (millis() - lastYawUpdate > yawUpdateInterval) {
    lastYawUpdate = millis();

    // Read current yaw
    if (myIMU.dataAvailable()) {
      float qw = myIMU.getQuatReal();
      float qx = myIMU.getQuatI();
      float qy = myIMU.getQuatJ();
      float qz = myIMU.getQuatK();

      float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
      yaw = yaw * 180.0 / PI;  // Convert to degrees

      float yawError = yaw - targetYaw;

      // Keep yaw within -180 to 180 degrees
      if (yawError > 180) yawError -= 360;
      if (yawError < -180) yawError += 360;

      Serial.print("Current Yaw: ");
      Serial.print(yaw, 2);
      Serial.print(" | Yaw Error: ");
      Serial.println(yawError, 2);

      // Apply correction if error exceeds tolerance
      if (yawError > yawTolerance) {
        Serial.println("Correcting: Turn Counterclockwise");
        anticlockwise();
        delay(100);  // Short correction time
      } 
      else if (yawError < -yawTolerance) {
        Serial.println("Correcting: Turn Clockwise");
        clockwise();
        delay(100);  // Short correction time
      }

      forward();  // Continue moving forward
    }
  }
}
