#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
#include <PID_v1.h>

BNO080 myIMU;
float initialYaw = 0, adjustedYaw = 0;
int BS = 100;  // Base speed
int FL_pwm = 8, FR_pwm = 10, RL_pwm = 6, RR_pwm = 12;
int FL_dir = 9, FR_dir = 11, RL_dir = 7, RR_dir = 13;
bool isMovingForward = true; // Set to false for reverse movement

// PID variables
double yawInput, yawOutput, yawSetpoint = 0;  // Setpoint = 0 means maintaining initial direction
double Kp = 1.5, Ki = 0.0, Kd = 0; // Tune these values for best performance
PID yawPID(&yawInput, &yawOutput, &yawSetpoint, Kp, Ki, Kd, DIRECT);

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

  // Initialize PID
  yawPID.SetMode(AUTOMATIC);
  yawPID.SetOutputLimits(-30, 30); // Limit speed correction range
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

    // Set PID input and compute output
    yawInput = adjustedYaw;
    yawPID.Compute();

    Serial.print("Adjusted Yaw: ");
    Serial.print(adjustedYaw, 2);
    Serial.print(" | PID Correction: ");
    Serial.println(yawOutput, 2);

    int leftSpeed = BS - yawOutput;
    int rightSpeed = BS + yawOutput;

    // Ensure values stay within motor limits
    leftSpeed = constrain(leftSpeed, 0, 255);
    rightSpeed = constrain(rightSpeed, 0, 255);

    if (isMovingForward) forward(leftSpeed, rightSpeed);
    else reverse(leftSpeed, rightSpeed);
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

// Stop the robot
void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
  Serial.println("Stop");
}
