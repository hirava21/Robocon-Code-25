#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>
BNO080 myIMU;
float initialYaw = 0; float adjustedYaw = 0;
int BS = 50; int correctionFactor =20; //basically the speed at which the bot will move when error is seen 
int FL_pwm = 8, FR_pwm = 10, RL_pwm = 6, RR_pwm = 12;
int FL_dir = 9, FR_dir = 11, RL_dir = 7, RR_dir = 13;
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
  pinMode(FR_pwm, OUTPUT); pinMode(FR_dir, OUTPUT);
  pinMode(RR_pwm, OUTPUT); pinMode(RR_dir, OUTPUT);
  pinMode(RL_pwm, OUTPUT); pinMode(RL_dir, OUTPUT);
  pinMode(FL_pwm, OUTPUT); pinMode(FL_dir, OUTPUT);
}
void loop() {
  // forward(30,30);
  // delay(2000);
  // stop();
  // delay(100000000000000);
  if (myIMU.dataAvailable() == true) {
    // Read the quaternion data
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();
    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw = yaw * 180.0 / PI;
    float adjustedYaw = yaw - initialYaw;
    if (adjustedYaw > 180) {
      adjustedYaw -= 360;
    } else if (adjustedYaw < -180) {
      adjustedYaw += 360;
    }
    // if (adjustedYaw < 0) {
    //   adjustedYaw += 360;
    // }
    Serial.print("Adjusted Yaw: ");
    Serial.println(adjustedYaw, 2);
  if (adjustedYaw > -5 && adjustedYaw < 5) {
    Serial.println("Moving Straight");
    forward(BS, BS);
  } 
  else if (adjustedYaw >= 5 && adjustedYaw < 10) {
    Serial.println("Correcting Right (Slow down Right)");
    forward(BS, BS - correctionFactor);
  } 
  else if (adjustedYaw <= -5 && adjustedYaw > -10) {
    Serial.println("Correcting Left (Slow down Left)");
    forward(BS - correctionFactor, BS);
  } 
  else if (adjustedYaw >= 10) {
    Serial.println("Turning Clockwise");
    clockwise();
  } 
  else if (adjustedYaw <= -10) {
    Serial.println("Turning Anticlockwise");
    anticlockwise();
  }
}
}
void moveStraight(){
  // if (adjustedYaw > -5 && adjustedYaw <5){
  //   forward(BS, BS);
  //   Serial.println("straight");
  // }
  // else if (adjustedYaw >=5 && adjustedYaw <10){
  //   forward(BS, BS -correctionFactor);
  //   Serial.println("right");
  //    }

  // else if (adjustedYaw <= -5 && adjustedYaw <-10){
  //   forward(BS- correctionFactor, BS);
  //   Serial.println("left");
  // }
  // else if (adjustedYaw <= 10){
  //   clockwise();
  // }
  // else if (adjustedYaw <= -10){
  //   anticlockwise();
  // }
}
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
void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
  Serial.println("Stop");
}