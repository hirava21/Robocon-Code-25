int PWM1 = 8, DIR1 = 9; int PWM2 = 10, DIR2 = 11;  
int PWM3 = 12, DIR3 = 13; int PWM4 = 6, DIR4 = 7;  int motorSpeed = 50;
#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 myIMU;
float initialYaw = 0;
float adjustedYaw = 0;

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
  pinMode(PWM1, OUTPUT); pinMode(DIR1, OUTPUT);
  pinMode(PWM2, OUTPUT); pinMode(DIR2, OUTPUT);
  pinMode(PWM3, OUTPUT); pinMode(DIR3, OUTPUT);
  pinMode(PWM4, OUTPUT); pinMode(DIR4, OUTPUT);
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
  delay(500);
}

void mapping() {
  if (adjustedYaw > -5 && adjustedYaw < 5) {
    Serial.println("s");
    stop();
  } else if (adjustedYaw >= 5) {
    Serial.println("clock");
    clockwise();
  } else if (adjustedYaw <= -5) {
    Serial.println("anticlock");
    anticlockwise();
  }
}

void clockwise() {
  digitalWrite(DIR1, 1);
  analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, 0);
  analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, 0);
  analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, 1);
  analogWrite(PWM4, motorSpeed);
  Serial.println("clockwise");
}
void anticlockwise() {
  digitalWrite(DIR1, 0);
  analogWrite(PWM1, motorSpeed);
  digitalWrite(DIR2, 1);
  analogWrite(PWM2, motorSpeed);
  digitalWrite(DIR3, 1);
  analogWrite(PWM3, motorSpeed);
  digitalWrite(DIR4, 0);
  analogWrite(PWM4, motorSpeed);
  Serial.println("clockwise");
}
void stop(){
analogWrite(PWM1, 0);
analogWrite(PWM2, 0);
analogWrite(PWM3, 0);
analogWrite(PWM4, 0);
Serial.println("stop");
}

