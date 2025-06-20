#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>
#include "IMU.h"

IMU imu;  // Create IMU object

void setup() {
  Serial.begin(115200);
  imu.initializeSensor();
}

void loop() {
  imu.processGyroscopeData();

  // To get angle in radians
  float angleRad = imu.getAngleInRad();
  Serial.print("Angle in Rad: ");
  Serial.println(angleRad);

  // To get angle in degrees
  float angleDeg = imu.getAngleInDeg();
  Serial.print("Angle in Degrees: ");
  Serial.println(angleDeg);

  delay(100);
}
