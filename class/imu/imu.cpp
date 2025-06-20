#include "IMU.h"

IMU::IMU() {
  angle = 0.0;
  previousTime = 0;
}

void IMU::initializeSensor() {
  Wire1.begin();
  if (!bno.begin_I2C(0x4A, &Wire1)) {
    Serial.println("Failed to find BNO08x chip");
    while (1);
  }
  Serial.println("BNO08x Found!");

  if (!bno.enableReport(SH2_GYROSCOPE_CALIBRATED)) {
    Serial.println("Could not enable gyroscope data reporting");
    while (1);
  }
}

void IMU::processGyroscopeData() {
  if (!bno.getSensorEvent(&sensorValue)) {
    Serial.println("Failed to read sensor data");
    return;
  }

  if (sensorValue.sensorId == SH2_GYROSCOPE_CALIBRATED) {
    float zRotation = sensorValue.un.gyroscope.z;
    unsigned long currentTime = millis();

    float deltaTime = (currentTime - previousTime) / 1000.0;
    previousTime = currentTime;

    if (abs(zRotation) > rotationThreshold) {
      angle += zRotation * deltaTime;  // theta = omega * deltaTime
    }

    Serial.print("Angle displaced: ");
    Serial.print(angle);
    Serial.print(" rad, ");
  }
}

float IMU::getAngleInRad() {
  return angle;
}

float IMU::getAngleInDeg() {
  return angle * (180.0 / PI);
}
