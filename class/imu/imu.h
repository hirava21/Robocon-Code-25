#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <Adafruit_Sensor.h>

class IMU {
  private:
    Adafruit_BNO08x bno;
    sh2_SensorValue_t sensorValue;
    float angle;                // Angle in radians
    unsigned long previousTime;
    const float rotationThreshold = 0.05;  // Threshold for rotation detection (in radians)
    const float angleTolerance = 0.05;     // Tolerance for correction (in radians)

  public:
    IMU();
    void initializeSensor();
    void processGyroscopeData();
    float getAngleInRad();
    float getAngleInDeg();
};

#endif
