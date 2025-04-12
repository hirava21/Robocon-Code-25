#ifndef IMU_H
#define IMU_H

#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

class IMU {
public:
    IMU(uint8_t address = 0x4A); // Constructor with optional I2C address
    bool begin(); // Initialize the sensor
    void calibrate(); // Calibrate and set initial yaw
    void update(); // Update sensor values

    float rad(); // Get yaw in radians
    float deg(); // Get yaw in degrees

private:
    BNO080 imuSensor;
    uint8_t i2cAddress;
    float initialYawRad;
    float currentYawRad;
};

#endif // IMU_H
