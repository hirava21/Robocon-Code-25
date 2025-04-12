#include "IMU.h"
#include <Arduino.h>

IMU::IMU(uint8_t address) : i2cAddress(address), initialYawRad(0), currentYawRad(0) {}

bool IMU::begin() {
    Wire.begin();
    if (!imuSensor.begin(i2cAddress)) {
        Serial.println("BNO085 not detected. Check your wiring or I2C address.");
        return false;
    }
    Serial.println("BNO085 detected!");
    imuSensor.enableRotationVector(100);
    delay(1000); // Allow sensor to stabilize
    calibrate(); // Perform initial yaw calibration
    return true;
}

void IMU::calibrate() {
    float sumYawRad = 0;
    int validReadings = 0;

    for (int i = 0; i < 100; i++) {
        if (imuSensor.dataAvailable()) {
            float qw = imuSensor.getQuatReal();
            float qx = imuSensor.getQuatI();
            float qy = imuSensor.getQuatJ();
            float qz = imuSensor.getQuatK();

            float yawRad = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
            sumYawRad += yawRad;
            validReadings++;
        }
        delay(10);
    }

    if (validReadings > 0) {
        initialYawRad = sumYawRad / validReadings;
        Serial.print("Initial Yaw (radians): ");
        Serial.println(initialYawRad, 6);
        Serial.print("Initial Yaw (degrees): ");
        Serial.println(initialYawRad * 180.0 / PI, 2);
    } else {
        Serial.println("Failed to get initial yaw.");
    }
}

void IMU::update() {
    if (imuSensor.dataAvailable()) {
        float qw = imuSensor.getQuatReal();
        float qx = imuSensor.getQuatI();
        float qy = imuSensor.getQuatJ();
        float qz = imuSensor.getQuatK();

        float yawRad = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
        currentYawRad = yawRad - initialYawRad;

        if (currentYawRad < 0) {
            currentYawRad += 2 * PI;
        }
    }
}

float IMU::rad() {
    return currentYawRad;
}

float IMU::deg() {
    return currentYawRad * 180.0 / PI;
}
