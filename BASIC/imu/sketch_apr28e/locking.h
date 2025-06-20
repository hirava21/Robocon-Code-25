#ifndef LOCKING_H
#define LOCKING_H

#include "IMU.h"

class Locking {
  private:
    IMU imu;
    float targetAngle;         // Target angle in degrees
    const float turnTolerance = 5.0;  // Degrees tolerance
    const int turnMotorSpeed = 50;
    bool lockingActive = false;

    // Motor control helper functions
    void setMotor(int pwmPin, int dirPin, int speed, bool forward);
    void stopMotors();
    void clockwise();
    void anticlockwise();

  public:
    Locking();
    void begin();
    void update();      // Should be called repeatedly in loop
    void target(float angle); // Set a new target angle
};

#endif