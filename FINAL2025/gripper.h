#ifndef GRIPPER_H
#define GRIPPER_H

#include <Arduino.h>

class Gripper {
public:
    Gripper(int relayA, int relayB, int pwmPin, int dirPin);

    void begin();
    void button1();  // Extend/Tilt or Retract/Reset based on state
    void button2();  // Open/Close gripper

private:
    int relayA, relayB, pwmPin, dirPin;
    bool gripperOpen = false;

    enum GripperState {
        IDLE,
        ARM_EXTENDED_AND_TILTED_DOWN,
        BALL_GRABBED
    };

    GripperState currentState = IDLE;

    void extendArmAndTiltDown();
    void openCloseGripper();
    void tiltUpAndRetract();
};

#endif