#include "Gripper.h"

Gripper::Gripper(int relayA, int relayB, int pwmPin, int dirPin)
    : relayA(relayA), relayB(relayB), pwmPin(pwmPin), dirPin(dirPin) {}

void Gripper::begin() {
    pinMode(relayA, OUTPUT);
    pinMode(relayB, OUTPUT);
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin, OUTPUT);

    digitalWrite(relayA, HIGH);  // Retracted
    digitalWrite(relayB, HIGH);  // Closed
    analogWrite(pwmPin, 0);
}

void Gripper::button1() {
    if (currentState == IDLE) {
        extendArmAndTiltDown();
    } else if (currentState == BALL_GRABBED) {
        tiltUpAndRetract();
    }
}

void Gripper::button2() {
    openCloseGripper();
}

void Gripper::extendArmAndTiltDown() {
    Serial.println("Step 1: Extending arm & tilting down");

    digitalWrite(relayA, LOW);  // Extend arm
    delay(1000);

    digitalWrite(dirPin, LOW);  // Tilt down
    analogWrite(pwmPin, 150);
    delay(4500);
    analogWrite(pwmPin, 0);

    currentState = ARM_EXTENDED_AND_TILTED_DOWN;
    Serial.println("Arm extended & tilted down.");
}

void Gripper::openCloseGripper() {
    if (!gripperOpen) {
        Serial.println("Step 2: Opening gripper");
        digitalWrite(relayB, HIGH); // Open
        gripperOpen = true;
    } else {
        Serial.println("Step 2: Closing gripper");
        digitalWrite(relayB, LOW); // Close
        gripperOpen = false;
        currentState = BALL_GRABBED;
        Serial.println("Ball grabbed.");
    }
}

void Gripper::tiltUpAndRetract() {
    Serial.println("Step 3: Tilting up & retracting arm");

    digitalWrite(dirPin, HIGH);     // Tilt up
    analogWrite(pwmPin, 150);
    delay(5500);
    analogWrite(pwmPin, 0);

    // digitalWrite(relayB, HIGH);  // Retract gripper
    // delay(500);

    digitalWrite(relayA, HIGH);  // Retract arm
    delay(2000);

    gripperOpen = false;
    currentState = IDLE;

    Serial.println("Cycle complete. Back to IDLE.");
}