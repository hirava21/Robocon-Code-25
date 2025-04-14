#include "conveyor.h"

Conveyor::Conveyor(uint8_t dirPin, uint8_t pwmPin) {
    motorDirPin = dirPin;
    motorPwmPin = pwmPin;
    pinMode(motorDirPin, OUTPUT);
    pinMode(motorPwmPin, OUTPUT);

    forwardRunning = false;
    reverseRunning = false;
}

void Conveyor::updateFrom(const String& command) {
    if (command == "BUTTON 1") {
        if (!forwardRunning) {
            digitalWrite(motorDirPin, LOW);
            analogWrite(motorPwmPin, 255);
            forwardRunning = true;
            reverseRunning = false;
            Serial.println("Motor FORWARD");
        } else {
            analogWrite(motorPwmPin, 0);
            forwardRunning = false;
            Serial.println("Motor STOPPED (from FORWARD)");
        }
    } 
    else if (command == "BUTTON 2") {
        if (!reverseRunning) {
            digitalWrite(motorDirPin, HIGH);
            analogWrite(motorPwmPin, 255);
            reverseRunning = true;
            forwardRunning = false;
            Serial.println("Motor REVERSE");
        } else {
            analogWrite(motorPwmPin, 0);
            reverseRunning = false;
            Serial.println("Motor STOPPED (from REVERSE)");
        }
    }
}