#include <Arduino.h>
#include "ps.h"
#include "Gripper.h"
#include "drive.h"
#include "conveyor.h"
int p1 = 13, d1 = 32;
bool motorOn = false;
bool prevDpadRight = false;

PS ps;
Gripper gripper(38, 40, 9, 50);  // relayA, relayB, PWM, DIR
Drive drive(4, 22, 5, 24, 6, 26, 7, 28);
Conveyor conveyor(30,8);

void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);

    gripper.begin();
    drive.begin();
}

void loop() {
    ps.updateFromSerial(Serial3);
    drive.move(ps.axisY, ps.axisRX, ps.l2, ps.r2);

    if (ps.dPadUp) {
        conveyor.updateFrom("BUTTON 1");  // D-Pad Up triggers Conveyor button 1
    } 
    else if (ps.dPadDown) {
        conveyor.updateFrom("BUTTON 2");  // D-Pad Down triggers Conveyor button 2
    }

    if (ps.buttonUp) {
        gripper.button1();
    }

    if (ps.buttonDown) {
        gripper.button2();
    }
    if (ps.dPadRight && !prevDpadRight) { // Only on press, not hold
        motorOn = !motorOn;  // Toggle state

        if (motorOn) {
            digitalWrite(d1, HIGH);
            analogWrite(p1, 255);
        } else {
            digitalWrite(d1, LOW);
            analogWrite(p1, 0);
        }
    }

    prevDpadRight = ps.dPadRight;
}


