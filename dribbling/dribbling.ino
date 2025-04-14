#include <Arduino.h>
#include "ps.h"
#include "drive.h"
#include "BLDC.h"
#include "IR.h"

// Flywheel motor pins
const int flywheelPWM = 9;
const int flywheelDir = 8;

// Flags and timing
bool prevDpadRight = false;
bool flywheelOn = false;
bool bldcStarted = false;
bool irStarted = true;
unsigned long bldcStartTime = 0;

// Objects
PS ps;
Drive drive(4, 22, 5, 24, 6, 26, 7, 28);
BLDC bldc(10);
IR ir(A0, A1, A2, 8);

void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);

    drive.begin();
    ir.begin();
    pinMode(13, OUTPUT);
    pinMode(flywheelPWM, OUTPUT);
    pinMode(flywheelDir, OUTPUT);
}

void loop() {
    ps.updateFromSerial(Serial3);
    drive.move(ps.axisY, ps.axisRX, ps.l2, ps.r2);

    if (ps.dPadRight && !prevDpadRight && !flywheelOn) {
        flywheelOn = true;
        Serial.println("dab gya");

        // Flywheel ON
        digitalWrite(flywheelDir, HIGH);
        analogWrite(flywheelPWM, 100);

        // Start BLDC
        bldc.up();
        delay(2000);
        bldc.setSpeed(0);  // ← just call it normally
        irStarted = false;     // ← no if condition needed here
    }

    prevDpadRight = ps.dPadRight;

    // IR sensor logic
    if (!irStarted) {
        ir.update();
        // Serial.println("IR");
    }
    // Stop everything on dPadLeft
    if (ps.dPadLeft && flywheelOn) {
        analogWrite(flywheelPWM, 0);
        digitalWrite(flywheelDir, LOW);
        flywheelOn = false;
        bldcStarted = false;
        irStarted = false;
    }
}
