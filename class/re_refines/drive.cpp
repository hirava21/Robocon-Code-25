#include "drive.h"

Drive::Drive(int p1, int d1, int p2, int d2, int p3, int d3, int p4, int d4)
    : pwm1(p1), dir1(d1), pwm2(p2), dir2(d2), pwm3(p3), dir3(d3), pwm4(p4), dir4(d4) {}

void Drive::begin() {
    pinMode(pwm1, OUTPUT); pinMode(dir1, OUTPUT);
    pinMode(pwm2, OUTPUT); pinMode(dir2, OUTPUT);
    pinMode(pwm3, OUTPUT); pinMode(dir3, OUTPUT);
    pinMode(pwm4, OUTPUT); pinMode(dir4, OUTPUT);
}

int Drive::ramp(int current, int target) {
    if (current < target) {
        current += rampStep;
        if (current > target) current = target;
    } else if (current > target) {
        current -= rampStep;
        if (current < target) current = target;
    }
    return current;
}

void Drive::move(int axisY, int axisRX, int l2, int r2) {
    // Deadzone
    if (abs(axisY) < 20) axisY = 0;
    if (abs(axisRX) < 20) axisRX = 0;

    // Convert joystick input
    int uppwm = map(axisY, 0, -512, 0, max_pwm);     // Forward
    int downpwm = map(axisY, 0, 512, 0, max_pwm);    // Backward
    int rightpwm = map(axisRX, 0, -512, 0, max_pwm); // Right
    int leftpwm = map(axisRX, 0, 512, 0, max_pwm);   // Left

    int rotation = r2 - l2;

    // Raw motor PWM calculations
    int m1pwm = uppwm - rightpwm + rotation;
    int m2pwm = uppwm + rightpwm - rotation;
    int m3pwm = downpwm - leftpwm + rotation;
    int m4pwm = downpwm + leftpwm - rotation;

    // Direction logic
    bool m1 = (m1pwm > 0);
    bool m2 = (m2pwm > 0);
    bool m3 = (m3pwm < 0);
    bool m4 = (m4pwm < 0);

    // PWM magnitudes
    int M1 = constrain(abs(m1pwm), 0, max_pwm);
    int M2 = constrain(abs(m2pwm), 0, max_pwm);
    int M3 = constrain(abs(m3pwm), 0, max_pwm);
    int M4 = constrain(abs(m4pwm), 0, max_pwm);

    // Straight-line compensation
    bool isStraight = (axisRX == 0 && rotation == 0);
    if (isStraight) {
        int basePWM = max(M1, max(M2, max(M3, M4)));
        M1 = M2 = M3 = M4 = basePWM;
    }

    // Apply ramping
    currentM1 = ramp(currentM1, M1);
    currentM2 = ramp(currentM2, M2);
    currentM3 = ramp(currentM3, M3);
    currentM4 = ramp(currentM4, M4);

    // Output to motors
    digitalWrite(dir1, m1);
    digitalWrite(dir2, m2);
    digitalWrite(dir3, m3);
    digitalWrite(dir4, m4);

    analogWrite(pwm1, currentM1);
    analogWrite(pwm2, currentM2);
    analogWrite(pwm3, currentM3);
    analogWrite(pwm4, currentM4);

    // Debug (optional)
    // Serial.print("M1: "); Serial.print(currentM1);
    // Serial.print(" | M2: "); Serial.print(currentM2);
    // Serial.print(" | M3: "); Serial.print(currentM3);
    // Serial.print(" | M4: "); Serial.println(currentM4);
}
