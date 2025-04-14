#include "drive.h"

Drive::Drive(int p1, int d1, int p2, int d2, int p3, int d3, int p4, int d4)
    : pwm1(p1), dir1(d1), pwm2(p2), dir2(d2), pwm3(p3), dir3(d3), pwm4(p4), dir4(d4) {}

void Drive::begin() {
    pinMode(pwm1, OUTPUT); pinMode(dir1, OUTPUT);
    pinMode(pwm2, OUTPUT); pinMode(dir2, OUTPUT);
    pinMode(pwm3, OUTPUT); pinMode(dir3, OUTPUT);
    pinMode(pwm4, OUTPUT); pinMode(dir4, OUTPUT);
}

void Drive::move(int axisY, int axisRX, int l2, int r2) {
    Serial.print("AxisY: "); Serial.print(axisY);
    Serial.print(" | AxisRX: "); Serial.print(axisRX);
    Serial.print(" | L2: "); Serial.print(l2);
    Serial.print(" | R2: "); Serial.println(r2);

    // Apply deadzone
    if (abs(axisY) < 20) axisY = 0;
    if (abs(axisRX) < 20) axisRX = 0;

    // Translational Movement
    int uppwm = map(axisY, 0, -512, 0, max_pwm);  // Forward
    int downpwm = map(axisY, 0, 512, 0, max_pwm); // Backward
    int rightpwm = map(axisRX, 0, -512, 0, max_pwm); // Right
    int leftpwm = map(axisRX, 0, 512, 0, max_pwm); // Left

    // Rotation from triggers
    int rot = r2 - l2;  // R2 = clockwise, L2 = anticlockwise

    // Mixed motor values for omnidirectional X-drive
    int m1pwm = uppwm - rightpwm - rot;  // Motor 1 (FL)
    int m2pwm = uppwm + rightpwm + rot;  // Motor 2 (FR)
    int m3pwm = downpwm - leftpwm - rot; // Motor 3 (RR)
    int m4pwm = downpwm + leftpwm + rot; // Motor 4 (RL)

    // Determine direction
    m1 = (m1pwm > 0);
    m2 = (m2pwm > 0);
    m3 = (m3pwm < 0);
    m4 = (m4pwm < 0);

    // Absolute PWM values
    M1 = constrain(abs(m1pwm), 0, max_pwm);
    M2 = constrain(abs(m2pwm), 0, max_pwm);
    M3 = constrain(abs(m3pwm), 0, max_pwm);
    M4 = constrain(abs(m4pwm), 0, max_pwm);

    digitalWrite(dir1, m1);
    digitalWrite(dir2, m2);
    digitalWrite(dir3, m3);
    digitalWrite(dir4, m4);

    analogWrite(pwm1, M1);
    analogWrite(pwm2, M2);
    analogWrite(pwm3, M3);
    analogWrite(pwm4, M4);

    // Debug output
    Serial.print("M1 -> DIR: "); Serial.print(m1); Serial.print(" | PWM: "); Serial.println(M1);
    Serial.print("M2 -> DIR: "); Serial.print(m2); Serial.print(" | PWM: "); Serial.println(M2);
    Serial.print("M3 -> DIR: "); Serial.print(m3); Serial.print(" | PWM: "); Serial.println(M3);
    Serial.print("M4 -> DIR: "); Serial.print(m4); Serial.print(" | PWM: "); Serial.println(M4);
}
