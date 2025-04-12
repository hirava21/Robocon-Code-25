#include "Motor.h"

// Constructor to initialize motor pins
Motor::Motor(int d1, int d2, int d3, int d4, int p1, int p2, int p3, int p4) {
    dirPins[0] = d1; dirPins[1] = d2; dirPins[2] = d3; dirPins[3] = d4;
    pwmPins[0] = p1; pwmPins[1] = p2; pwmPins[2] = p3; pwmPins[3] = p4;

    // Set pins as outputs
    for (int i = 0; i < 4; i++) {
        pinMode(dirPins[i], OUTPUT);
        pinMode(pwmPins[i], OUTPUT);
    }
}

// Function to control motor movement
void Motor::drive(int dir1, int dir2, int dir3, int dir4, int pwm1, int pwm2, int pwm3, int pwm4) {
    digitalWrite(dirPins[0], dir1);
    digitalWrite(dirPins[1], dir2);
    digitalWrite(dirPins[2], dir3);
    digitalWrite(dirPins[3], dir4);

    analogWrite(pwmPins[0], pwm1);
    analogWrite(pwmPins[1], pwm2);
    analogWrite(pwmPins[2], pwm3);
    analogWrite(pwmPins[3], pwm4);
}
