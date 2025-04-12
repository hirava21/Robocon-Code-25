#include "ps.h"

int pwm1 = 8, pwm2 = 10;
int dir1 = 30, dir2 = 6;

bool motor1Direction = true;  // true = CW, false = CCW
bool motor2Direction = true;

PS ps;

void setup() {
    Serial.begin(115200);
    Serial3.begin(115200);  // Communication from ESP32

    pinMode(pwm1, OUTPUT);
    pinMode(pwm2, OUTPUT);
    pinMode(dir1, OUTPUT);
    pinMode(dir2, OUTPUT);

    Serial.println("System Ready.");
}

void loop() {
    if (Serial3.available()) {
        String command = Serial3.readStringUntil('\n');
        command.trim();
        ps.update(command);

        if (ps.buttonUp) {  // Button 1 - Motor 1 toggle
digitalWrite(dir1, HIGH);
analogWrite(pwm1, 255);
        }

        // if (ps.buttonRight) {  // Button 2 - Motor 2 toggle
        //     motor2Direction = !motor2Direction;
        //     digitalWrite(dir2, motor2Direction ? HIGH : LOW);
        //     analogWrite(pwm2, 255);  // Full speed
        // }

        ps.reset();
    }
}
