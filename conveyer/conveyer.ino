#include "ps.h"
#include "conveyer.h"

PS ps;
Conveyer conveyer(5, 4);  // Example PWM and DIR pins

bool isForwardRunning = false;

void setup() {
    Serial.begin(9600);
    Serial1.begin(115200);  // Communicate with ESP32 via Serial1

    conveyer.begin();
}

void loop() {
    if (Serial1.available()) {
        String cmd = Serial1.readStringUntil('\n');
        cmd.trim();  // Remove whitespace/newlines
        ps.update(cmd);
    }

    if (ps.buttonUp) {
        if (isForwardRunning) {
            conveyer.stop();
            isForwardRunning = false;
        } else {
            conveyer.up();
            isForwardRunning = true;
        }
    }

    if (ps.buttonDown) {
        conveyer.down();
        isForwardRunning = false;
    }

    ps.reset();
}
