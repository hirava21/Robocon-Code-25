#include "ps.h"

void PS::resetButtons() {
    dPadRight = false;
    dPadLeft = false;
    dPadUp = false;
    dPadDown = false;
    buttonUp = false;
    buttonDown = false;
    buttonLeft = false;
    buttonRight = false;
}

void PS::updateFromSerial(HardwareSerial& serial) {
    if (serial.available()) {
        String command = serial.readStringUntil('\n');
        command.trim();

        // Reset button/dpad states (but not triggers or joysticks)
        resetButtons();

        if (command == "TARGET_1") {
            dPadRight = true;
        } else if (command == "TARGET_2") {
            dPadLeft = true;
        } else if (command == "TARGET_3") {
            dPadUp = true;
        } else if (command == "TARGET_4") {
            dPadDown = true;
        } else if (command == "BUTTON_1") {
            buttonUp = true;
        } else if (command == "BUTTON_2") {
            buttonDown = true;
        } else if (command == "BUTTON_3") {
            buttonLeft = true;
        } else if (command == "BUTTON_4") {
            buttonRight = true;
        } else if (command.startsWith("L2 VALUE:")) {
            l2 = command.substring(10).toInt();
            lastL2Update = millis();
        } else if (command.startsWith("R2 VALUE:")) {
            r2 = command.substring(10).toInt();
            lastR2Update = millis();
        } else if (command.startsWith("LEFT_JOYSTICK:")) {
            int xIndex = command.indexOf("X=");
            int yIndex = command.indexOf("Y=");
            if (xIndex >= 0 && yIndex > xIndex) {
                axisX = command.substring(xIndex + 2, command.indexOf(',', xIndex)).toInt();
                axisY = command.substring(yIndex + 2).toInt();
            }
        } else if (command.startsWith("RIGHT_JOYSTICK:")) {
            int rxIndex = command.indexOf("RX=");
            int ryIndex = command.indexOf("RY=");
            if (rxIndex >= 0 && ryIndex > rxIndex) {
                axisRX = command.substring(rxIndex + 3, command.indexOf(',', rxIndex)).toInt();
                axisRY = command.substring(ryIndex + 3).toInt();
            }
        }
    }

    handleTimeouts();  // Check for stuck L2/R2
}

void PS::handleTimeouts() {
    const unsigned long timeout = 200; // 200ms
    unsigned long now = millis();

    if (now - lastL2Update > timeout) {
        l2 = 0;
    }

    if (now - lastR2Update > timeout) {
        r2 = 0;
    }
}

