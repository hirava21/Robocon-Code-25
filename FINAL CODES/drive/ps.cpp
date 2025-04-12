#include "ps.h"

PS ps;  // Define the global instance

PS::PS() {
    _axisY = _axisX = _dpad = _throttle = _brake = 0;
}

void PS::update() {
    if (Serial3.available() > 0) {
        String data = Serial3.readStringUntil('\n');
        int delimiter1 = data.indexOf(',');
        int delimiter2 = data.indexOf(',', delimiter1 + 1);
        int delimiter3 = data.indexOf(',', delimiter2 + 1);
        int delimiter4 = data.indexOf(',', delimiter3 + 1);

        if (delimiter1 != -1 && delimiter4 != -1) {
            _axisY = data.substring(0, delimiter1).toInt();
            _axisX = data.substring(delimiter1 + 1, delimiter2).toInt();
            _dpad = data.substring(delimiter2 + 1, delimiter3).toInt();
            _throttle = data.substring(delimiter3 + 1, delimiter4).toInt();
            _brake = data.substring(delimiter4 + 1).toInt();

                        // Ignore small joystick values
            if (abs(_axisX) < 5) _axisX = 0;
            if (abs(_axisY) < 55) _axisY = 0;
        }
    }
}

int PS::getAxisY() { return _axisY; }
int PS::getAxisX() { return _axisX; }
int PS::getDpad() { return _dpad; }
int PS::getThrottle() { return _throttle; }
int PS::getBrake() { return _brake; }
