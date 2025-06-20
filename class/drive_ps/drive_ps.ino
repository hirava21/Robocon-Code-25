#include <Arduino.h>
#include "ps.h"
#include "drive.h"

PS ps;
Drive drive(14, 30, 15, 31, 18, 32, 19, 33);

void setup() {
    Serial.begin(115200);
    Serial5.begin(115200);
    drive.begin();
}

void loop() {
    ps.update();
    drive.move(ps.axisY, ps.axisRX, ps.brake, ps.throttle);

}