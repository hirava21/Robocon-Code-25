#include "ps.h"

void PS::update(String command) {
    reset();  // reset previous states before updating

    if (command == "RIGHT") {
        buttonRight = true;
    } else if (command == "BUTTON") {
        buttonLeft = true;
    } else if (command == "TARGET_3") {
        buttonUp = true;
    } else if (command == "CROSS") {
        buttonDown = true;
    }
}

void PS::reset() {
    buttonRight = false;
    buttonLeft = false;
    buttonUp = false;
    buttonDown = false;
}
