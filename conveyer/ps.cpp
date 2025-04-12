#include "ps.h"

void PS::update(String command) {
    reset();

    if (command == "RIGHT") {
        buttonRight = true;
    } else if (command == "LEFT") {
        buttonLeft = true;
    } else if (command == "BUTTON 1") {
        buttonUp = true;
    } else if (command == "BUTTON 2") {
        buttonDown = true;
    }
}

void PS::reset() {
    buttonRight = false;
    buttonLeft = false;
    buttonUp = false;
    buttonDown = false;
}
