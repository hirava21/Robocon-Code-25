#include <Bluepad32.h>

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

#define RX2 16
#define TX2 17

bool lastdPadState1 = false;
bool lastdPadState2 = false;
bool lastdPadState3 = false;
bool lastdPadState4 = false;

bool lastButtonState1 = false;
bool lastButtonState2 = false;
bool lastButtonState3 = false;
bool lastButtonState4 = false;

bool lastL1State = false;
bool lastR1State = false;
bool lastL2State = false;
bool lastR2State = false;

int lastAxisX = 0, lastAxisY = 0;
int lastAxisRX = 0, lastAxisRY = 0;

void onConnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            myControllers[i] = ctl;
            Serial.printf("Controller connected at index=%d\n", i);
            break;
        }
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = 0; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            myControllers[i] = nullptr;
            Serial.printf("Controller disconnected from index=%d\n", i);
            break;
        }
    }
}

void processGamepad(ControllerPtr ctl) {
    uint8_t dpadState = ctl->dpad();
    uint16_t buttonState = ctl->buttons();
    int l2Value = ctl->throttle(); // L2 analog trigger
    int r2Value = ctl->brake();
    int axisX = ctl->axisX();
    int axisY = ctl->axisY();
    int axisRX = ctl->axisRX();
    int axisRY = ctl->axisRY();

    
    bool dpadLeft  = (dpadState & 0x08) != 0;
    bool dpadRight = (dpadState & 0x04) != 0;
    bool dpadDown  = (dpadState & 0x02) != 0;
    bool dpadUp    = (dpadState & 0x01) != 0;

    bool buttonDown  = (buttonState & 0x0001) != 0;
    bool buttonRight = (buttonState & 0x0002) != 0;
    bool buttonLeft  = (buttonState & 0x0004) != 0;
    bool buttonUp    = (buttonState & 0x0008) != 0;

    bool buttonL1 = (buttonState & 0x0010) != 0;
    bool buttonR1 = (buttonState & 0x0020) != 0;
    bool buttonL2 = (buttonState & 0x0040) != 0;
    bool buttonR2 = (buttonState & 0x0080) != 0;


    // D-Pad
    if (dpadRight && !lastdPadState1) {
        Serial2.println("TARGET_1");
        Serial.println("Sent: RIGHT");
    }
    if (dpadLeft && !lastdPadState2) {
        Serial2.println("TARGET_2");
        Serial.println("Sent: LEFT");
    }
    if (dpadUp && !lastdPadState3) {
        Serial2.println("TARGET_3");
        Serial.println("Sent: UP");
    }
    if (dpadDown && !lastdPadState4) {
        Serial2.println("TARGET_4");
        Serial.println("Sent: DOWN");
    }

    // Buttons
    if (buttonUp && !lastButtonState1) {
        Serial2.println("BUTTON_1");
        Serial.println("Sent: BUTTON UP");
    }
    if (buttonDown && !lastButtonState2) {
        Serial2.println("BUTTON_2");
        Serial.println("Sent: BUTTON DOWN");
    }
    if (buttonRight && !lastButtonState3) {
        Serial2.println("BUTTON_4");
        Serial.println("Sent: BUTTON RIGHT");
    }
    if (buttonLeft && !lastButtonState4) {
        Serial2.println("BUTTON_3");
        Serial.println("Sent: BUTTON LEFT");
    }

    // L1, R1, L2, R2 Buttons
    if (l2Value != lastL2State) {
        Serial2.printf("L2 VALUE: %d\n", l2Value);
        Serial.printf("L2 VALUE: %d\n", l2Value);
    }
    if (r2Value != lastR2State) {
        Serial2.printf("R2 VALUE: %d\n", r2Value);
        Serial.printf("R2 VALUE: %d\n", r2Value);
    }

    // Joysticks (no threshold)
    Serial2.printf("LEFT_JOYSTICK: X=%d, Y=%d\n", axisX, axisY);
    Serial.printf("Left Joystick: X=%d, Y=%d\n", axisX, axisY);

    Serial2.printf("RIGHT_JOYSTICK: RX=%d, RY=%d\n", axisRX, axisRY);
    Serial.printf("Right Joystick: RX=%d, RY=%d\n", axisRX, axisRY);

    // Update last states
    lastdPadState1 = dpadRight;
    lastdPadState2 = dpadLeft;
    lastdPadState3 = dpadUp;
    lastdPadState4 = dpadDown;

    lastButtonState1 = buttonUp;
    lastButtonState2 = buttonDown;
    lastButtonState3 = buttonRight;
    lastButtonState4 = buttonLeft;

    lastL1State = buttonL1;
    lastR1State = buttonR1;
    lastL2State = buttonL2;
    lastR2State = buttonR2;

    lastAxisX = axisX;
    lastAxisY = axisY;
    lastAxisRX = axisRX;
    lastAxisRY = axisRY;
}

void processControllers() {
    for (auto ctl : myControllers) {
        if (ctl && ctl->isConnected() && ctl->hasData()) {
            processGamepad(ctl);
        }
    }
}

void setup() {
    Serial.begin(115200);
    Serial2.begin(115200, SERIAL_8N1, RX2, TX2);

    BP32.setup(&onConnectedController, &onDisconnectedController);
    BP32.forgetBluetoothKeys();
}

void loop() {
    BP32.update();
    processControllers();
    delay(150);
}
