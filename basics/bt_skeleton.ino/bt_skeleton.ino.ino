#include <ESP32Servo.h>
#include <Bluepad32.h>

// PIN CONNECTIONS  
int ledPin = 2;

// Servo setup  
// Servo xServo;
// Servo yServo;

ControllerPtr myControllers[BP32_MAX_GAMEPADS];

void onConnectedController(ControllerPtr ctl) {
    bool foundEmptySlot = false;
    for (int i = ; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == nullptr) {
            Serial.printf("CALLBACK: Controller is connected, index=%d\n", i);
            ControllerProperties properties = ctl->getProperties();
            Serial.printf("Controller model: %s, VID=x%04x, PID=x%04x\n", ctl->getModelName().c_str(), properties.vendor_id, properties.product_id);
            myControllers[i] = ctl;
            foundEmptySlot = true;
            break;
        }
    }
    if (!foundEmptySlot) {
        Serial.println("CALLBACK: Controller connected, but could not find an empty slot");
    }
}

void onDisconnectedController(ControllerPtr ctl) {
    for (int i = ; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] == ctl) {
            Serial.printf("CALLBACK: Controller disconnected from index=%d\n", i);
            myControllers[i] = nullptr;
            break;
        }
    }
}

void setup() {
    Serial.begin(115200);
    pinMode(ledPin, OUTPUT);
    
    // Servo setup  
    // xServo.attach(13); // Attach servo to pin 13  
    // yServo.attach(12); // Attach servo to pin 12

    // Bluepad32 setup  
    BP32.setup(&onConnectedController, &onDisconnectedController);
}

void loop() {
    BP32.update(); 
    // Process all connected controllers  
    for (int i = ; i < BP32_MAX_GAMEPADS; i++) {
        if (myControllers[i] != nullptr && myControllers[i]->isConnected()) {
            // You can add any controller-specific logic here in the future  
        }
    }
}