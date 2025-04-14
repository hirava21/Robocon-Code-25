#include "IR.h"

IR::IR(int ir1, int ir2, int ir3, int relay, int threshold) {
  irPins[0] = ir1;
  irPins[1] = ir2;
  irPins[2] = ir3;
  relayPin = relay;
  detectionThreshold = threshold;
  detectionCount = 0;
  objectPreviouslyDetected = false;
}

void IR::begin() {
  pinMode(relayPin, OUTPUT);
  digitalWrite(relayPin, HIGH);  // Relay OFF initially
  pinMode(13, OUTPUT);
}

void IR::update() {
  bool anyDetected = false;

  // Check all IR sensors
  for (int i = 0; i < 3; i++) {
    int value = analogRead(irPins[i]);
    if (value < detectionThreshold) {
      anyDetected = true;
      break;
    }
  }

  // Detect transition
  if (anyDetected && !objectPreviouslyDetected) {
    detectionCount++;
    Serial.print("Detection #");
    Serial.println(detectionCount);
    objectPreviouslyDetected = true;
  }

  // Reset flag when no object detected
  if (!anyDetected) {
    objectPreviouslyDetected = false;
  }

  // If detectionCount hits 2, trigger relay
  if (detectionCount >= 2) {
    digitalWrite(relayPin, LOW);  
    digitalWrite(13, HIGH); // Relay ON
    delay(1000);
    digitalWrite(13, LOW);
    Serial.println("Relay LOW");
    delay(1000);                   // Hold for 1 second
    digitalWrite(relayPin, HIGH);  // Relay OFF again
    detectionCount = 0;            // Reset counter if you want a repeatable trigger
  }
}
