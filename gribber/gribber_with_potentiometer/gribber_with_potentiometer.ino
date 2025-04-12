const int relayA = 24;    // Gripper arm in/out
const int relayB = 25;    // Gripper open/close
const int PWM = 13;        // Gripper tilt motor (PWM-capable)
const int DIR = 32;        // Direction control for tilt motor
const int potPin = A0;    // Potentiometer feedback pin

bool Done = true;

enum GripperState {
  IDLE,
  ARM_EXTENDED_AND_TILTED_DOWN,
  BALL_GRABBED
};

GripperState currentState = IDLE;

void setup() {
  Serial.begin(115200);
  Serial3.begin(115200);  // Serial from ESP32

  pinMode(relayA, OUTPUT);
  pinMode(relayB, OUTPUT);
  pinMode(PWM, OUTPUT);
  pinMode(DIR, OUTPUT);

  digitalWrite(relayA, HIGH); // Arm retracted
  digitalWrite(relayB, HIGH); // Gripper closed
  analogWrite(PWM, 0);

  Serial3.setTimeout(200);
  Serial.println("System Ready.");
}

void loop() {
  String command = "";

  if (Serial3.available()) {
    command = Serial3.readStringUntil('\n');
    command.trim();
    Serial.println("Received: " + command);
  }

  // STEP 1: Extend arm & tilt down using potentiometer feedback
  if (command == "BUTTON 1" && currentState == IDLE) {
    Serial.println("Step 1: Extending arm & tilting down");

    digitalWrite(relayA, LOW);  // Extend arm
    delay(1000);

    int potVal = analogRead(potPin);
    while (abs(potVal - 245) > 5) { // Target tilt down position
      potVal = analogRead(potPin);
      Serial.print("Potentiometer: ");
      Serial.println(potVal);

      if (potVal < 245) {
        digitalWrite(DIR, HIGH);  // Tilt down
        analogWrite(PWM, 150);
      } else {
        analogWrite(PWM, 0);
      }

      delay(50);
    }

    analogWrite(PWM, 0); // Stop motor
    currentState = ARM_EXTENDED_AND_TILTED_DOWN;
    Serial.println("Arm extended & tilted to target position.");
  }

  // STEP 2: Grab or regrab
  else if (command == "BUTTON 2") {
    Serial.println("Step 2: Releasing then grabbing");

    digitalWrite(relayB, HIGH); // Open gripper
    delay(200);
    digitalWrite(relayB, LOW);  // Close gripper
    delay(800);

    currentState = BALL_GRABBED;
    Serial.println("Ball grabbed.");
  }

  // STEP 3: Tilt back up & retract
  else if (command == "BUTTON 1" && currentState == BALL_GRABBED && Done) {
    Serial.println("Step 3: Tilting up & retracting arm");

    int potVal = analogRead(potPin);
    while (abs(potVal - 5) > 5) { // Target tilt-up position
      potVal = analogRead(potPin);
      Serial.print("Potentiometer: ");
      Serial.println(potVal);

      if (potVal > 5) {
        digitalWrite(DIR, LOW);  // Tilt up
        analogWrite(PWM, 150);
      } else {
        analogWrite(PWM, 0);
      }

      delay(50);
    }

    analogWrite(PWM, 0); // Stop motor

    digitalWrite(relayB, HIGH); // Open gripper to release
    delay(500);

    digitalWrite(relayA, HIGH); // Retract arm
    delay(500);

    currentState = IDLE;
    Done = true;

    Serial.println("Cycle complete. Back to IDLE.");
  }
}
