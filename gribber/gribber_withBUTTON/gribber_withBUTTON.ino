const int relayA = 38;    // Gripper arm in/out
const int relayB = 40;    // Gripper open/close
const int PWM = 9;        // Gripper tilt motor (PWM-capable)
const int DIR = 50;        // Direction control for tilt motor
bool gripperOpen = false;

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

  digitalWrite(relayA, HIGH); // Retracted
  // delay()
  digitalWrite(relayB, HIGH); // Closed
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

  // STEP 1: Extend arm & tilt down
  if (command == "BUTTON 1" && currentState == IDLE) {
    Serial.println("Step 1: Extending arm & tilting down");

    digitalWrite(relayA, LOW);  // Extend arm
    delay(1000);
    digitalWrite(DIR, LOW);    // Tilt down
    analogWrite(PWM, 150);
    delay(4500);
    analogWrite(PWM, 0);

    currentState = ARM_EXTENDED_AND_TILTED_DOWN;
    Serial.println("Arm extended & tilted down.");
  }

  // STEP 2: Toggle gripper open/close
  else if (command == "BUTTON 2") {
    if (!gripperOpen) {
      Serial.println("Step 2: Opening gripper");
      digitalWrite(relayB, HIGH); // Open
      gripperOpen = true;
    } else {
      Serial.println("Step 2: Closing gripper");
      digitalWrite(relayB, LOW); // Close
      gripperOpen = false;
      currentState = BALL_GRABBED;
      Serial.println("Ball grabbed.");
    }
  }

  // STEP 3: Tilt back up & retract
  else if (command == "BUTTON 1" && currentState == BALL_GRABBED) {
    Serial.println("Step 3: Tilting up & retracting arm");

    digitalWrite(DIR, HIGH);     // Tilt up
    analogWrite(PWM, 150);
    delay(5500);
    analogWrite(PWM, 0);

    digitalWrite(relayB, HIGH); // Retract arm
    delay(500);

    digitalWrite(relayA, HIGH); // Release object
    delay(2000);

    gripperOpen = false; // Assume gripper closed at end of cycle
    currentState = IDLE;
    Serial.println("Cycle complete. Back to IDLE.");
  }
}