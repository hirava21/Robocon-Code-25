const int relayA = 8;    // Gripper arm in/out
const int relayB = 9;    // Gripper open/close
const int motorPWM = 2; // Gripper tilt motor
const int button1 = 2;
const int button2 = 3;

bool actionStarted = false;
bool ballGrabbed = false;

void setup() {
  pinMode(relayA, OUTPUT);
  pinMode(relayB, OUTPUT);
  pinMode(motorPWM, OUTPUT);

  pinMode(button1, INPUT_PULLUP); // active LOW
  pinMode(button2, INPUT_PULLUP); // active LOW

  // Initial states
  digitalWrite(relayA, LOW);
  digitalWrite(relayB, LOW);
  analogWrite(motorPWM, 0);
}

void loop() { 
  if (digitalRead(button1) == LOW && !actionStarted) {
    actionStarted = true;

    // Step 1: Extend gripper arm
    digitalWrite(relayA, HIGH);
    delay(1000);  // adjust as needed

    // Step 2: Tilt gripper down
    digitalWrite(motorPWM, HIGH);
    analogWrite(motorPWM, 150); // forward direction PWM
    delay(1000);                // run motor for 1s
    analogWrite(motorPWM, 0);   // stop motor
  }

  // When Button 2 is pressed (continue sequence)
  if (digitalRead(button2) == LOW && actionStarted && !ballGrabbed) {
    ballGrabbed = true;

    // Step 3: Grab ball
    digitalWrite(relayB, HIGH);
    delay(1000);  // adjust grip timing

    // Step 4: Tilt gripper back up
    digitalWrite(motorPWM, LOW);
    analogWrite(motorPWM, 100); // reverse direction (depends on motor wiring)
    delay(1000);
    analogWrite(motorPWM, 0);

    // Step 5: Release ball
    digitalWrite(relayB, LOW);
    delay(500);

    // Step 6: Retract gripper arm
    digitalWrite(relayA, LOW);

    // Reset state (optional)
    delay(1000);
    actionStarted = false;
    ballGrabbed = false;
  }
}
