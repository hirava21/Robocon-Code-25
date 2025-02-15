int FL_pwm = 8; 
int FR_pwm = 10; 
int RL_pwm = 6; 
int RR_pwm = 12;

int FL_dir = 9; 
int FR_dir = 11; 
int RL_dir = 7; 
int RR_dir = 13; 

int motorSpeed = 25;
volatile unsigned int counter1 = 0;  // Counter for encoder 1 pulses
volatile unsigned int counter2 = 0;  // Counter for encoder 2 pulses

unsigned long lastTime = 0;          // To store the last time value was sent
const long interval = 1000;          // Interval at which to send results (1000 ms = 1 second)
const int pulsesPerRevolution = 600; // Encoder's pulses per revolution (PPR)
const float wheelDiameter = 10;      // Diameter of the wheel in cm
const float wheelCircumference = 3.14159 * wheelDiameter; // Circumference of the wheel in cm

float totalDistance1 = 0.0;          // Total distance traveled for encoder 1 in cm
float totalDistance2 = 0.0;          // Total distance traveled for encoder 2 in cm

bool forwardCompleted = false;       // Flag to check forward motion completi         on
bool rightCompleted = false;// Flag to check right turn completion

void setup() {
  Serial.begin(9600);

  // Configure encoder pins as input
  pinMode(2, INPUT);          // Encoder 1 signal
  pinMode(3, INPUT);          // Encoder 2 signal

  // Enable pull-up resistors
  digitalWrite(2, HIGH);
  digitalWrite(3, HIGH);

  // Attach interrupts for both encoders
  attachInterrupt(digitalPinToInterrupt(2), encoder1ISR, RISING); // Interrupt for encoder 1
  attachInterrupt(digitalPinToInterrupt(3), encoder2ISR, RISING); // Interrupt for encoder 2

  forward();  // Start moving forward initially
}

void loop() {
  if (timeInterval()) {
    sendData();

    if (!forwardCompleted) {
      move750();
    } else if (!rightCompleted) {
      move400();
    } else {
      stop();
      Serial.println("Task completed!");
      while (true);  // Stop further execution
    }
  }
}

void encoder1ISR() {
  counter1++; // Increment counter for encoder 1
}

void encoder2ISR() {
  counter2++; // Increment counter for encoder 2
}

void sendData() {
  // Calculate RPM for both encoders
  float rpm1 = (float(counter1) * 60) / pulsesPerRevolution;
  float rpm2 = (float(counter2) * 60) / pulsesPerRevolution;

  // Calculate distance traveled for both encoders in this interval
  float distanceThisInterval1 = (float(counter1) / pulsesPerRevolution) * wheelCircumference;
  float distanceThisInterval2 = (float(counter2) / pulsesPerRevolution) * wheelCircumference;

  // Add interval distances to total distances
  totalDistance1 += distanceThisInterval1;
  totalDistance2 += distanceThisInterval2;

  // Print results for encoder 1
  Serial.print("Encoder 1 - RPM: ");
  Serial.print(rpm1);
  Serial.print("\t Distance this interval: ");
  Serial.print(distanceThisInterval1);
  Serial.print(" cm\t Total distance: ");
  Serial.print(totalDistance1);
  Serial.println(" cm");

  // Print results for encoder 2
  Serial.print("Encoder 2 - RPM: ");
  Serial.print(rpm2);
  Serial.print("\t Distance this interval: ");
  Serial.print(distanceThisInterval2);
  Serial.print(" cm\t Total distance: ");
  Serial.print(totalDistance2);
  Serial.println(" cm");

  resetSampling();
}

void resetSampling() {
  counter1 = 0;
  counter2 = 0;
}

void move750() {
  if (totalDistance1 >= 750) {
    stop();
    forwardCompleted = true;
    Serial.println("Reached 750 cm, stopping forward motion.");
    delay(1000);  // Small delay before next movement
    right();      // Start right turn
  }
}

void move400() {
  if (totalDistance2 >= 400) {
    stop();
    rightCompleted = true;
    Serial.println("Reached 400 cm, stopping right turn.");
  }
}

boolean timeInterval() {
  unsigned long currentTime = millis();

  // Check if the interval has elapsed
  if (currentTime - lastTime >= interval) {
    lastTime = currentTime;  // Reset lastTime to the current timestamp
    return true;
  }
  return false;
}

void forward() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 1);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 1);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("Moving forward");
}

void stop() {
  analogWrite(FL_pwm, 0);
  analogWrite(FR_pwm, 0);
  analogWrite(RR_pwm, 0);
  analogWrite(RL_pwm, 0);
  Serial.println("Stopped");
}

void right() {
  digitalWrite(FL_dir, 1);
  analogWrite(FL_pwm, motorSpeed);
  digitalWrite(FR_dir, 0);
  analogWrite(FR_pwm, motorSpeed);
  digitalWrite(RR_dir, 1);
  analogWrite(RR_pwm, motorSpeed);
  digitalWrite(RL_dir, 0);
  analogWrite(RL_pwm, motorSpeed);
  Serial.println("Turning right");
}
