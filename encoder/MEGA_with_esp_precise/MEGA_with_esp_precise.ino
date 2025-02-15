// Motor control pins
int PWM1 = 8, FL_dir = 9;
int PWM2 = 10, FR_dir = 11;
int PWM3 = 12, RR_dir = 13;
int PWM4 = 6, RL_dir = 7;
int motorSpeed = 0;

// Encoder pin definitions
#define X_ENCODER_A 2  
#define X_ENCODER_B 3
#define Y_ENCODER_A 19
#define Y_ENCODER_B 18 

// Encoder and movement tracking variables
const float PULSES_PER_REVOLUTION = 600.0;
const float WHEEL_DIAMETER = 10.0;  // cm
const float CM_PER_PULSE = (PI * WHEEL_DIAMETER) / PULSES_PER_REVOLUTION;

volatile long x_encoder_count = 0;
volatile long y_encoder_count = 0;
float x_position = 0.0;
float y_position = 0.0;
float x_calibration = 1.0;
float y_calibration = 1.0;
bool forwardCompleted = false;
bool rightCompleted = false;

void setup() {
    Serial.begin(9600);  // Serial Monitor
    Serial3.begin(9600); // UART communication with ESP32

    // Motor Pins Setup
    pinMode(PWM1, OUTPUT);
    pinMode(FL_dir, OUTPUT);
    pinMode(PWM2, OUTPUT);
    pinMode(FR_dir, OUTPUT);
    pinMode(PWM3, OUTPUT);
    pinMode(RR_dir, OUTPUT);
    pinMode(PWM4, OUTPUT);
    pinMode(RL_dir, OUTPUT);

    // Encoder Pins Setup
    pinMode(X_ENCODER_A, INPUT_PULLUP);
    pinMode(X_ENCODER_B, INPUT_PULLUP);
    pinMode(Y_ENCODER_A, INPUT_PULLUP);
    pinMode(Y_ENCODER_B, INPUT_PULLUP);

    // Attach interrupts
    attachInterrupt(digitalPinToInterrupt(X_ENCODER_A), x_encoder_isr, CHANGE);
    attachInterrupt(digitalPinToInterrupt(Y_ENCODER_A), y_encoder_isr, CHANGE);
    
    Serial.println("System Initialized.");
}

void loop() {
    js();
}

// Function to process joystick, buttons, and execute encoder-based tasks
void js() {
    if (Serial3.available() > 0) {
        String data = Serial3.readStringUntil('\n'); // Read joystick, D-pad, throttle, and brake data

        // Extract values from the received string
        int delimiter1 = data.indexOf(',');
        int delimiter2 = data.indexOf(',', delimiter1 + 1);
        int delimiter3 = data.indexOf(',', delimiter2 + 1);
        int delimiter4 = data.indexOf(',', delimiter3 + 1);
        int delimiter5 = data.indexOf(',', delimiter4 + 1);

        if (delimiter1 != -1 && delimiter5 != -1) {
            int axisY = data.substring(0, delimiter1).toInt();
            int axisX = data.substring(delimiter1 + 1, delimiter2).toInt();
            int dpad = data.substring(delimiter2 + 1, delimiter3).toInt();
            int throttle = data.substring(delimiter3 + 1, delimiter4).toInt();
            int brake = data.substring(delimiter4 + 1, delimiter5).toInt();
            int button = data.substring(delimiter5 + 1).toInt(); // Extract button value

            if (button == 2) {
                executeEncoderTask();  // Run encoder movement task
            } else {
                // Standard joystick-based movement
                if (axisY <= -25) {
                    motorSpeed = map(axisY, -25, -512, 0, 50);
                    forward();
                    Serial.println("Forward");
                } else if (axisY >= 25) {
                    motorSpeed = map(axisY, 25, 512, 0, 50);
                    reverse();
                    Serial.println("Reverse");
                }
                if (axisX <= -25) {
                    motorSpeed = map(axisX, -25, -512, 0, 50);
                    left();
                    Serial.println("Left");
                } else if (axisX >= 25) {
                    motorSpeed = map(axisX, 25, 512, 0, 50);
                    right();
                    Serial.println("Right");
                }
                if (throttle > 0) {
                    motorSpeed = map(throttle, 0, 1024, 0, 50);
                    clockwise();
                    Serial.println("Clockwise");
                }
                if (brake > 0) {
                    motorSpeed = map(brake, 0, 1024, 0, 50);
                    anticlockwise();
                    Serial.println("Anticlockwise");
                }
                if (axisY > -25 && axisY < 25 && axisX > -25 && axisX < 25 && throttle == 0 && brake == 0) {
                    stop();
                    Serial.println("STOP");
                }
            }
        }
    }
}

// Encoder interrupt service routines
void x_encoder_isr() {
    int a = digitalRead(X_ENCODER_A);
    int b = digitalRead(X_ENCODER_B);
    x_encoder_count += (a == b) ? 1 : -1;
}

void y_encoder_isr() {
    int a = digitalRead(Y_ENCODER_A);
    int b = digitalRead(Y_ENCODER_B);
    y_encoder_count += (a == b) ? 1 : -1;
}

// Function to execute encoder-based movement (Move forward and then right 30 cm each)
void executeEncoderTask() {
    Serial.println("Encoder task started!");

    // Reset encoder counts
    x_encoder_count = y_encoder_count = 0;
    x_position = y_position = 0.0;
    forwardCompleted = rightCompleted = false;

    // Move forward 30 cm
    while (!forwardCompleted) {
        x_position = x_encoder_count * CM_PER_PULSE * x_calibration;
        y_position = y_encoder_count * CM_PER_PULSE * y_calibration;
        moveForward30();
    }

    // Reset encoder counts before moving right
    x_encoder_count = y_encoder_count = 0;
    x_position = y_position = 0.0;
    forwardCompleted = false;  // Reset for future movements

    // Move right 30 cm
    while (!rightCompleted) {
        x_position = x_encoder_count * CM_PER_PULSE * x_calibration;
        y_position = y_encoder_count * CM_PER_PULSE * y_calibration;
        moveRight30();
    }

    Serial.println("Encoder task completed!");
    stop();
}

// Move forward until reaching 30 cm
void moveForward30() {
    if (abs(x_position) >= 30 || abs(y_position) >= 30) {
        stop();
        forwardCompleted = true;
        Serial.println("Reached 30 cm, stopping forward motion.");
    } else {
        forward();
    }
}

// Move right until reaching 30 cm
void moveRight30() {
    if (abs(x_position) >= 30 || abs(y_position) >= 30) {
        stop();
        rightCompleted = true;
        Serial.println("Completed 30 cm right move.");
    } else {
        right();
    }
}
void forward() {
    digitalWrite(FL_dir, 1); analogWrite(PWM1, motorSpeed);
    digitalWrite(FR_dir, 1); analogWrite(PWM2, motorSpeed);
    digitalWrite(RR_dir, 1); analogWrite(PWM3, motorSpeed);
    digitalWrite(RL_dir, 1); analogWrite(PWM4, motorSpeed);
}
void right() {
    digitalWrite(FL_dir, 1); analogWrite(PWM1, motorSpeed);
    digitalWrite(FR_dir, 0); analogWrite(PWM2, motorSpeed);
    digitalWrite(RR_dir, 1); analogWrite(PWM3, motorSpeed);
    digitalWrite(RL_dir, 0); analogWrite(PWM4, motorSpeed);
}
void stop() {
    analogWrite(PWM1, 0); analogWrite(PWM2, 0);
    analogWrite(PWM3, 0); analogWrite(PWM4, 0);
    Serial.println("Stopped");
}
void clockwise() {
    digitalWrite(FL_dir, 1); analogWrite(FL_pwm, motorSpeed);
    digitalWrite(FR_dir, 0); analogWrite(FR_pwm, motorSpeed);
    digitalWrite(RR_dir, 0); analogWrite(RR_pwm, motorSpeed);
    digitalWrite(RL_dir, 1); analogWrite(RL_pwm, motorSpeed);
    Serial.println("Rotating Clockwise");
}
void anticlockwise() {
    digitalWrite(FL_dir, 0); analogWrite(FL_pwm, motorSpeed);
    digitalWrite(FR_dir, 1); analogWrite(FR_pwm, motorSpeed);
    digitalWrite(RR_dir, 1); analogWrite(RR_pwm, motorSpeed);
    digitalWrite(RL_dir, 0); analogWrite(RL_pwm, motorSpeed);
    Serial.println("Rotating Anti-clockwise");
}
void reverse() {
    digitalWrite(FL_dir, 0); analogWrite(PWM1, motorSpeed);
    digitalWrite(FR_dir, 0); analogWrite(PWM2, motorSpeed);
    digitalWrite(RR_dir, 0); analogWrite(PWM3, motorSpeed);
    digitalWrite(RL_dir, 0); analogWrite(PWM4, motorSpeed);
}
void left(){
    digitalWrite(FL_dir, 0); analogWrite(PWM1, motorSpeed);
    digitalWrite(FR_dir, 1); analogWrite(PWM2, motorSpeed);
    digitalWrite(RR_dir, 0); analogWrite(PWM3, motorSpeed);
    digitalWrite(RL_dir, 1); analogWrite(PWM4, motorSpeed);
}