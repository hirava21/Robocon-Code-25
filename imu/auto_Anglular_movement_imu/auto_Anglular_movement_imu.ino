#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

BNO080 myIMU;

// Motor control pins
int FL_pwm = 8, FR_pwm = 10, RL_pwm = 6, RR_pwm = 12;
int FL_dir = 9, FR_dir = 11, RL_dir = 7, RR_dir = 13;
int motorSpeed = 25;

// IMU variables
float initialYaw = 0, adjustedYaw = 0;
float targetedYaw = 180;  // Target yaw in degrees
bool hasReachedTarget = false;  // Flag to track target reach

void setup() {
    Serial.begin(115200);
    while (!Serial);

    Serial.println("Starting setup...");
    Wire.begin();

    if (!myIMU.begin(0x4A)) {
        Serial.println("BNO085 not detected. Check wiring or I2C address.");
        while (1);
    }

    Serial.println("BNO085 detected!");
    myIMU.enableRotationVector(100);  // Update every 100ms
    delay(1000);

    // Read initial yaw multiple times for stability
    float sumYaw = 0;
    int validReadings = 0;

    for (int i = 0; i < 50; i++) {  // Reduced to 50 iterations for efficiency
        if (myIMU.dataAvailable()) {
            sumYaw += getYaw();
            validReadings++;
        }
        delay(10);
    }

    if (validReadings > 0) {
        initialYaw = sumYaw / validReadings;
        Serial.print("Initial Yaw: ");
        Serial.println(initialYaw, 2);
    } else {
        Serial.println("Failed to get initial yaw.");
    }

    Serial.println("Setup complete, starting loop...");

    // Set motor pins as outputs
    pinMode(FL_dir, OUTPUT); pinMode(FL_pwm, OUTPUT);
    pinMode(FR_dir, OUTPUT); pinMode(FR_pwm, OUTPUT);
    pinMode(RR_dir, OUTPUT); pinMode(RR_pwm, OUTPUT);
    pinMode(RL_dir, OUTPUT); pinMode(RL_pwm, OUTPUT);
}

void loop() {
    if (myIMU.dataAvailable()) {
        float currentYaw = getYaw();
        adjustedYaw = currentYaw - initialYaw;

        // Normalize to 0-360 range
        if (adjustedYaw < 0) adjustedYaw += 360;
        if (adjustedYaw > 360) adjustedYaw -= 360;

        Serial.print("Adjusted Yaw: ");
        Serial.println(adjustedYaw, 2);

        if (!hasReachedTarget) {
            moveToTarget(adjustedYaw);
        }
    }

    delay(100);  // Reduced delay for smoother updates
}

void moveToTarget(float adjustedYaw) {
    float error = targetedYaw - adjustedYaw;

    // Normalize error to range (-180, 180) for shortest rotation direction
    if (error > 180) error -= 360;
    if (error < -180) error += 360;

    // Serial.print("Error: ");
    // Serial.println(error, 2);

    // Stop when within ±2° of target
    if (abs(error) <= 1) {
        stop();
        Serial.println("Targeted angle reached!");
        hasReachedTarget = true;
        return;
    }

    // Rotate in the shortest direction
    if (targetedYaw > 180) {
        clockwise();
    } else {
        anticlockwise();
    }
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

void stop() {
    analogWrite(FL_pwm, 0);
    analogWrite(FR_pwm, 0);
    analogWrite(RR_pwm, 0);
    analogWrite(RL_pwm, 0);
    Serial.println("Stop");
}

// Function to get Yaw from BNO085
float getYaw() {
    float qw = myIMU.getQuatReal();
    float qx = myIMU.getQuatI();
    float qy = myIMU.getQuatJ();
    float qz = myIMU.getQuatK();

    float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz));
    yaw = yaw * 180.0 / PI;  // Convert to degrees

    return yaw;
}
