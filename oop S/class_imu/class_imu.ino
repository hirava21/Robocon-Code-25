#include <Wire.h>
#include <SparkFun_BNO080_Arduino_Library.h>

class OmniRobot {
private:
    // Motor pins structure
    struct MotorPins {
        const int pwm;
        const int dir;
        MotorPins(int p, int d) : pwm(p), dir(d) {}
    };

    // Control parameters
    struct ControlParams {
        const int motorDeadzone = 15;
        const int maxMotorSpeed = 70;
        const float yawThreshold = 0.25;
        const float maxCorrection = 35;
        const float Kp = 2.0;
        const float Ki = 0.0;
        const float Kd = 0.4;
    };

    // Motor configuration
    const MotorPins motors[4] = {
        MotorPins(8, 9),   // Motor 1
        MotorPins(10, 11), // Motor 2
        MotorPins(12, 13), // Motor 3
        MotorPins(6, 7)    // Motor 4
    };

    // IMU and control state
    BNO080 imu;
    ControlParams params;
    bool imuInitialized = false;
    
    // Motion tracking state
    float previousError = 0;
    float previousYaw = 0;
    float startingYaw = 0;
    float yawOffset = 0;
    unsigned long lastPIDTime = 0;
    bool isMovingStraight = false;
    int straightLineCounter = 0;
    
    // Constants
    static const int PID_INTERVAL = 5;
    static const int STABLE_COUNT = 10;

    // Private methods for IMU handling
    float getRawYaw() {
        if (!imuInitialized || !imu.dataAvailable()) {
            return previousYaw;
        }

        float qw = imu.getQuatReal();
        float qx = imu.getQuatI();
        float qy = imu.getQuatJ();
        float qz = imu.getQuatK();
        
        float yaw = atan2(2.0 * (qw * qz + qx * qy), 1.0 - 2.0 * (qy * qy + qz * qz)) * 180.0 / PI;
        previousYaw = yaw;
        return yaw;
    }

    float getNormalizedYaw() {
        float currentYaw = getRawYaw() - yawOffset;
        
        // Normalize to -180 to 180
        while (currentYaw > 180) currentYaw -= 360;
        while (currentYaw < -180) currentYaw += 360;
        
        return currentYaw;
    }

    float calculateCorrection() {
        unsigned long currentTime = millis();
        if (currentTime - lastPIDTime < PID_INTERVAL) {
            return 0;
        }
        
        float dt = (currentTime - lastPIDTime) / 1000.0;
        lastPIDTime = currentTime;
        
        float currentYaw = getNormalizedYaw();
        float error = currentYaw - startingYaw;
        
        // Normalize error
        while (error > 180) error -= 360;
        while (error < -180) error += 360;
        
        // Calculate derivative
        float derivative = (error - previousError) / dt;
        previousError = error;
        
        // Aggressive correction calculation
        float correction = (params.Kp * error) + (params.Kd * derivative);
        
        // Apply non-linear scaling for small errors
        if (abs(error) < params.yawThreshold) {
            correction *= (abs(error) / params.yawThreshold);
        }
        
        return constrain(correction, -params.maxCorrection, params.maxCorrection);
    }

    void setMotorSpeed(int motorIndex, int speed) {
        analogWrite(motors[motorIndex].pwm, abs(speed));
        digitalWrite(motors[motorIndex].dir, speed > 0 ? HIGH : LOW);
    }

public:
    OmniRobot() {}

    bool begin() {
        // Initialize serial communications
        Serial.begin(115200);
        Serial1.begin(9600);
        
        // Initialize motor pins
        for (int i = 0; i < 4; i++) {
            pinMode(motors[i].pwm, OUTPUT);
            pinMode(motors[i].dir, OUTPUT);
        }
        
        // Initialize IMU
        Wire.begin();
        if (imu.begin(0x4A)) {
            imu.enableRotationVector(200);
            delay(50);
            imuInitialized = true;
            calibrateIMU();
            return true;
        }
        return false;
    }

    void calibrateIMU() {
        if (!imuInitialized) return;
        
        float sumYaw = 0;
        int readings = 0;
        
        for (int i = 0; i < 10; i++) {
            if (imu.dataAvailable()) {
                sumYaw += getRawYaw();
                readings++;
            }
            delay(5);
        }
        
        if (readings > 0) {
            yawOffset = sumYaw / readings;
            previousYaw = 0;
        }
    }

    void move(int speedX, int speedY, int rotation) {
        // Detect straight-line motion
        bool movingStraight = (abs(speedY) > params.motorDeadzone && 
                             abs(speedX) < params.motorDeadzone && 
                             rotation == 0);
        
        // State management for straight-line motion
        if (movingStraight) {
            if (!isMovingStraight) {
                startingYaw = getNormalizedYaw();
                straightLineCounter = 0;
                isMovingStraight = true;
            }
            straightLineCounter++;
        } else {
            isMovingStraight = false;
            straightLineCounter = 0;
        }
        
        // Apply enhanced deadzone
        speedX = abs(speedX) < params.motorDeadzone ? 0 : speedX;
        speedY = abs(speedY) < params.motorDeadzone ? 0 : speedY;
        rotation = abs(rotation) < params.motorDeadzone ? 0 : rotation;
        
        // Calculate correction
        float correction = 0;
        if (isMovingStraight && straightLineCounter >= STABLE_COUNT) {
            correction = calculateCorrection();
            if (speedY != 0) {
                correction *= (float)abs(speedY) / params.maxMotorSpeed;
            }
        }
        
        // Calculate motor speeds with correction
        float leftCorrection = correction;
        float rightCorrection = -correction;
        
        int motorSpeeds[4] = {
            speedY + speedX + rotation + leftCorrection,  // M1
            speedY - speedX - rotation + rightCorrection, // M2
            speedY - speedX + rotation + leftCorrection,  // M3
            speedY + speedX - rotation + rightCorrection  // M4
        };
        
        // Normalize speeds
        int maxSpeed = 0;
        for (int speed : motorSpeeds) {
            maxSpeed = max(maxSpeed, abs(speed));
        }
        
        if (maxSpeed > params.maxMotorSpeed) {
            float scale = (float)params.maxMotorSpeed / maxSpeed;
            for (int i = 0; i < 4; i++) {
                motorSpeeds[i] *= scale;
            }
        }
        
        // Apply motor speeds
        for (int i = 0; i < 4; i++) {
            setMotorSpeed(i, motorSpeeds[i]);
        }
    }

    void update() {
        if (Serial1.available() > 0) {
            String data = Serial1.readStringUntil('\n');
            int indices[2] = {
                data.indexOf(','),
                data.indexOf(',', data.indexOf(',') + 1)
            };

            if (indices[0] != -1 && indices[1] != -1) {
                int axisX = data.substring(0, indices[0]).toInt();
                int axisY = data.substring(indices[0] + 1, indices[1]).toInt();
                int axisRX = data.substring(indices[1] + 1).toInt();
                
                // Map joystick values to motor speeds
                int speedX = map(axisX, -512, 512, -params.maxMotorSpeed, params.maxMotorSpeed);
                int speedY = map(axisY, -512, 512, -params.maxMotorSpeed, params.maxMotorSpeed);
                int rotation = map(axisRX, -512, 512, -params.maxMotorSpeed/2, params.maxMotorSpeed/2);
                
                move(speedX, speedY, rotation);
            }
        }
    }
};

// Main program
OmniRobot robot;

void setup() {
    if (!robot.begin()) {
        Serial.println("Failed to initialize robot!");
    }
}

void loop() {
    robot.update();
}