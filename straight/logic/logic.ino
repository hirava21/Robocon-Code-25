float calculateTargetAngle(int joystick_cw, int joystick_ccw, float max_angular_speed) {
    static float target_angle = 0.0;         // Stores target angle
    static unsigned long last_update_time = 0; // Stores last update timestamp

    unsigned long current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0; // Time difference in seconds

    if (dt > 0.0) { // Prevent divide-by-zero errors
        last_update_time = current_time; // Update timestamp

        // Compute rotation speed (ω)
        float omega = ((joystick_cw - joystick_ccw) / 1024.0) * max_angular_speed;

        // Update target angle
        target_angle = fmod((target_angle + (omega * dt)), 360.0);

        // Keep target_angle within 0–360 range
        if (target_angle < 0) {
            target_angle += 360.0;
        }
    }

    return target_angle; // Return the updated target angle
}

void setup() {
    Serial.begin(115200);
    pinMode(A0, INPUT);
    pinMode(A1, INPUT);
}

void loop() {
    int joystick_cw = analogRead(A0);   // Read clockwise joystick value (0-1024)
    int joystick_ccw = analogRead(A1);  // Read counterclockwise joystick value (0-1024)
    
    float max_speed = 180.0; // Set max rotation speed (degrees/sec)

    float target_angle = calculateTargetAngle(joystick_cw, joystick_ccw, max_speed);

    Serial.print("Target Angle: ");
    Serial.println(target_angle);

    // Other parallel tasks can run here
}


