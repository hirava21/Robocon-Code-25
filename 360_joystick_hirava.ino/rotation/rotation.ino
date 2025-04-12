int PWM1 = 8, DIR1 = 9;
int PWM2 = 10, DIR2 = 11;
int PWM3 = 12, DIR3 = 13;
int PWM4 = 6, DIR4 = 7;

float old_target_angle = 0.0;
const float max_ang_s = 180.0; // Max angular speed (deg/sec)

void setup() {
    Serial.begin(115200); // Higher baud rate for better accuracy
    Serial3.begin(9600);  // Communication with ESP32

    pinMode(PWM1, OUTPUT); pinMode(DIR1, OUTPUT);
    pinMode(PWM2, OUTPUT); pinMode(DIR2, OUTPUT);
    pinMode(PWM3, OUTPUT); pinMode(DIR3, OUTPUT);
    pinMode(PWM4, OUTPUT); pinMode(DIR4, OUTPUT);
}

void loop() {
    js();  // Process joystick input continuously
}
float calculateTargetAngle(int throttle, int brake, float max_angular_speed) {
    static float target_angle = 0.0;
    static unsigned long last_update_time = 0;

    unsigned long current_time = millis();
    float dt = (current_time - last_update_time) / 1000.0;

    if (dt > 0.0) { // Prevent divide-by-zero errors
        last_update_time = current_time;

        // Compute rotation speed (ω)
        float omega = ((throttle - brake) / 1024.0) * max_angular_speed;

        // Update target angle
        target_angle = fmod((target_angle + (omega * dt)), 360.0);

        // Keep target_angle within 0–360 range
        if (target_angle < 0) {
            target_angle += 360.0;
        }
    }

    return target_angle;
}
void js() {
    if (Serial3.available() > 0) {
        String data = Serial3.readStringUntil('\n');

        // stp 1 JoyX, JoyY, Throttle, Brake
        int delimiter1 = data.indexOf(',');
        int delimiter2 = data.indexOf(',', delimiter1 + 1);
        int delimiter3 = data.indexOf(',', delimiter2 + 1);

        if (delimiter1 != -1 && delimiter3 != -1) {
            int JoyX = data.substring(0, delimiter1).toInt();
            int JoyY = data.substring(delimiter1 + 1, delimiter2).toInt();
            int Throttle = data.substring(delimiter2 + 1, delimiter3).toInt();
            int Brake = data.substring(delimiter3 + 1).toInt();
            
            float target_angle = calculateTargetAngle(Throttle, Brake, max_ang_s);

            Serial.print("Target Angle: ");
            Serial.println(target_angle);
            // step 2 cals angularS
            int R = Throttle - Brake;
            float ang_s = (R / 1023.0) * max_ang_s;

            // update target ang
            static unsigned long last_update_time = millis();
            unsigned long current_time = millis();
            float delta_t = (current_time - last_update_time) / 1000.0; // Convert to seconds
            last_update_time = current_time;

            float new_tarAn = target_angle + (ang_s * delta_t);
            target_angle = new_tarAn; //update target anglr

            // step 3 normalization 
            float xnor = JoyX / 512.0;
            float ynor = JoyY / 512.0;
            float rnor = ang_s / max_ang_s;

            //  motor formula for dir
            float M1 = xnor + ynor + rnor;
            float M2 = ynor - xnor - rnor;
            float M3 = ynor - xnor + rnor;
            float M4 = ynor + xnor - rnor;

            // find max absolute value
            float max_val = max(max(abs(M1), abs(M2)), max(abs(M3), abs(M4)));

            // nrmalize if max_val > 1
            if (max_val > 1) {
                M1 /= max_val;
                M2 /= max_val;
                M3 /= max_val;
                M4 /= max_val;
            }

            // convert to PWM
            int Pwm1 = M1 * 255;
            int Pwm2 = M2 * 255;
            int Pwm3 = M3 * 255;
            int Pwm4 = M4 * 255;

  
            digitalWrite(DIR1, Pwm1 > 0);
            digitalWrite(DIR2, Pwm2 > 0);
            digitalWrite(DIR3, Pwm3 > 0);
            digitalWrite(DIR4, Pwm4 > 0);

            analogWrite(PWM1, abs(Pwm1));
            analogWrite(PWM2, abs(Pwm2));
            analogWrite(PWM3, abs(Pwm3));
            analogWrite(PWM4, abs(Pwm4));

            Serial.print("Target Angle: ");
            Serial.println(new_tarAn);
            Serial.print("PWM1: "); Serial.print(Pwm1);
            Serial.print(" PWM2: "); Serial.print(Pwm2);
            Serial.print(" PWM3: "); Serial.print(Pwm3);
            Serial.print(" PWM4: "); Serial.println(Pwm4);
        }
    }
}