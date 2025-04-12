#include "ps.h"
#include "rot.h"
#include "imu.h"
#include "PID.h"
#include "FOC.h"
#include "Kine.h"
#include "Motor.h"

#define KP 1
#define KI 0.00
#define KD 0.0

Rot rotation;  
IMU imu;
PID yawPID(KP, KI, KD);
Motor motor(22, 24, 26, 28, 4, 5, 6, 7);
float target_Yaw = 0;

static unsigned long last_update_time = 0; // Moving this to globally to track time properly
bool dir[4]; 
int pwm[4];
void setup() {
    Serial.begin(115200); // Monitor
    Serial3.begin(9600); // Communication with maybe IMU
    while (!Serial);

    if (!imu.begin()) {
        Serial.println("IMU initialization failed!");
        while (1);
    }  
}

void loop() {
    ps.update();  
    imu.update();

  // Step 1: Get data from remote: (No problem here)
    float throttle = ps.getThrottle();
    float brake = ps.getBrake();
    float joyX = ps.getAxisX();
    float joyY = ps.getAxisY();

    if (abs(joyX) < 5) joyX = 0; //applying the deadzone
if (abs(joyY) < 5) joyY = 0;


    Serial.print("Throttle: "); Serial.print(throttle);
    Serial.print(" | Brake: "); Serial.print(brake);
    Serial.print(" | X: "); Serial.print(joyX);
    Serial.print(" | Y: "); Serial.print(joyY);

  // Step 2: Get Rot speed -> target angle: Got target angle from 0 to 360
    unsigned long current_time = millis();
    float delta_t = (current_time - last_update_time) / 1000.0;
    last_update_time = current_time;  

    // float target_Yaw = rotation.ang(delta_t); // (-180 to 180 degrees)
    // if(target_Yaw> 180) target_Yaw -= 360;
    // int ang_speed = rotation.ang_spd(throttle, brake); // (-180 to 180 deg/sec) 
    // // Serial.print(" | Target Angle: "); Serial.println(target_Yaw);
    
  // Step 3: Get data from imu: Got data from imu (0-360)
    float actual_Yaw = imu.deg(); // (0-360 degrees)
    if (actual_Yaw > 180) actual_Yaw -= 360; // Convert to (-180 to 180)
    // Serial.print(" | imu(degree): "); Serial.print(actual_Yaw);
    
   // Step 4: PID, Got the val of error(-180 to 180) and norm val(-1to1)

    // Errornorm = constrain(Errornorm , -1, 1);
    // Serial.print(" | Error: "); Serial.println(Error);
    // Serial.print(" | Error norm: "); Serial.println(Errornorm);
    float correct_Yaw = yawPID.compute(target_Yaw, actual_Yaw);
    float Error = target_Yaw - correct_Yaw;
    float Errornorm = Error/180;


    // Serial.print(" | Yaw_Final: "); Serial.println(yaw_final);
    
    // Step 5:Angular Summation
    // float yaw_final = target_Yaw + yaw_final;//backup

    // Step 6: Normalization(Class Baad me)(Got X,Y,R val betn -1 to 1)
    float newYaw = target_Yaw - correct_Yaw;
    Serial.print("  | new Yaw ");
    Serial.print(newYaw);
    // float Xnorm = joyX / 512.0;
    // float Ynorm = -joyY / 512.0;
    float Rnorm = newYaw/360 ;

    // Serial.print("NormValues - Xnorm: ");
    // Serial.print(Xnorm);
    // Serial.print(" | Ynorm: ");
    // Serial.print(Ynorm);
    // Serial.print(" | Rnorm: ");
    // Serial.print(Rnorm);

    // Step 7:FOC
    float Vx, Vy;
    // FOC::compute(Xnorm, Ynorm, Rnorm, Vx, Vy); //actual_Yaw for FOC or Rnorm IDK
    // Serial.print("Vx: ");
    // Serial.print(Vx);
    // Serial.print(" | Vy: ");
    // Serial.println(Vy);

    // Step 8: Kinematics
    float M1, M2, M4, M3;
    float L = 0.6, W = 0.6; // Robot dimensions
    // float wheel_factor = (L + W); // Distance factor for rotation
    // Kine::compute(Xnorm, Ynorm, Rnorm, L, W, M1, M2, M4, M3);

      float M1 = + Rnorm;
      float M2 = - Rnorm;
      float M3 = - Rnorm;
      float M4 = + Rnorm;
      
    // Step 9: Motor Control (Implement this)
    // motor.setSpeed(M1, M2, M4, M3);
    float max_speed = max(max(abs(M1), abs(M2)), max(abs(M4), abs(M3)));

    if (max_speed > 1.0) {
      M1 /= max_speed;
      M2 /= max_speed;
      M4 /= max_speed;
      M3 /= max_speed;
      }

   dir[0] = M1 > 0;
    dir[1] = M2 > 0;
    dir[2] = M3 > 0;
    dir[3] = M4 > 0;

    if (M1 == 0 && M2 == 0 && M3 == 0 && M4 == 0) {
    pwm[0] = pwm[1] = pwm[2] = pwm[3] = 0;
} else {
    pwm[0] = abs(M1 * 255);
    pwm[1] = abs(M2 * 255);
    pwm[2] = abs(M3 * 255);
    pwm[3] = abs(M4 * 255);
}


    // Print direction values
    Serial.print("  |Dir: ");
    Serial.print(dir[0]); Serial.print(" ");
    Serial.print(dir[1]); Serial.print(" ");
    Serial.print(dir[2]); Serial.print(" ");
    Serial.print(dir[3]);

    // Print PWM values
    Serial.print("  | PWM: ");
    Serial.print(pwm[0]); Serial.print(" ");
    Serial.print(pwm[1]); Serial.print(" ");
    Serial.print(pwm[2]); Serial.print(" ");
    Serial.println(pwm[3]);

    // Motor control
    motor.drive(dir[0], dir[1], dir[2], dir[3], pwm[0], pwm[1], pwm[2], pwm[3]);
}