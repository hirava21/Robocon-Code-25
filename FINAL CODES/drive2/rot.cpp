#include "rot.h"

// Constructor
Rot::Rot() { 
    ang_speed = 0;
    target_angle = 0;
    max_speed = 180.0; // Max rotation speed in deg/s
    limit_val = 0;  // 
}

// Angular speed
float Rot::ang_spd(float throttle, float brake) {
    ang_speed = ((throttle - brake) / 1024.0) * max_speed;
    return ang_speed;  
}

// Target yaw angle
float Rot::ang(float delta_t) {
    target_angle += ang_speed * delta_t;

    target_angle = ((int)target_angle % 360 + 360) % 360;
    return target_angle;   
}


//Random Testing for -180 to 180 conversion logic
//     // // Normalize to -180 to 180 degrees
//     while (target_angle > 180) 
//     {target_angle = ((int)target_angle % 180 + 180) % 180 * X;//test1
//     }
//     while (target_angle < 180) target_angle = ((int)target_angle % 180 + 180) % 180;

    // if(target_angle > 180) 
    // {target_angle = -target_angle/2;//test2
    // }
    // else if(target_angle < 180) 
    // {target_angle = target_angle/2;
    // }
    // else target_angle = target_angle;
    // target_angle = ((int)target_angle % 180 + 180) % 180;//test 3 output: 0-180
