#include "trans.h"

// Constructor
Trans::Trans() : old_target_ang(0) {}

// Calculate angular speed
float Trans::ang_speed(float throttle, float brake) {
    float R = throttle - brake;
    return (R / 1023.0) * max_ang_speed;
}

// Update target angle
float Trans::target_angle(float old_target_ang, float ang_speed, float deltaT) {
    float temp = old_target_ang + (ang_speed * deltaT);
    float updated_ang = ((int)temp % 360 + 360) % 360;
    
    this->old_target_ang = updated_ang;  // Store updated angle
    return updated_ang;
}

// Getter method for old_target_ang
float Trans::getTargetAngle() {
    return old_target_ang;
}

// Setter method for old_target_ang
void Trans::setTargetAngle(float newAngle) {
    old_target_ang = newAngle;
}

// X, Y, and angular speed in -1 and 1
NormValues Trans::normal(float X, float Y, float ang_speed) {
    NormValues norm;
    norm.Xnorm = X / 512.0;
    norm.Ynorm = -Y / 512.0;
    norm.Rnorm = ang_speed / max_ang_speed; 
    return norm;
}
