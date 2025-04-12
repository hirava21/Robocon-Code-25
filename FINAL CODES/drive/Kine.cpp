#include "Kine.h"
#include <Arduino.h>

void Kine::compute(float Vx, float Vy, float Omega, float L, float W,
                   float &V_FL, float &V_FR, float &V_BL, float &V_BR) {
    float wheel_factor = (L + W); // Distance factor for rotation

    // Compute individual wheel speeds
    V_FL = Vy - Vx + Omega * wheel_factor;
    V_FR = Vy + Vx - Omega * wheel_factor;
    V_BL = Vy + Vx + Omega * wheel_factor;
    V_BR = Vy - Vx - Omega * wheel_factor;

    // Find max speed among all wheels
    float max_speed = max(max(abs(V_FL), abs(V_FR)), max(abs(V_BL), abs(V_BR)));

    // Normalize if max speed exceeds 1
    if (max_speed > 1.0) {
        V_FL /= max_speed;
        V_FR /= max_speed;
        V_BL /= max_speed;
        V_BR /= max_speed;
    }
}
