#include "FOC.h"
#include <math.h>

void FOC::compute(float X_norm, float Y_norm, float yaw, float &Vx, float &Vy) {
    float rad = yaw * M_PI / 180.0; // Convert degrees to radians
    Vx = X_norm * cos(rad) - Y_norm * sin(rad);
    Vy = X_norm * sin(rad) + Y_norm * cos(rad);
}
