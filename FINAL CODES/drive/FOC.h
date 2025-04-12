#ifndef FOC_H
#define FOC_H

class FOC {
public:
    static void compute(float X_norm, float Y_norm, float yaw, float &Vx, float &Vy);
};

#endif
