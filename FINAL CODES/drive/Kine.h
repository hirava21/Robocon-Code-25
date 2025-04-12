#ifndef KINE_H
#define KINE_H

class Kine {
public:
    static void compute(float Vx, float Vy, float Omega, float L, float W, 
                        float &V_FL, float &V_FR, float &V_BL, float &V_BR);
};

#endif
