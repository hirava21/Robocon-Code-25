#ifndef TRANS_H
#define TRANS_H

struct NormValues {
    float Xnorm;
    float Ynorm;
    float Rnorm;
};

class Trans {
public:
    Trans();
    
    float ang_speed(float throttle, float brake);
    float target_angle(float old_target_ang, float ang_speed, float deltaT);
    NormValues normal(float X, float Y, float ang_speed);

    void setTargetAngle(float newAngle);  // Setter
    float getTargetAngle();  // Getter

private:
    float old_target_ang = 0;  // Store last target angle
    float max_ang_speed = 180.0;
};

#endif