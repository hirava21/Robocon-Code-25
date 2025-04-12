#ifndef ROT_H
#define ROT_H

class Rot {  // Renamed from rot to Rot
private:
    float ang_speed;     // Angular speed in deg/s
    float target_angle;  // Target yaw angle
    float max_speed;     // Max rotation speed in deg/s
    float limit_val;
public:
    Rot();  // Constructor

    // Function to calculate angular speed
    float ang_spd(float throttle, float brake);

    // Function to calculate target yaw angle
    float ang(float delta_t);
};

#endif
