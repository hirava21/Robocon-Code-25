#ifndef PS_H   // Update include guard
#define PS_H

#include <Arduino.h>

class PS {
private:
    int _axisY, _axisX, _dpad, _throttle, _brake;  

public:
    PS();  // Constructor
    void update();

    int getAxisY();
    int getAxisX();
    int getDpad();
    int getThrottle();
    int getBrake();

    bool update();
};

extern PS ps;  // Keep the global instance

#endif
