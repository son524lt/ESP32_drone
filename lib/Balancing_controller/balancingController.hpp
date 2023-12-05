#include <Arduino.h>

class balancingController
{
private:
    float kP, kI, kD;
    int error = 0, lastError = 0;
    long unsigned deltaTime, timer;
    long double integral = 0;
public:
    long double output = 0;
    balancingController(float kP, float kI, float kD);
    void calculateOutput(double value, double desiredValue);
    void reset();
};

