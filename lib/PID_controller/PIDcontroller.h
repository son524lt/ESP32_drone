#ifndef _PIDCONTROLLER_H_
#define _PIDCONTROLLER_H_
#include <Arduino.h>
class PIDcontroller
{
private:
    double kP, kI, kD,  P=0, I=0, D=0, error=0, last_error=0;
    int lowerLimit=-10000, upperLimit=10000;
public:
    double output=0;
    PIDcontroller(double kP, double kI, double kD);
    void calculateOutput(double error, uint8_t baseStage);
    void setLimit(unsigned lowerLimit, unsigned upperLimit);
};

#endif