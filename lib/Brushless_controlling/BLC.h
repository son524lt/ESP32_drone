#include <Arduino.h>
#include <Servo.h>
#ifndef _BLC_H_
#define _BLC_H_

class BL_ESC {
private:
    uint8_t pin;
    Servo ESC;
public:
    unsigned int speed;
    BL_ESC(char pin_number);
    void Run();
    void setSpeed(unsigned int speed);
    void Stop();
};


#endif