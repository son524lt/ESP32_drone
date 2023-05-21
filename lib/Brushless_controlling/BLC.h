#include <Arduino.h>
#include <Servo.h>
#ifndef _BLC_H_
#define _BLC_H_

class BL_ESC {
private:
    uint8_t pin;
    Servo ESC;
public:
    int speed, threshold=1000;
    BL_ESC(char pin_number, int threshold = 0);
    void Run();
    void setSpeed(int speed, bool run=false);
    void Stop();
};


#endif