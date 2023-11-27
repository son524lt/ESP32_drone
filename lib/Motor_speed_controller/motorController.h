#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>

#define MIN_PULSE 85
#define MAX_PULSE 529

class motorController {
private:
    bool pState, nState, res;
    unsigned long onTime, now, deltaTime = 1000000;
    uint8_t readPin, outChan;
    uint16_t motorPower = 0, initPWM, minPWM, maxPWM;
    double kP, kI, kD;
    Adafruit_PWMServoDriver *BLDC;
    bool pulledToHIGH();
public:
    double motorSpeed = 0;
    motorController(uint8_t sensorPin, uint8_t PWMchannel, uint16_t initPWM, uint16_t minPWM, uint16_t maxPWM, Adafruit_PWMServoDriver *PWMcontroller);
    void updateSpeed();
    void writeSpeed(double speed);
    void triggerESC();
    void setPID(double kP, double kI, double kD);
};