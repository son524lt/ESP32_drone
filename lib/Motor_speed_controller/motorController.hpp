#include <Arduino.h>
#include <Adafruit_PWMServoDriver.h>
class motorController {
private:
    bool pState, nState, res;
    unsigned long onTime, now, deltaTime = 1000000;
    uint8_t readPin, outChan;
    uint16_t initPWM, minPWM, maxPWM;
    Adafruit_PWMServoDriver *BLDC;
    bool pulledToHIGH();
    double nonIntMap(double x, double in_min, double in_max, double out_min, double out_max);
public:
    uint16_t pwmVal = 0;
    double motorSpeed = 0;
    motorController(uint8_t sensorPin, uint8_t PWMchannel, uint16_t initPWM, uint16_t minPWM, uint16_t maxPWM, Adafruit_PWMServoDriver *PWMcontroller);
    void updateSpeed();
    // void writeSpeed(double speed);
    void init();
    // void setPID(double kP, double kI, double kD, double scale);
    void writePower(double power);
};