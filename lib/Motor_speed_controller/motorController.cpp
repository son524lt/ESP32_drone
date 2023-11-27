#include <motorController.h>

motorController::motorController(uint8_t sensorPin, uint8_t PWMchannel, uint16_t _initPWM, uint16_t _minPWM, uint16_t _maxPWM, Adafruit_PWMServoDriver *PWMcontroller) {
    pinMode(sensorPin, INPUT);
    onTime = micros();
    pState = digitalRead(readPin);
    readPin = sensorPin;
    outChan = PWMchannel;
    initPWM = _initPWM; minPWM = _minPWM, maxPWM = _maxPWM;
    BLDC = PWMcontroller;
}

bool motorController::pulledToHIGH() {
    nState = digitalRead(readPin);
    res = nState && !pState;
    pState = nState;
    return res;
}

void motorController::updateSpeed() {
    now = micros();
    if (now - onTime > deltaTime) {
        deltaTime = now - onTime;
    }
    if (pulledToHIGH()) {
        deltaTime = now - onTime;
        onTime = micros();
    }
    motorSpeed = 60000000.0/deltaTime;
    if (motorSpeed < 200) motorSpeed = 0;
}

void motorController::writeSpeed(double speed) {
    if (speed <= 0) {
        BLDC->setPWM(outChan,0,minPWM);
    }
    BLDC->setPWM(outChan,0,motorPower);
}

void motorController::triggerESC() {
    BLDC->setPWM(outChan,0,initPWM);
}

void motorController::setPID(double _kP, double _kI, double _kD) {
    kP = _kP;
    kI = _kI;
    kD = _kD;
}