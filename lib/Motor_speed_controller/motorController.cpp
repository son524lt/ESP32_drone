#include <motorController.hpp>

motorController::motorController(uint8_t sensorPin, uint8_t PWMchannel, uint16_t _initPWM, uint16_t _minPWM, uint16_t _maxPWM, Adafruit_PWMServoDriver *PWMcontroller) {
    onTime = micros();
    pState = digitalRead(readPin);
    readPin = sensorPin;
    outChan = PWMchannel;
    initPWM = _initPWM; minPWM = _minPWM, maxPWM = _maxPWM;
    BLDC = PWMcontroller;
}

double motorController::nonIntMap(double x, double in_min, double in_max, double out_min, double out_max) {
    const double run = in_max - in_min;
    if(run == 0){
        log_e("map(): Invalid input range, min == max");
        return -1;
    }
    const double rise = out_max - out_min;
    const double delta = x - in_min;
    return (delta * rise) / run + out_min;
}

bool motorController::pulledToHIGH() {
    nState = digitalRead(readPin);
    res = nState && !pState;
    pState = nState;
    return res;
}

void motorController::updateSpeed() {
    now = micros();
    if (pulledToHIGH()) {
        deltaTime = now - onTime;
        onTime = micros();
    } else if (now - onTime > deltaTime) {
        deltaTime = now - onTime;
    }
    motorSpeed = 60000000.0/deltaTime;
    if (motorSpeed < 50) motorSpeed = 0;
}

// void motorController::writeSpeed(double desiredSpeed) {
//     if (desiredSpeed > 0 && desiredSpeed < 4000) desiredSpeed = 4000;
//     if (desiredSpeed > 12000) desiredSpeed = 12000;
//     if (desiredSpeed <= 0) {
//         BLDC->setPWM(outChan,0,initPWM);
//         integral = 0;
//         return;
//     }
//     error = desiredSpeed*speedScale - motorSpeed;
//     integral += (micros()-PIDtimer)*error/1000000;
//     if (kI*integral > maxPWM) integral = maxPWM/kI;
//     derivative = (micros()-PIDtimer)*(preError-error)/1000000;
//     motorPower = minPWM + kP * error + kI * integral + kD * derivative;
//     if (motorPower < minPWM) motorPower = minPWM;
//     if (motorPower > maxPWM) motorPower = maxPWM;
//     PIDtimer = micros();
//     BLDC->setPWM(outChan,0,(motorPower));
//     preError = error;
// }

void motorController::init() {
    pinMode(readPin, INPUT);
    BLDC->setPWM(outChan,0,initPWM);
}

// void motorController::setPID(double _kP, double _kI, double _kD, double scale) {
//     kP = _kP;
//     kI = _kI;
//     kD = _kD;
//     speedScale = scale;
// }

void motorController::writePower(double power) {
    if (power == 0) {
        pwmVal = initPWM;
        BLDC->setPWM(outChan, 0, initPWM);
        return;
    }
    if (power < 0) power = 0;
    if (power > 1e4) power = 1e4;
    pwmVal = nonIntMap(power, 0, 1e4, minPWM, maxPWM);
    BLDC->setPWM(outChan, 0, pwmVal);
}