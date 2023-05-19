#include <PIDcontroller.h>

PIDcontroller::PIDcontroller(double _kP, double _kI, double _kD) {
    kP = _kP;
    kI = _kI;
    kD = _kD;
}

void PIDcontroller::calculateOutput(double _error, uint8_t _baseStage) {
    error = _error;
    P = error;
    I += error;
    D = error - last_error;
    output = (kP*P + kI*I + kD*D)*_baseStage;
    if (output < lowerLimit) output = lowerLimit;
    if (output > upperLimit) output = upperLimit;
    last_error = error; 
};

void PIDcontroller::setLimit(unsigned _lowerLimit, unsigned _upperLimit) {
    lowerLimit = _lowerLimit;
    upperLimit = _upperLimit;
}