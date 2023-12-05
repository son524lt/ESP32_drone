#include <balancingController.hpp>

balancingController::balancingController(float _kP, float _kI, float _kD) {
    kP = _kP; kI = _kI, kD = _kD;
}

void balancingController::calculateOutput(double val, double desired) {
    error = desired - val;
    deltaTime = micros() - timer;
    integral += error*deltaTime/1e6; // integral by seconds
    timer = micros();
    output = kP * (int)error + kI * integral + kD * (error-lastError)*1e6/deltaTime;
    lastError = error;
}

void balancingController::reset() {
    integral = 0;
}