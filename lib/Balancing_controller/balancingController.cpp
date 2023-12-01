#include <balancingController.hpp>

balancingController::balancingController(float _kP, float _kI, float _kD) {
    kP = _kP; kI = _kI, kD = _kD;
}

void balancingController::calculateOutput(int val, int desired) {
    error = desired - val;
    deltaTime = micros() - timer;
    integral += error*deltaTime/1e6; // integral by seconds
    timer = micros();
    output = kP * error + kI * integral + kD * (lastError-error)/deltaTime;
    lastError = error;
}

void balancingController::reset() {
    integral = 0;
}