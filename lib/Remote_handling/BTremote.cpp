#include <BTremote.h>

BTremote::BTremote(const char * BTname, BL_ESC * _motor) {
  SerialBT.begin(BTname);
  motor = _motor;
}

void BTremote::handleSignal(bool &running) {
    if (SerialBT.available()) {
    char cmd = SerialBT.read();
    switch (cmd) {
    case '0':
      stopBLC();
      running = false;
      baseSpeed = 0;
      break;
    case '1':
      running = true;
      baseSpeed = 2000;
      break;
    case '2':
      running = true;
      baseSpeed = 2500;
      break;
    case '3':
      running = true;
      baseSpeed = 3000;
      break;
    case '4':
      running = true;
      baseSpeed = 3500;
      break;
    case '5':
      running = true;
      baseSpeed = 4000;
      break;
    }
  }
}

void BTremote::runBLC(uint8_t motor_number) {
  if (motor_number < 4) motor[motor_number].Run();
  else for (size_t i = 0; i < 4; i++) motor[i].Run();
}

void BTremote::stopBLC(uint8_t motor_number) {
  if (motor_number < 4) motor[motor_number].Stop();
  else for (size_t i = 0; i < 4; i++) motor[i].Stop();
}

void BTremote::setSpeedBLC(uint8_t motor_number, unsigned _speed) {
  motor[motor_number].setSpeed(_speed);
}

void BTremote::calculateSpeed(PIDcontroller &rollPID, PIDcontroller &pitchPID, PIDcontroller &yawPID) {
  motor[blackBackLeft].setSpeed(baseSpeed+yawPID.output-rollPID.output+pitchPID.output);
  motor[blackFrontRight].setSpeed(baseSpeed+yawPID.output+rollPID.output-pitchPID.output);
  motor[redBackRight].setSpeed(baseSpeed-yawPID.output+rollPID.output+pitchPID.output);
  motor[redFrontLeft].setSpeed(baseSpeed-yawPID.output-rollPID.output-pitchPID.output);
}