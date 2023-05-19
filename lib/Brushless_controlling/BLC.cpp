#include <BLC.h>

BL_ESC::BL_ESC(char pin_number, int _threshold) {
    threshold = (int)map(_threshold, 0, 10000, 1000, 2000);
    pin_number;
    ESC.attach(pin_number);
}

void BL_ESC::setSpeed(int _speed, bool _run) {
    if (_speed < 0) _speed = 0;
    if (_speed > 10000) _speed = 10000;
    speed = _speed;
    if (_run) Run();
}

void BL_ESC::Run() {
    ESC.writeMicroseconds((int)map(speed,0,10000,threshold,2000));
}

void BL_ESC::Stop() {
    speed = 0;
    ESC.writeMicroseconds(1000);
}