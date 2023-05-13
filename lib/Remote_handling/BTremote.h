#ifndef _BTREMOTE_H_
#define _BTREMOTE_H_

#include <BLC.h>
#include <BluetoothSerial.h>
#include <PIDcontroller.h>

#define redFrontLeft    0
#define blackFrontRight 1
#define redBackRight    2
#define blackBackLeft   3

class BTremote {
private:
    unsigned baseSpeed=0;
    BluetoothSerial SerialBT;
    BL_ESC * motor;
public:
    BTremote(const char * BTname, BL_ESC * motor);
    void handleSignal(bool &running);
    void runBLC(uint8_t motor_number=4);
    void stopBLC(uint8_t motor_number=4);
    void setSpeedBLC(uint8_t motor_number, unsigned speed);
    void calculateSpeed(PIDcontroller &rollPID, PIDcontroller &pitchPID, PIDcontroller &yawPID);
};

#endif