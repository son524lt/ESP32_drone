#include <axisHandler.h>


axisHandler::axisHandler() {
    imu.initialize();
}

void axisHandler::getBaseRollPitch(double &roll, double &pitch) {
    int sum[3]={0,0,0};
    for (size_t i = 0; i < 100; i++){
        imu.getAcceleration(&ax, &ay, &az);
        sum[0]+=ax;
        sum[1]+=ay;
        sum[2]+=az;
    }
    
}

void axisHandler::calculateRollPitch(double &roll, double &pitch) {

    

    if (roll>180) roll-=360;
    if (roll<-180) roll+=360;
    if (pitch>180) pitch-=360;
    if (pitch<-180) pitch+=360;
}