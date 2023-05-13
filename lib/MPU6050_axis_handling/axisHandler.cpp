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
    ax=sum[0]/100;ay=sum[1]/100;az=sum[2]/100;
    roll = atan2(ay, az) * 180.0 / M_PI;
    pitch = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
}

void axisHandler::calculateRollPitch(double &roll, double &pitch) {

    

    if (roll>180) roll-=360;
    if (roll<-180) roll+=360;
    if (pitch>180) pitch-=360;
    if (pitch<-180) pitch+=360;
}