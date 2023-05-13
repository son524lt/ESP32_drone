#include <Arduino.h>
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>

#ifndef _AXISHANDLER_H_
#define _AXISHANDLER_H_

class axisHandler
{
private:
    MPU6050 imu;
    int16_t ax, ay, az, gx, gy, gz;
    SimpleKalmanFilter kfA[3] = {
        SimpleKalmanFilter(1, 1, 0.001),
        SimpleKalmanFilter(1, 1, 0.001),
        SimpleKalmanFilter(1, 1, 0.001)
    };
    SimpleKalmanFilter kfG[3] = {
        SimpleKalmanFilter(1, 1, 0.001),
        SimpleKalmanFilter(1, 1, 0.001),
        SimpleKalmanFilter(1, 1, 0.001)
    };
public:
    axisHandler();
    void getBaseRollPitch(double &roll, double &pitch);
    void calculateRollPitch(double &roll, double &pitch);
};


#endif