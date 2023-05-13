#include <Arduino.h>
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>
#include <Wire.h>

#ifndef _AXISHANDLER_H_
#define _AXISHANDLER_H_

class axisHandler
{
private:
    MPU6050 imu;
    int16_t ax, ay, az, gx, gy, gz;
    
public:
    axisHandler();
    void getBaseRollPitch(double &roll, double &pitch);
    void calculateRollPitch(double &roll, double &pitch);
};


#endif