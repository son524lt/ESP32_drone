#ifndef _MAIN_H_
#define _MAIN_H_
#include <Arduino.h>
#include <BLC.h>
#include <PIDcontroller.h>
#include <Wire.h>
#include <MPU6050.h>
#include <SimpleKalmanFilter.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <QuickEspNow.h>

#define X 0
#define Y 1
#define Z 2

#define FLe 19
#define FRi 18
#define BRi 32
#define BLe 33

#define accelScale 16384.0
#define gyroScale 130.0

#define alpha 0.9

#endif

