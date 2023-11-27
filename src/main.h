#ifndef _MAIN_H_
#define _MAIN_H_
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include <esp_wifi.h>
#include <WiFi.h>
#include <QuickEspNow.h>
#include <Adafruit_PWMServoDriver.h>
#include <SPI.h>
#include <motorController.h>

#define encoderPinLF 36

#define accelScale 4096.0
#define gyroScale 65.5

#define SERVO_FREQ 65
#define MIN_SERVO_PULSE 85
#define MAX_SERVO_PULSE 529

#define LF_TRIG_PWM 210
#define RF_TRIG_PWM 215
#define RB_TRIG_PWM 200
#define LB_TRIG_PWM 210

#define LF_MIN_PWM 210
#define RF_MIN_PWM 380
#define RB_MIN_PWM 355
#define LB_MIN_PWM 290

#define LF_MAX_PWM 590
#define RF_MAX_PWM 415
#define RB_MAX_PWM 395
#define LB_MAX_PWM 515

#define LF_CHANNEL 4
#define RF_CHANNEL 5
#define RB_CHANNEL 6
#define LB_CHANNEL 7

#define LF_READ 39
#define RF_READ 36
#define RB_READ 35
#define LB_READ 34

// #define rollOffset
// #define pitchOffset

// Variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MPU6050 imu;
int16_t ax, ay, az, gx, gy, gz;
float gyro_x, gyro_y, gyro_z, 
desired_pitch, desired_roll, 
accel_pitch, accel_roll, 
accel_x, accel_y, accel_z, 
speed_x = 0, speed_y = 0, speed_z = 0, 
gyro_roll, gyro_pitch;
unsigned long timer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

// Functions and Methods
void task2(void *parameter);

void init_stuff() {
    Wire.begin();
    Wire.setClock(400000);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    imu.initialize();
    imu.CalibrateGyro();
    imu.setDLPFMode(6);
    xTaskCreatePinnedToCore(
    task2,      // Function to implement the task
    "Task2",    // Name of the task
    10000,      // Stack size (words)
    NULL,       // Parameter to pass to the function
    1,          // Priority (0 is the lowest priority)
    NULL,       // Task handle
    0           // Core to run the task on (0 for core 0)
  );
}
void calculate_from_accel() {
    imu.getAcceleration(&ax, &ay, &az);
    accel_pitch = atan2(ay, az) * RAD_TO_DEG;
    accel_roll = atan2(-ax, sqrt(ay * ay + az * az)) * RAD_TO_DEG;
    accel_x = ax / accelScale;
    accel_y = ay / accelScale;
    accel_z = az / accelScale;
}
void calculate_from_gyro() {
    imu.getRotation(&gx, &gy, &gz);
    gyro_x = gx/gyroScale;
    gyro_y = gy/gyroScale;
    gyro_z = gz/gyroScale;
}
void resetCalibration() {
    imu.setXAccelOffset(0);
    imu.setYAccelOffset(0);
    imu.setZAccelOffset(0);
    imu.setXGyroOffset(0);
    imu.setYGyroOffset(0);
    imu.setZGyroOffset(0);
}

void kalman_1d(float KalmanState, float KalmanUncertainty, float KalmanInput, float KalmanMeasurement) {
    KalmanState=KalmanState+0.004*KalmanInput;
    KalmanUncertainty=KalmanUncertainty + 0.004 * 0.004 * 4 * 4;
    float KalmanGain=KalmanUncertainty * 1/(1*KalmanUncertainty + 3 * 3);
    KalmanState=KalmanState+KalmanGain * (KalmanMeasurement-KalmanState);
    KalmanUncertainty=(1-KalmanGain) * KalmanUncertainty;
    Kalman1DOutput[0]=KalmanState; 
    Kalman1DOutput[1]=KalmanUncertainty;
}

void updateRollPitch() {
    gyro_pitch = 0.5 * accel_pitch + 0.5 * (gyro_pitch + gyro_x * (micros() - timer)/1000000);
    gyro_roll = 0.5 * accel_roll + 0.5 * (gyro_roll + gyro_y * (micros() - timer)/1000000);
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, gyro_y, gyro_roll);
    KalmanAngleRoll=Kalman1DOutput[0]; 
    KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, gyro_x, gyro_pitch);
    KalmanAnglePitch=Kalman1DOutput[0]; 
    KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
}

#endif

