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
#include <motorController.hpp>
#include <balancingController.hpp>
#include <BluetoothSerial.h>

BluetoothSerial SerBT;

#define accelScale 4096.0
#define gyroScale 65.5

#define SERVO_FREQ 200

#define LF_TRIG_PWM 635
#define RF_TRIG_PWM 640
#define RB_TRIG_PWM 625
#define LB_TRIG_PWM 640

#define LF_MIN_PWM 700
#define RF_MIN_PWM 715
#define RB_MIN_PWM 800
#define LB_MIN_PWM 870

#define LF_MAX_PWM 1735
#define RF_MAX_PWM 1700
#define RB_MAX_PWM 1700
#define LB_MAX_PWM 1745

#define LF_CHANNEL 4
#define RF_CHANNEL 5
#define RB_CHANNEL 6
#define LB_CHANNEL 7

#define LF 0
#define RF 1
#define RB 2
#define LB 3

#define LF_READ 39
#define RF_READ 36
#define RB_READ 35
#define LB_READ 34

#define rollOffset -4.6
#define pitchOffset 0

#define roll_kP 70
#define roll_kI 0
#define roll_kD 3

#define pitch_kP 70
#define pitch_kI 0
#define pitch_kD 3

#define yaw_kP 0
#define yaw_kI 0
#define yaw_kD 0

// Variables
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();
MPU6050 imu;
int16_t ax, ay, az, gx, gy, gz;
float gyro_x, gyro_y, gyro_z, 
desired_pitch, desired_roll, 
accel_pitch, accel_roll, 
accel_x, accel_y, accel_z, 
speed_x = 0, speed_y = 0, speed_z = 0, 
gyro_roll=0, gyro_pitch=0, 
roll, pitch;
unsigned long timer;
float KalmanAngleRoll=0, KalmanUncertaintyAngleRoll=2*2;
float KalmanAnglePitch=0, KalmanUncertaintyAnglePitch=2*2;
float Kalman1DOutput[]={0,0};

motorController motor[4] = {
  motorController(LF_READ, LF_CHANNEL, LF_TRIG_PWM, LF_MIN_PWM, LF_MAX_PWM, &pwm),
  motorController(RF_READ, RF_CHANNEL, RF_TRIG_PWM, RF_MIN_PWM, RF_MAX_PWM, &pwm),
  motorController(RB_READ, RB_CHANNEL, RB_TRIG_PWM, RB_MIN_PWM, RB_MAX_PWM, &pwm),
  motorController(LB_READ, LB_CHANNEL, LB_TRIG_PWM, LB_MIN_PWM, LB_MAX_PWM, &pwm)
};
double motorPow[4] = {0, 0, 0, 0};

balancingController pitchBalancer(pitch_kP, pitch_kI, pitch_kD);
balancingController rollBalancer(roll_kP, roll_kI, roll_kD);
balancingController yawBalancer();

// Functions and Methods
void task2(void *parameter);

void init_stuff() {
    Wire.begin();
    Wire.setClock(400000);
    pwm.begin();
    pwm.setOscillatorFrequency(27000000);
    pwm.setPWMFreq(SERVO_FREQ);
    for (uint8_t i = 0; i < 16; i++) {
        pwm.setPWM(i,0,0);
    }
    imu.initialize();
    imu.CalibrateGyro();
    imu.setDLPFMode(6);
}

void init_task2() {
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
    gyro_pitch = 0.2 * accel_pitch + 0.8 * (gyro_pitch + gyro_x * (micros() - timer)/1000000);
    gyro_roll = 0.2 * accel_roll + 0.8 * (gyro_roll + gyro_y * (micros() - timer)/1000000);
    kalman_1d(KalmanAngleRoll, KalmanUncertaintyAngleRoll, gyro_roll, accel_roll);
    KalmanAngleRoll=Kalman1DOutput[0];
    KalmanUncertaintyAngleRoll=Kalman1DOutput[1];
    kalman_1d(KalmanAnglePitch, KalmanUncertaintyAnglePitch, gyro_pitch,accel_pitch);
    KalmanAnglePitch=Kalman1DOutput[0];
    KalmanUncertaintyAnglePitch=Kalman1DOutput[1];
    roll = KalmanAngleRoll-rollOffset;
    pitch = KalmanAnglePitch-pitchOffset;
}

// Debugging functions
void freeze() {
    while (1);
}

void debugLog(Stream &Seri) {
    Seri.print(motor[LF].motorSpeed);
    Seri.print("\t");
    // Seri.print(motorPow[LF]);
    // Seri.print("\t");
    Seri.print(motor[RF].motorSpeed);
    Seri.print("\t");
    // Seri.print(motorPow[RF]);
    // Seri.print("\t");
    Seri.print(motor[RB].motorSpeed);
    Seri.print("\t");
    // Seri.print(motorPow[RB]);
    // Seri.print("\t");
    Seri.print(motor[LB].motorSpeed);
    Seri.print("\t");
    // Seri.print(motorPow[LB]);
    // Seri.print("\t");
    Seri.print(roll);
    Seri.print("\t");
    Seri.print(pitch);
    Seri.print("\t");
    Seri.println();
}

#endif