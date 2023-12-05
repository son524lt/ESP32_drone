#include <main.hpp>

void setup() {
  Serial.begin(1000000);
  SerBT.begin("Drone");
  init_stuff();
  calculate_from_accel();
  gyro_pitch = accel_pitch;
  gyro_roll = accel_roll;
  timer = micros();
  motor[LF].init();
  motor[RF].init();
  motor[RB].init();
  motor[LB].init();
  delay(7000);
  init_task2();
}

double basePow = 4000;
void loop() {
  motor[LF].updateSpeed();
  motor[RF].updateSpeed();
  motor[RB].updateSpeed();
  motor[LB].updateSpeed();
  calculate_from_accel();
  calculate_from_gyro();
  updateRollPitch();
  timer = micros();
  pitchBalancer.calculateOutput(pitch, 0);
  rollBalancer.calculateOutput(roll, 0);
  Serial.println();
  motorPow[LF] = basePow + (+ pitchBalancer.output + rollBalancer.output)/2.0;
  motorPow[RF] = basePow + (+ pitchBalancer.output - rollBalancer.output)/2.0;
  motorPow[RB] = basePow + (- pitchBalancer.output - rollBalancer.output)/2.0;
  motorPow[LB] = basePow + (- pitchBalancer.output + rollBalancer.output)/2.0;
  motor[LF].writePower(motorPow[LF]);
  motor[RF].writePower(motorPow[RF]);
  motor[RB].writePower(motorPow[RB]);
  motor[LB].writePower(motorPow[LB]);
}

void task2(void *paramter) {
  for (;;) {
    debugLog(SerBT);
    debugLog(Serial);
    delay(1);
    // if (i > 1e6) {break;}
  }
}