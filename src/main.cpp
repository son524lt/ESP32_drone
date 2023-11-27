#include <main.h>

void setup() {
  Serial.begin(250000);
  init_stuff();
  calculate_from_accel();
  gyro_pitch = accel_pitch;
  gyro_roll = accel_roll;
  timer = micros();
}
void loop() {
  calculate_from_accel();
  calculate_from_gyro();
  updateRollPitch();
  timer = micros();
  delayMicroseconds(100);
}

void task2(void *paramter) {
  for (;;) {
    Serial.print(gyro_pitch);
    Serial.print("\t");
    Serial.print(gyro_roll);
    Serial.print("\t");
    Serial.print(gyro_x);
    Serial.print("\t");
    Serial.print(gyro_y);
    Serial.print("\t");
    Serial.println();
    delay(1);
  }
}