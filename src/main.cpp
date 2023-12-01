#include <main.h>

motorController motor[4] = {
  motorController(LF_READ, LF_CHANNEL, LF_TRIG_PWM, LF_MIN_PWM, LF_MAX_PWM, &pwm),
  motorController(RF_READ, RF_CHANNEL, RF_TRIG_PWM, RF_MIN_PWM, RF_MAX_PWM, &pwm),
  motorController(RB_READ, RB_CHANNEL, RB_TRIG_PWM, RB_MIN_PWM, RB_MAX_PWM, &pwm),
  motorController(LB_READ, LB_CHANNEL, LB_TRIG_PWM, LB_MIN_PWM, LB_MAX_PWM, &pwm)
};

void setup() {
  Serial.begin(1000000);
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
uint i;
void loop() {
  for (i = 0; i <= 1e6; i+=10) {
    calculate_from_accel();
    calculate_from_gyro();
    updateRollPitch();
    timer = micros();
    motor[LF].writePermillionage(i);
    motor[RF].writePermillionage(i);
    motor[RB].writePermillionage(i);
    motor[LB].writePermillionage(i);
  }
  while (1);
  // delayMicroseconds(10);
}

void task2(void *paramter) {
  for (;;) {
    Serial.print(i);
    Serial.print("\t");
    motor[LF].updateSpeed();
    Serial.print(motor[LF].pwmVal);
    Serial.print("\t");
    Serial.print(motor[LF].motorSpeed);
    Serial.print("\t");
    motor[RF].updateSpeed();
    Serial.print(motor[RF].pwmVal);
    Serial.print("\t");
    Serial.print(motor[RF].motorSpeed);
    Serial.print("\t");
    motor[RB].updateSpeed();
    Serial.print(motor[RB].pwmVal);
    Serial.print("\t");
    Serial.print(motor[RB].motorSpeed);
    Serial.print("\t");
    motor[LB].updateSpeed();
    Serial.print(motor[LB].pwmVal);
    Serial.print("\t");
    Serial.print(motor[LB].motorSpeed);
    Serial.print("\t");
    Serial.print(roll);
    Serial.print("\t");
    Serial.print(pitch);
    Serial.print("\t");
    Serial.println();
    delay(1);
    if (i > 1e6) {break;}
  }
}