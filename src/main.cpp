#include <main.h>
BL_ESC redFrontLeft(FLe);
BL_ESC blackFrontRight(FRi);
BL_ESC blackBackLeft(BLe);
BL_ESC redBackRight(BRi);
// BluetoothSerial SerialBT;
double roll, pitch, gyro_x, gyro_y, gyro_z;
PIDcontroller rollPID(10,0.0001,1);
PIDcontroller pitchPID(10,0.0001,1);
PIDcontroller yawPID(0,0,0);
axisHandler axis;
void stopBLC() {
  redFrontLeft.Stop();
  blackFrontRight.Stop();
  blackBackLeft.Stop();
  redBackRight.Stop();
}
void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println();
  stopBLC();
  delay(1000);
  // SerialBT.begin("Drone");
  Serial.println("Wait me...");
  axis.getBaseRollPitch(roll, pitch);
  Serial.println("Got roll pitch!");
  delay(2000);
}
bool running = false;
unsigned baseSpeed = 0;
double rollError=0, pitchError=0, yawError=0;
unsigned long previousTime = 0;
double sum_x = 0, sum_y = 0;
unsigned total = 0;
void loop() {
  // if (SerialBT.available()) {
  //   char cmd = SerialBT.read();
  //   switch (cmd) {
  //   case '0':
  //     stopBLC();
  //     running = false;
  //     baseSpeed = 0;
  //     break;
  //   case '1':
  //     running = true;
  //     baseSpeed = 2000;
  //     break;
  //   case '2':
  //     running = true;
  //     baseSpeed = 2500;
  //     break;
  //   case '3':
  //     running = true;
  //     baseSpeed = 3000;
  //     break;
  //   case '4':
  //     running = true;
  //     baseSpeed = 3500;
  //     break;
  //   case '5':
  //     running = true;
  //     baseSpeed = 4000;
  //     break;
  //   }
  // }
  // imu.getRotation(&gx, &gy, &gz);
  // unsigned long currentTime = micros();
  // unsigned long elapsedTime = currentTime - previousTime;
  // gx = kfG[X].updateEstimate(gx);
  // gy = kfG[Y].updateEstimate(gy);
  // gz = kfG[Z].updateEstimate(gz);
  // if (elapsedTime >= 100) {
  //   gyro_x = (double)gx/131.0;
  //   gyro_y = (double)gy/131.0;
  //   gyro_z = (double)gz/131.0;
  //   roll += gyro_x * (elapsedTime/1000000.0);
  //   pitch += gyro_y * (elapsedTime/1000000.0);
  //   previousTime = currentTime;
  // }
  // float yaw = (float)kf[GZ].updateEstimate(gz)/131;
  // roll = kf[ROLL].updateEstimate(roll);
  // pitch = kf[PITCH].updateEstimate(pitch);
  // if (roll>-1&&roll<1) {rollError = 0;} else rollError = roll;
  // if (pitch>-1&&pitch<1) {pitchError = 0;} else pitchError = pitch;
  // if (yaw>0&&yaw<1) {yawError = 0;} else yawError = yaw-0.5;
  // rollPID.calculateOutput(rollError);
  // pitchPID.calculateOutput(pitchError);
  // yawPID.calculateOutput(yawError);
  blackBackLeft.setSpeed(baseSpeed+yawPID.output+rollPID.output-pitchPID.output);
  blackFrontRight.setSpeed(baseSpeed+yawPID.output-rollPID.output+pitchPID.output);
  redBackRight.setSpeed(baseSpeed-yawPID.output+rollPID.output+pitchPID.output);
  redFrontLeft.setSpeed(baseSpeed-yawPID.output-rollPID.output-pitchPID.output);
  running = 0;
  if (running) {
    blackBackLeft.Run();
    blackFrontRight.Run();
    redBackRight.Run();
    redFrontLeft.Run();
  } else stopBLC();
}

