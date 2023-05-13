#include <main.h>

BL_ESC MOTOR[4] = {BL_ESC(FLe),BL_ESC(FRi),BL_ESC(BRi),BL_ESC(BLe)};
axisHandler axis;
double roll, pitch;
PIDcontroller rollPID(10,0.0001,1);
PIDcontroller pitchPID(10,0.0001,1);
PIDcontroller yawPID(0,0,0);
// BluetoothSerial SerialBT;
BTremote remote("Drone", MOTOR);

void setup() {
  Wire.begin();
  Serial.begin(9600);
  Serial.println();
  remote.stopBLC();
  // delay(1000);
  // Serial.println("Wait me...");
  // axis.getBaseRollPitch(roll, pitch);
  // Serial.print(roll);
  // Serial.print("\t");
  // Serial.println(pitch);
  // Serial.println("Got roll pitch!");
  delay(2000);
  // for (size_t i = 0; i < 4; i++)
  // {
  //   MOTOR[i].setSpeed(100);
  //   MOTOR[i].Run();
  //   delay(200);
  //   MOTOR[i].Stop();
  // }
  while (1);
}
bool running = false;
unsigned baseSpeed = 0;
double rollError=0, pitchError=0, yawError=0;
unsigned long previousTime = 0;
void loop() {
  if (roll>-1&&roll<1) {rollError = 0;} else rollError = roll;
  if (pitch>-1&&pitch<1) {pitchError = 0;} else pitchError = pitch;
  rollPID.calculateOutput(rollError);
  pitchPID.calculateOutput(pitchError);
  yawPID.calculateOutput(yawError);
  running = 0;
  remote.handleSignal(running);
  if (running) {
    remote.runBLC();
  }
}