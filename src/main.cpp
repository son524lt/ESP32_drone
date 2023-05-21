#include <main.h>

// WIFI MAC ADDRESS OF THE BOARD: 48:E7:29:C4:35:4C
int receivedData[4];
uint8_t senderMacAddress[] = {0x48, 0xE7, 0x29, 0xC4, 0x11, 0xD0};
BL_ESC rFL(FLe,300);
BL_ESC bFR(FRi,260);
BL_ESC rBR(BRi,1300);
BL_ESC bBL(BLe,300);

bool running = false;
uint16_t baseSpeed = 0;
uint8_t baseStage = 0;
float rollError=0, pitchError=0, yawError=0;
float expectedRoll=0, expectedPitch=0, expectedYaw=0;
MPU6050 imu;
int16_t ax, ay, az;
float gx, gy, gz, roll, pitch, yaw;
PIDcontroller rollPID(7,0,0.01);
PIDcontroller pitchPID(7,0,0.01);
PIDcontroller yawPID(0,0,0);
SimpleKalmanFilter kf[5] = {
  SimpleKalmanFilter(2, 2, 1),
  SimpleKalmanFilter(2, 2, 1),
  SimpleKalmanFilter(1, 1, 1),
  SimpleKalmanFilter(1, 1, 0.1),
  SimpleKalmanFilter(1, 1, 0.1)
};

SimpleKalmanFilter signals[4] = {
  SimpleKalmanFilter(1, 1, 0.1),
  SimpleKalmanFilter(1, 1, 0.1),
  SimpleKalmanFilter(1, 1, 0.1),
  SimpleKalmanFilter(1, 1, 0.1)
};

void dataReceived(uint8_t* address, uint8_t* data, uint8_t len, signed int rssi, bool broadcast) {
  receivedData[0]=int(data[0])-10;
  receivedData[0] = (int)signals[0].updateEstimate(receivedData[0]);
  baseStage = receivedData[0];
  baseSpeed = receivedData[0] * 300;
  running = (baseSpeed > 0);
  receivedData[1]=int(data[1])-10;
  receivedData[1] = (int)signals[1].updateEstimate(receivedData[1]);
  expectedYaw=receivedData[1];
  receivedData[2]=int(data[2])-10;
  receivedData[2] = (int)signals[2].updateEstimate(receivedData[2]);
  expectedPitch=receivedData[2]*5;
  receivedData[3]=int(data[3])-10;
  receivedData[3] = (int)signals[3].updateEstimate(receivedData[3]);
  expectedRoll=receivedData[3]*5;
}

void setup() {
  pinMode(16,OUTPUT);
  Wire.begin(); 
  Serial.begin(9600);
  Serial.println();
  imu.initialize();
  digitalWrite(16,HIGH);
  delay(500);
  digitalWrite(16,LOW);
  rFL.Stop();
  bFR.Stop();
  rBR.Stop();
  bBL.Stop();
  imu.CalibrateGyro();
  // Setting up P2P Wifi connection
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect(false, true);
  quickEspNow.onDataRcvd(dataReceived);
  quickEspNow.begin();
  digitalWrite(16,HIGH);
  delay(500);
  digitalWrite(16,LOW);
  pitch = atan2(ay, az) * 180.0 / M_PI;
  roll = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
}
unsigned long prev = micros();
void loop() {
  imu.getAcceleration(&ax, &ay, &az);
  float pitch_acc = atan2(ay, az) * 180.0 / M_PI;
  float roll_acc = atan2(-ax, sqrt(ay * ay + az * az)) * 180.0 / M_PI;
  // roll_acc = kf[0].updateEstimate(roll_acc);
  // pitch_acc = kf[1].updateEstimate(pitch_acc);
  imu.getRotation(&ax, &ay, &az);
  unsigned long current_time = micros();
  gx = kf[2].updateEstimate(ax/gyroScale);
  gy = kf[3].updateEstimate(ay/gyroScale);
  gz = kf[4].updateEstimate(az/gyroScale);
  // gx = ax/gyroScale;
  // gy = ay/gyroScale;
  // gz = az/gyroScale;
  yaw = gz;
  float roll_gyro = gx*(current_time-prev)/1000000.0 + roll;
  float pitch_gyro = gy*(current_time-prev)/1000000.0 + pitch;
  roll = alpha * roll_gyro + (1 - alpha) * roll_acc;
  pitch = alpha * pitch_gyro + (1 - alpha) * pitch_acc;
  
  rollError = roll-expectedRoll;
  pitchError = pitch-expectedPitch;
  yawError = yaw-expectedYaw;
  if (rollError<2&&rollError>-2) rollError=0;
  if (pitchError<2&&pitchError>-2) pitchError=0;
  if (yawError<0.5&&yawError>-0.5) yawError=0;
  rollPID.calculateOutput(rollError, baseStage);
  pitchPID.calculateOutput(pitchError, baseStage);
  yawPID.calculateOutput(yawError, baseStage);
  // running = 0;
  bBL.setSpeed(baseSpeed+yawPID.output-rollPID.output+pitchPID.output);
  bFR.setSpeed(baseSpeed+yawPID.output+rollPID.output-pitchPID.output);
  rBR.setSpeed(baseSpeed-yawPID.output+rollPID.output+pitchPID.output);
  rFL.setSpeed(baseSpeed-yawPID.output-rollPID.output-pitchPID.output);
  Serial.print(roll);
  Serial.print("\t");
  Serial.print(pitch);
  Serial.print("\t");
  Serial.print(yaw);
  Serial.print("\t");
  Serial.print(rollError);
  Serial.print("\t");
  Serial.print(pitchError);
  Serial.print("\t");
  Serial.print(yawError);
  Serial.print("\t");
  Serial.print(bBL.speed);
  Serial.print("\t");
  Serial.print(bFR.speed);
  Serial.print("\t");
  Serial.print(rBR.speed);
  Serial.print("\t");
  Serial.print(rFL.speed);
  Serial.print("\t");
  Serial.print(rollPID.output);
  Serial.print("\t");
  Serial.print(pitchPID.output);
  Serial.print("\t");
  Serial.print(yawPID.output);
  Serial.print("\t");
  Serial.print(baseSpeed);
  Serial.print("\t");
  Serial.println();
  if (running) {bBL.Run(); bFR.Run(); rBR.Run(); rFL.Run();} else {bBL.Stop(); bFR.Stop(); rBR.Stop(); rFL.Stop();}
  delay(50);
  prev = current_time;
}