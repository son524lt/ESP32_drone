#include <main.h>

// Z axis joystick (left joystick)
#define V_LEFT 32   // vertical axis
#define H_LEFT 33   // horizontal axis
// XY plane (right joystick)
#define V_RIGHT 39  // vertical axis
#define H_RIGHT 36  // horizontal axis
// WIFI MAC ADDRESS OF THE BOARD: 48:E7:29:C4:11:D0
uint8_t receiverMacAddress[] = {0x48, 0xE7, 0x29, 0xC4, 0x35, 0x4C};
uint8_t dataToSend[4] = {0,0,0,0};
void setup() {
  Serial.begin(9600);
  WiFi.mode(WIFI_MODE_STA);
  WiFi.disconnect (false, true);
  quickEspNow.begin();

}
int max_vl=3400,
    max_hl=3400, 
    max_vr=3400,
    max_hr=3400, 
    min_vl = 400, 
    min_hl = 360,
    min_vr = 400, 
    min_hr = 400;
    
// int max_vl=0,
//     max_hl=0, 
//     max_vr=0,
//     max_hr=0, 
//     min_vl = 4000, 
//     min_hl = 4000,
//     min_vr = 4000, 
//     min_hr = 4000;
void loop() {
  int vleft = analogRead(V_LEFT), 
      hleft = analogRead(H_LEFT),
      vright = analogRead(V_RIGHT), 
      hright = analogRead(H_RIGHT);
  if (vright>max_vr) max_vr = vright;
  if (vright<min_vr) min_vr = vright;
  if (hright>max_hr) max_hr = hright;
  if (hright<min_hr) min_hr = hright;
  if (hleft>max_hl) max_hl = hleft;
  if (hleft<min_hl) min_hl = hleft;
  if (vleft>max_vl) max_vl = vleft;
  if (vleft<min_vl) min_vl = vleft;
  vleft = round(map(vleft,min_vl,max_vl,20,-3));
  hleft = round(map(hleft,min_hl,max_hl,5,-6));
  vright = round(map(vright,min_vr,max_vr,9,-10));
  hright = round(map(hright,min_hr,max_hr,9,-10));
  if (vleft<0) vleft = 0;
  if (hleft<-5) hleft = -5;
  if (vright<-9) vright = -9;
  if (hright<-9) hright = -9;
  dataToSend[0] = vleft+10;
  dataToSend[1] = hleft+10;
  dataToSend[2] = vright+10;
  dataToSend[3] = hright+10;
  quickEspNow.send(receiverMacAddress, dataToSend, 4);
}