
#include <SPI.h>
#include <mcp2515.h>

#define GearRatio 2.7
#define radius_inch 9
#define Circumfrence 2 * 3.14 * 0.0000254 * radius_inch

struct can_frame canMsg;
MCP2515 mcp2515(10);
float Curr=0, InstVol=0, SOC=0, HighTemp=0, LowTemp=0, InputSupp=0;
int DtcFlag1=0, DtcFlag2=0;

float RPM=0, Speed=0, DutyCycle=0, InputVol=0, mc_AC_Curr=0, mc_DC_Curr=0;
float ControllerTemp=0, MotorTemp=0, Id=0, Iq=0;
int FaultCode=0, throttle=0, throttle_temp=0, limits_1=0, limits_2=0; 
float prev_time = millis(), curr_time = millis(), Distance = 0;
String message, bms1, bms2, mc1, mc2, bps;
String message_found = "0 ";
int sensorval = 0;
void setup() {
  Serial.begin(9600);
  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();
}

void loop() { 
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    message_found = "1 ";
    switch (canMsg.can_id){

      case 2147485360: 
        Curr = (float)(canMsg.data[0]*256 + canMsg.data[1])/10;
        InstVol = (float)(canMsg.data[2]*256 + canMsg.data[3])/10;
        SOC = (float)canMsg.data[4]/2;
        HighTemp = canMsg.data[5];
        LowTemp = canMsg.data[6];
        break;

      case 2147485361:
        DtcFlag1 = canMsg.data[0]*256 + canMsg.data[1];
        DtcFlag2 = canMsg.data[2]*256 + canMsg.data[3];
        InputSupp = (float)(canMsg.data[4]*256 + canMsg.data[5])/10;
        break;
      
      case 2147483712:
        curr_time = millis();
        RPM = (canMsg.data[0]*16777216 + canMsg.data[1]*65536 + canMsg.data[2]*256 + canMsg.data[3])/10;
        Speed = RPM * Circumfrence * 60/ GearRatio;
        Distance += Speed*(curr_time - prev_time)/1000;
        prev_time = curr_time;
        DutyCycle = (canMsg.data[4]*256 + canMsg.data[5])/10;
        InputVol = (canMsg.data[6]*256 + canMsg.data[7]);
        break;

      case 2147483968:
        mc_AC_Curr = (canMsg.data[0]*256 + canMsg.data[1])/10;
        mc_DC_Curr = (canMsg.data[2]*256 + canMsg.data[3])/10;
        throttle = throttle_temp;
        break;
      
      case 2147484224:
        ControllerTemp = (float)(canMsg.data[0]*256 + canMsg.data[1])/10;
        MotorTemp = (float)(canMsg.data[2]*256 + canMsg.data[3])/10;
        FaultCode = canMsg.data[4];
        break;

      case 2147484480:
        Id = (canMsg.data[0]*16777216 + canMsg.data[1]*65536 + canMsg.data[2]*256 + canMsg.data[3])/100;
        Iq = (canMsg.data[4]*16777216 + canMsg.data[5]*65536 + canMsg.data[6]*256 + canMsg.data[7])/100;
        break;
        
      case 2147484736:
        throttle_temp=canMsg.data[0];
        limits_1=canMsg.data[4];
        limits_2=canMsg.data[5]; 
        break; 
      
      default:
        Serial.println("ID not recognised");
    }
  }
  sensorval = analogRead(A0);
  bms1 = String(Curr) + " " + String(InstVol) + " " + String(SOC) + " " + String(HighTemp) + " " + String(LowTemp) + " ";
  bms2 = String(DtcFlag1) + " " + String(DtcFlag2) + " " + String(InputSupp) + " " ;
  mc1 = String(RPM) + " " + String(Speed) + " " +  String(Distance) + " " + String(DutyCycle) + " " + String(InputVol) + " " + String(mc_AC_Curr) +
   " " + String(mc_DC_Curr) + " " ;
  mc2 = String(ControllerTemp) + " " + String(MotorTemp) + " " + String(FaultCode) + " " + String(Id) + " " +String(Iq) +
   " " + String(throttle) + " " + String(limits_1) + " " + String(limits_2) + " " ;
  bps = String(sensorval);
  message = message_found + bms1 + bms2 + mc1 + mc2 + bps ; 
  Serial.println(message);
  //delay(500);
  message_found = "0 ";
}
