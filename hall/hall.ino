#include <SPI.h>
#include <mcp2515.h>
#include <ros.h>
#include <std_msgs/Float32.h>

#define GearRatio 2.4
#define radius_inch 9
#define Correction 1//1.43/1.2
#define Circumfrence 2 * 3.14 * 0.0254 * radius_inch * Correction

struct can_frame canMsg;
MCP2515 mcp2515(10);
float prev_rpm=0,RPM=0, Speed=0, prev_time = millis(), curr_time = millis(), Distance = 0;

bool start_time = false;
int input_pin=8;

#define radius 0.2286
#define turn_theta 0.89757
#define Steering_Pin A0
#define SteeringCentre 250
#define SteeringMax 600
#define SteeringMin 70

int i = 0;
float distance = 0;
int prev=0, curr=0;
int initial = 0;
// int prev_time = 0;
// int curr_time = millis();
// float RPM = 0;

float ExtraNeg = (float)90/(float)(SteeringCentre-SteeringMin);
float ExtraPos = (float)118/(float)(SteeringMax-SteeringCentre);
int flag = 0;
float SteeringDegree = 0;

ros::NodeHandle  nh;
int desire= SteeringCentre;
std_msgs::Float32 steering_msg;
ros::Publisher steering_pub("SteeringPosition", &steering_msg);
std_msgs::Float32 odom_msg;
ros::Publisher odom_pub("distance_hall", &odom_msg);
std_msgs::Float32 rpm_msg;
ros::Publisher rpm_pub("RPM", &rpm_msg);

void setup()
{ 
  Serial.begin(9600);
  pinMode(input_pin,INPUT);

  mcp2515.reset();
  mcp2515.setBitrate(CAN_500KBPS, MCP_8MHZ);
  mcp2515.setNormalMode();

  nh.initNode();
  nh.advertise(odom_pub);
  nh.advertise(rpm_pub);
  nh.advertise(steering_pub);
}

void loop(){
  if (mcp2515.readMessage(&canMsg) == MCP2515::ERROR_OK) {
    switch (canMsg.can_id){
      case 2147483712:
        curr_time = millis();
        RPM = (canMsg.data[0]*16777216 + canMsg.data[1]*65536 + canMsg.data[2]*256 + canMsg.data[3])/10;
        RPM = max(RPM,0);
        Speed = (((prev_rpm+RPM)/2) * Circumfrence) / (60 * GearRatio);
        Distance = Speed*(curr_time - prev_time)/1000;
        prev_rpm = RPM;
        prev_time = curr_time;
        rpm_msg.data = RPM;
        rpm_pub.publish(&rpm_msg);
        odom_msg.data = Distance;
        odom_pub.publish(&odom_msg);
        // DutyCycle = (canMsg.data[4]*256 + canMsg.data[5])/10;
        // InputVol = (canMsg.data[6]*256 + canMsg.data[7]);
        break;
    }
  }

  int steer = analogRead(Steering_Pin)-SteeringCentre;
  SteeringDegree = (steer > 0) ? (steer * ExtraPos) : (steer * ExtraNeg);
  SteeringDegree = 0.01745*SteeringDegree; //to radians
  steering_msg.data = steer;
  steering_pub.publish(&steering_msg);
  nh.spinOnce();
  // delay(1);

  // curr_time = millis();
  // RPM = 166.7/(curr_time - prev_time);
  // prev=curr;
  // curr = (analogRead(A3)>300 ? 1 : 0);
  // curr_time = millis();
  // if (curr_time - prev_time >2000) RPM = 0;
  // if (curr-prev==1){
  //   if (!flag){
  //     prev_time = millis();
  //   }
  //   else{
  //     RPM = 166.7/(curr_time - prev_time);
  //   }
  //   flag = !flag;}
  // }
  // else{
  //   RPM = 0;
  //   flag = 0;
  //   prev_time = curr_time;
  // }
  // if (curr-prev==1){
  //   RPM = 166.7/(curr_time - prev_time);
  //   prev_time = millis();    
  //   if (initial == 0){
  //     initial = 1;
  //   }
  //   else {
  //     distance = distance + radius*turn_theta;
  //     odom_msg.data = distance;
  //     odom_pub.publish(&odom_msg);
  //   }
  // }
  // else{
  //   RPM = 2;
  // }
  // rpm_msg.data = RPM;
  // rpm_pub.publish(&rpm_msg);
  // nh.spinOnce();
  // Serial.println("1");
  // delay(1);
}