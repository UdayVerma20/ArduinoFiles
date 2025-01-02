#include <ros.h>
#include <std_msgs/Float32.h>

#define RPWM 5
#define LPWM 3
#define REN 4
#define LEN 2
#define Steering_Pin A0
#define SteeringCentre 250
#define SteeringMax 600
#define SteeringMin 70

float ExtraNeg = (float)90 / (float)(SteeringCentre - SteeringMin);
float ExtraPos = (float)118 / (float)(SteeringMax - SteeringCentre);
int denom = 0;
int desire = 450;
float kp = 1, kd = 0.1, ki = 0.1;
float curr_error = 0, prev_error = 0, diff_error = 0, sum_error = 0;
bool flag = 1;
float prev_ef = 0, ef = 0, alpha = 0.5;
ros::NodeHandle nh;

void motorCb(const std_msgs::Float32 &motor_msg) {
  desire = motor_msg.data;
}

ros::Subscriber<std_msgs::Float32> sub("MotorControl", &motorCb);

void motor_stop() {
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
  digitalWrite(REN, 0);
  digitalWrite(LEN, 0);
}

void setup() {
  Serial.begin(115200);
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);
  pinMode(A0, INPUT);

  nh.initNode();
  nh.subscribe(sub);
}

void loop() {
  int steer = analogRead(Steering_Pin);
  // Serial.println(steer);
  // denom = 800;
  digitalWrite(REN, 1);
  digitalWrite(LEN, 1);
  
  // // pid
  curr_error = steer - desire;
  if (abs(curr_error)<5) return;
  if(flag){
    flag = 0;
    prev_ef = curr_error;
  }
  ef = alpha*curr_error + (1-alpha)*prev_ef;
  // diff_error = curr_error - prev_error;
  diff_error = ef - prev_ef;
  sum_error += curr_error;
  float control = kp*curr_error + kd*diff_error + ki*sum_error;
  // prev_error = curr_error;
  prev_ef = ef;
  
  if (control > 0){
    analogWrite(LPWM, 0);
    analogWrite(RPWM, min(255,floor(abs(control))));
  } else if (control < 0) {
    analogWrite(RPWM, 0);
    analogWrite(LPWM, min(255,floor(abs(control))));
  } else {
    motor_stop();
  }

  // // variable p
  // float val = (steer - desire);
  // val /= 1023.0;
  // if (abs(val) < 0.35 && abs(val) > 0) val = (val / abs(val)) * 0.35;
  // if (abs(val) > 1) val = val / abs(val);

  // float x = 255 * val;

  // if (x > 0) {
  //   analogWrite(LPWM, 0);
  //   analogWrite(RPWM, floor(abs(x)));
  // } else if (x < 0) {
  //   analogWrite(RPWM, 0);
  //   analogWrite(LPWM, floor(abs(x)));
  // } else {
  //   motor_stop();
  // }

  nh.spinOnce();  // Process incoming messages
  delay(10);      // Avoid excessive CPU usage
}
