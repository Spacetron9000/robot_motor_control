
#include <ros.h>
#include <std_msgs/UInt8MultiArray.h> 
#include <std_msgs/Int16.h> 

//Motor Pin Definitions
int E1 = 5;     //M1 Speed Control
int E2 = 6;     //M2 Speed Control
int M1 = 4;     //M1 Direction Control
int M2 = 7;     //M2 Direction Control

//Motor Signal Definitions
int sL = 0; 
int sR = 0;
int dL = 0; 
int dR = 1; 

//Encoder Pin Definitions 
int ENC_PIN_L = 0;
int ENC_PIN_R = 1; 

std_msgs::Int16 left; 
std_msgs::Int16 right; 

void motorCallback(std_msgs::UInt8MultiArray &arr) {
  sL = arr.data[0];
  sR = arr.data[1];
  dL = arr.data[2];
  dR = arr.data[3]; 

  setMotors(sL, sR, dL, dR);
}


void setMotors(int a, int b, int lD, int rD)
{
  analogWrite (E1, a);
  digitalWrite(M1, lD);
  analogWrite (E2, b);
  digitalWrite(M2, rD);
}

void lWheelSpeed() {
  prev_left_ticks = left_ticks; 
  
  if (dR == 0) {
    left.data--;
  }
   else {
    left.data++; 
   }

  
}

void rWheelSpeed() {
  
  prev_right_ticks = right_ticks; 
  
  if  (dL == 1){ 
    right.data--; 
  }
  else {
    right.data++; 
  }
}

ros::Subscriber<std_msgs::UInt8MultiArray> sub("motor", &motorCallback);

ros::Publisher left_pub("encoders/left", &left); 
ros::Publisher right_pub("encoders/right", &right); 



void setup(void) {

  
  nh.getHardware()->setBaud(115200); 
  int i;
  for (i = 4; i <= 7; i++)
    pinMode(i, OUTPUT);
  Serial.begin(115200);      //Set Baud Rate

  digitalWrite(E1, LOW);
  digitalWrite(E2, LOW);

  attachInterrupt(ENC_PIN_L, lWheelSpeed, CHANGE); 
  attachInterrupt(ENC_PIN_R, rWheelSpeed, CHANGE); 

  
  nh.initNode(); 

  left.data = 0; 
  right.data = 0; 
  nh.advertise(left_pub); 
  nh.advertise(right_pub); 
  
  nh.subscribe(sub);
  nh.spinOnce();
}


void loop(void)
{

  left_pub.publish(&left); 
  right_pub.publish(&right);   
  nh.spinOnce(); 
  
  delay(10); 
}
