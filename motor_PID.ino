#include <TimerOne.h>
#include <math.h>

#define EN_R   5
#define M_R1   7
#define M_R2   8
#define EN_L   11
#define M_L1   9
#define M_L2   10
int speedR=0,speedL=0;
volatile int cntR=0,cntL=0;

int goal=3;
int PID_val;

float errorSumL = 0.0;
float errorSumR = 0.0;
float errorCurr = 0.0;
float errorPrevL = 0.0;
float errorPrevR = 0.0;
float errorDiff = 0.0;

float error_P = 1.8; //ku=10
float error_I = 3.2;
float error_D = 2.5;//2.5;
void pid_control(){

  int gain;
  Serial.print(goal);Serial.print(" ");  
  Serial.print(cntR);Serial.print(" ");
  Serial.print(cntL);Serial.print(" ");  
  errorCurr = cntR - goal;
  errorSumR = errorSumR + errorCurr;
  errorDiff = errorCurr - errorPrevR;
  gain = error_P * errorCurr + error_I * errorSumR + error_D * errorDiff; 
  errorPrevR = errorCurr;
  speedR=gain;
  if(speedR>=255) speedR=255;
  else if(speedR<=-255) speedR=255;
  speedR=abs(speedR);
  
  //Serial.print(gain);Serial.print(" ");
  Serial.print(speedR);Serial.print(" ");  
  errorCurr = cntL - goal;
  errorSumL = errorSumL + errorCurr;
  errorDiff = errorCurr - errorPrevL;
  gain = error_P * errorCurr + error_I * errorSumL + error_D * errorDiff; 
  errorPrevL = errorCurr;
  speedL=gain; 
  if(speedL>=255) speedL=255;
  else if(speedL<=-255) speedL=255;
  speedL=abs(speedL);
  
  //Serial.print(gain);Serial.print(" ");
  Serial.println(speedL);   

  forward();
}
void timerIsr()
{
  pid_control();  
  Timer1.detachInterrupt();           //stop the timer
  cntR=0;cntL=0;
  Timer1.attachInterrupt(timerIsr);   //enable the timer
}
void CNTR(){
  cntR++;  
}
void CNTL(){
  cntL++;  
}
void setup(){
  Serial.begin(9600);   //시리얼모니터
  mySerial.begin(9600); //블루투스 시리얼
  pinMode(EN_R,OUTPUT);
  pinMode(M_R1,OUTPUT);  
  pinMode(M_R2,OUTPUT);
  pinMode(EN_L,OUTPUT);  
  pinMode(M_L1,OUTPUT);
  pinMode(M_L2,OUTPUT);
  pinMode(2, INPUT_PULLUP); //오른쪽
  pinMode(3, INPUT_PULLUP); //왼쪽


  Timer1.initialize(100000);
  attachInterrupt(digitalPinToInterrupt(2),CNTR, FALLING );  
  attachInterrupt(digitalPinToInterrupt(3),CNTL, FALLING );  
  Timer1.attachInterrupt(timerIsr);
  forward();

}
void forward(){
 analogWrite(EN_R, speedR);
 digitalWrite(M_R1,HIGH);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L, speedL);
 digitalWrite(M_L1,HIGH);
 digitalWrite(M_L2,LOW);
}
void loop(){
}
