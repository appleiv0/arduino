#include <SoftwareSerial.h>
#include <TimerOne.h>
#include <math.h>

int blueTx=12;   //Tx (보내는핀 설정)at
int blueRx=13;   //Rx (받는핀 설정)
SoftwareSerial mySerial(blueTx, blueRx);
#define EN_R   5
#define M_R1   7
#define M_R2   8
#define EN_L   11
#define M_L1   9
#define M_L2   10
int speedR=0,speedL=0;
volatile int cntR=0,cntL=0;

int goal=5;
int PID_val;

double errorSumL = 0.0;
double errorSumR = 0.0;
double errorCurr = 0.0;
double errorPrevL = 0.0;
double errorPrevR = 0.0;
double errorDiff = 0.0;

float error_P = 1.8; //ku=10
float error_I = 3.2;
float error_D = 2.5;//2.5;
void pid_control(){
//  Serial.println("PID");
  int gain;
  Serial.print(goal);
  Serial.print(" ");  
  Serial.print(cntR);
  Serial.print(" ");
  Serial.print(cntL);
  Serial.print(" ");  
  errorCurr = cntR - goal;
  errorSumR = errorSumR + errorCurr;
  errorDiff = errorCurr - errorPrevR;
  gain = error_P * errorCurr + error_I * errorSumR + error_D * errorDiff; 
  errorPrevR = errorCurr;
  speedR=gain;
  if(speedR>=255) speedR=255;
  else if(speedR<=-255) speedR=255;
  speedR=abs(speedR);
  
//  Serial.print(gain);
//  Serial.print(" ");
  Serial.print(speedR);
  Serial.print(" ");  
  errorCurr = cntL - goal;
  errorSumL = errorSumL + errorCurr;
  errorDiff = errorCurr - errorPrevL;
  gain = error_P * errorCurr + error_I * errorSumL + error_D * errorDiff; 
  errorPrevL = errorCurr;
  speedL=gain; 
  if(speedL>=255) speedL=255;
  else if(speedL<=-255) speedL=255;
  speedL=abs(speedL);
  
//  Serial.print(gain);
//  Serial.print(" ");
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


void CNTR()
{
  cntR++;  
}
void CNTL()
{
  cntL++;  
}


void setup()
{
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
void stop()
{
 analogWrite(EN_R,LOW);
 digitalWrite(M_R1,LOW);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L,LOW);
 digitalWrite(M_L1,LOW);  
 digitalWrite(M_L2,LOW);
}
void forward()
{
 analogWrite(EN_R, speedR);
 digitalWrite(M_R1,HIGH);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L, speedL);
 digitalWrite(M_L1,HIGH);
 digitalWrite(M_L2,LOW);
}
void back()
{
 analogWrite(EN_R, speedR);
 digitalWrite(M_R1,LOW);
 digitalWrite(M_R2,HIGH);
 analogWrite(EN_L, speedL);
 digitalWrite(M_L1,LOW);
 digitalWrite(M_L2,HIGH);
}
void backward()
{
 analogWrite(EN_R, speedR);
 digitalWrite(M_R1,LOW);
 digitalWrite(M_R2,HIGH);
 analogWrite(EN_L, speedL);
 digitalWrite(M_L1,LOW);
 digitalWrite(M_L2,HIGH);
}
void left()
{
 analogWrite(EN_R, speedR);
 digitalWrite(M_R1,HIGH);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L, 0);
 digitalWrite(M_L1,LOW);
 digitalWrite(M_L2,LOW);
}
void right()
{
 analogWrite(EN_R, speedR);
 digitalWrite(M_R1,LOW);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L, speedL);
 digitalWrite(M_L1,HIGH);
 digitalWrite(M_L2,LOW);
}
void forwardright()
{
 analogWrite(EN_R, speedR-50);
 digitalWrite(M_R1,HIGH);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L, speedL);
 digitalWrite(M_L1,HIGH);
 digitalWrite(M_L2,LOW);
}
void forwardleft()
{
 analogWrite(EN_R, speedR);
 digitalWrite(M_R1,HIGH);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L, speedL-50);
 digitalWrite(M_L1,HIGH);
 digitalWrite(M_L2,LOW);
}
#define PID_CONTROL 0
#define BT_CONTROL 1
int mode=PID_CONTROL;
void loop()
{
  noInterrupts();
  if (mySerial.available()) {  

    char command = mySerial.read();
    if(command!='S') Serial.write(command);
    switch(command){
       case 'w':
       case 'W':
          Timer1.stop();
          mode=BT_CONTROL;
          break;
       case 'U':
       case 'u':
          Timer1.start();
          Timer1.detachInterrupt();           //stop the timer
          cntR=0;cntL=0;
          Timer1.attachInterrupt(timerIsr);   //enable the timer    
          mode=PID_CONTROL;     
          break;
       case 'S':  //속도조절
            if(mode==BT_CONTROL)  stop();
          break;
       speedL=200;
       speedR=200;
       case 'F':
          forward();
          break;
       case 'G':
          forwardright();
          break;    
       case 'I':
          forwardleft();
          break;
       case 'B':
          back();
          break;    
       case 'L':
          left();
          break;
       case 'R':
          right();
          break;
       case '0':  //속도조절
          goal=0;
          break;
       case '1':
          goal=1;
          break;  
       case '2':
          goal=2;
          break;
       case '3':
          goal=3;
          break;      
       case '4':
          goal=4;
          break;          
       case '5':
          goal=5;
          break;          
       case '6':
          goal=6;
          break;
       case '7':
          goal=7;
          break;          
       case '8':
          goal=8;
          break;  
       case '9':
          goal=9;
          break;          
       case 'q':
          goal=10;
          break;      
     }
  }
  interrupts();
}
