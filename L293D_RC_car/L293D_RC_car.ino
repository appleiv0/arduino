#include <SoftwareSerial.h>
int blueTx=12;   //Tx (보내는핀 설정)at
int blueRx=13;   //Rx (받는핀 설정)
SoftwareSerial mySerial(blueTx, blueRx);
#define EN_R   5
#define M_R1   6
#define M_R2   7
#define EN_L   11// 9
#define M_L1   9//10
#define M_L2   10
int speed=200;
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
 analogWrite(EN_R, speed);
 digitalWrite(M_R1,HIGH);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L, speed);
 digitalWrite(M_L1,HIGH);
 digitalWrite(M_L2,LOW);
}
void back()
{
 analogWrite(EN_R, speed);
 digitalWrite(M_R1,LOW);
 digitalWrite(M_R2,HIGH);
 analogWrite(EN_L, speed);
 digitalWrite(M_L1,LOW);
 digitalWrite(M_L2,HIGH);
}
void backward()
{
 analogWrite(EN_R, speed);
 digitalWrite(M_R1,LOW);
 digitalWrite(M_R2,HIGH);
 analogWrite(EN_L, speed);
 digitalWrite(M_L1,LOW);
 digitalWrite(M_L2,HIGH);
}
void left()
{
 analogWrite(EN_R, speed);
 digitalWrite(M_R1,LOW);
 digitalWrite(M_R2,HIGH);
 analogWrite(EN_L, 0);
 digitalWrite(M_L1,LOW);
 digitalWrite(M_L2,LOW);
}
void right()
{
 analogWrite(EN_R, speed);
 digitalWrite(M_R1,LOW);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L, speed);
 digitalWrite(M_L1,HIGH);
 digitalWrite(M_L2,LOW);
}
void forwardright()
{
 analogWrite(EN_R, speed-50);
 digitalWrite(M_R1,HIGH);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L, speed);
 digitalWrite(M_L1,HIGH);
 digitalWrite(M_L2,LOW);
}
void forwardleft()
{
 analogWrite(EN_R, speed);
 digitalWrite(M_R1,HIGH);
 digitalWrite(M_R2,LOW);
 analogWrite(EN_L, speed-50);
 digitalWrite(M_L1,HIGH);
 digitalWrite(M_L2,LOW);
}
void loop()
{
  if (mySerial.available()) {  
    char command = mySerial.read();
    Serial.write(command);
    switch(command){
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
       case 'S':  //속도조절
          stop();
          break;
       case '0':  //속도조절
          speed=0;
          break;
       case '1':       case '2':       case '3':
          speed=150;
          break;      
       case '4':       case '5':       case '6':
          speed=200;
          break;
       case '7':
       case '8':       case '9':       case 'q':
          speed=255;
          break;      
     }
  }
}
