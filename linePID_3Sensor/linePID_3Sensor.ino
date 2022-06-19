//#include <math.h>


#define EN_R   5
#define M_R1   7
#define M_R2   8
#define EN_L   11
#define M_L1   9
#define M_L2   10

#define LINE_LEFT 6
#define LINE_MIDDLE 4
#define LINE_RIGHT 12 

#define BASE_SPEED 80
int speedR=BASE_SPEED,speedL=BASE_SPEED;
//double P,I,D,previousError,PIDvalue,Kp=10,Ki=0.01,Kd=0;//80
double P,I,D,previousError,PIDvalue,Kp=7,Ki=0,Kd=70;//80
//double P,I,D,previousError,PIDvalue,Kp=10,Ki=0,Kd=140;//80

int error;
void calculatePID()
{
  P = error;
  I = I + error;
  D = error-previousError;
  PIDvalue = (Kp*P) + (Ki*I) + (Kd*D);
  //Serial.print("PID:");
  //Serial.print(P);Serial.print(" ");
  previousError = error;
}

void motorPIDcontrol()
{

  speedR = BASE_SPEED + PIDvalue;
  speedL = BASE_SPEED - PIDvalue;
  Serial.print(error);
  Serial.print(" ");
  Serial.print(PIDvalue);
  Serial.print(" ");  
  Serial.print(speedL);
  Serial.print(" ");
  Serial.println(speedR);

  forward();
}

void setup()
{
  Serial.begin(9600);   //시리얼모니터

  pinMode(A0,INPUT);
  pinMode(A1,INPUT);
  pinMode(A2,INPUT);
  pinMode(EN_R,OUTPUT);
  pinMode(M_R1,OUTPUT);  
  pinMode(M_R2,OUTPUT);
  pinMode(EN_L,OUTPUT);  
  pinMode(M_L1,OUTPUT);
  pinMode(M_L2,OUTPUT);
  pinMode(2, INPUT_PULLUP); //오른쪽
  pinMode(3, INPUT_PULLUP); //왼쪽

  pinMode(LINE_LEFT,INPUT);
  pinMode(LINE_MIDDLE,INPUT);
  pinMode(LINE_RIGHT,INPUT);

//  Timer1.initialize(100000);
//  attachInterrupt(digitalPinToInterrupt(2),CNTR, FALLING );  
//  attachInterrupt(digitalPinToInterrupt(3),CNTL, FALLING );  
//  Timer1.attachInterrupt(timerIsr);
  forward();

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


void loop()
{
  read_line_error();
  calculatePID();
  motorPIDcontrol();
  
}
int sensorVal[3];
void read_line_error()
{
  sensorVal[0]=digitalRead(A0);  
  sensorVal[1]=digitalRead(A1);  
  sensorVal[2]=digitalRead(A2);
  Serial.print(sensorVal[0]);  
  Serial.print(sensorVal[1]);  
  Serial.print(sensorVal[2]);
  Serial.print(" ");
  
  if(sensorVal[0] && !sensorVal[1] && !sensorVal[2])      error=-4;//80
  else if(sensorVal[0] && sensorVal[1] && !sensorVal[2])  error=-8;
  else if(!sensorVal[0] && sensorVal[1] && !sensorVal[2]) error=0;

  else if(!sensorVal[0] && sensorVal[1] && sensorVal[2])  error=4;
  else if(!sensorVal[0] && !sensorVal[1] && sensorVal[2]) error=8;

/*  if(sensorVal[0] && !sensorVal[1] && !sensorVal[2])      error=-2.0; //90
  else if(sensorVal[0] && sensorVal[1] && !sensorVal[2])  error=-8;
  else if(!sensorVal[0] && sensorVal[1] && !sensorVal[2]) error=0;
  else if(!sensorVal[0] && !sensorVal[1] && sensorVal[2]) error=2.0;
  else if(!sensorVal[0] && sensorVal[1] && sensorVal[2])  error=8;
 */
}
