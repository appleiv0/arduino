#define BASE_R 150
#define BASE_L 150
int speedR=BASE_R;
int speedL=BASE_L;

#define LINE_LEFT A2
#define LINE_MIDDLE A1
#define LINE_RIGHT A0
#define EN_R   5
#define M_R1   7
#define M_R2   8
#define EN_L   11
#define M_L1   9
#define M_L2   10
void setup() {
  pinMode(EN_R,OUTPUT);
  pinMode(M_R1,OUTPUT);
  pinMode(M_R2,OUTPUT);
  pinMode(M_L1,OUTPUT);
  pinMode(M_L2,OUTPUT);
  pinMode(EN_L,OUTPUT);

  pinMode(LINE_RIGHT,INPUT);
  pinMode(LINE_MIDDLE,INPUT);
  pinMode(LINE_LEFT,INPUT);

  digitalWrite(M_R1,HIGH);
  digitalWrite(M_R2,LOW);
  digitalWrite(M_L1,HIGH);
  digitalWrite(M_L2,LOW);
  Serial.begin(9600);
}
int prev=0;
void loop() {
  analogWrite(EN_R,speedR);
  analogWrite(EN_L,speedL);

  int r=digitalRead(LINE_RIGHT);
  int m=digitalRead(LINE_MIDDLE);
  int l=digitalRead(LINE_LEFT);
  Serial.print(r);Serial.print(m);Serial.print(l);
  Serial.println();

  if(!r && m && !l) {speedR=BASE_R;speedL=BASE_L;prev=10;}
  else if(r && (m && !l)) {speedR=0;speedL=BASE_L-50;delay(0);prev=110;}
  else if(r && (!m && !l)) {speedR=0;speedL=BASE_L-50;delay(0);prev=100;}
  else if( (!r && m) && l) {speedR=BASE_R-50;speedL=0;delay(0);prev=11;}
  else if( (!r && !m)&& l) {speedR=BASE_R-50;speedL=0;delay(0);prev=1;}
  else if(r && m && l){speedR=0;speedL=0;}
/*  else if( !r && !m && !l)
  {
    if(prev==100) {speedR=0;speedL=BASE_L;delay(10);}
    else if(prev==1){speedR=BASE_R-50;speedL=0;delay(10);}
    else {speedR=0;speedL=0;}
  }
*/
}
