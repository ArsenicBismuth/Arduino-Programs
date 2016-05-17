//Libraries
#include <SoftwareSerial.h>

//Structures

//Hardware specs
SoftwareSerial bt(7,12);             //RX, TX; thus the opposite for the BT module
const int motorpin[] {4,5,10,11};   //Motors (digital), use inverter to get 2 more spaces
const int motorpwmpin[] {6,9};      //Enable pin for IC (PWM), smooth voltage change.

//App specs
const int mxspd=10;
const int mxtrn=51;
//Avoids negative

  

//Settings
const bool idle=false;            //For debugging, turning off motor
int speedratio=0;                 //-mcspd until mcspd
int turnratio=0;                  //Ranges from -mxtrn to mxtrn, with mxtrn being centered right turn and 0 being straight.

//Calculation
int reducer=turnratio*(255*2/mxtrn);
int normal=255*speedratio/mxspd;               //Maximum speed tweaked by speedratio
int slower=(255-reducer)*speedratio/mxspd;   //Reduced maximum speed for turning purpose

int code=-1;
int temp;
int counter=0;
String s;
unsigned long prevmil[2]={0};
unsigned long mil[2]={0};

void setup() {
  Serial.begin(9600); //For native USB port only
  bt.begin(9600);
  bt.println("Connected");
  
  pinMode(13,OUTPUT); //Indicator light
  for (int i=0;i<sizeof(motorpin)/sizeof(int);i++){
    pinMode(motorpin[i],OUTPUT);
  }
  for (int i=0;i<sizeof(motorpwmpin)/sizeof(int);i++){
    pinMode(motorpwmpin[i],OUTPUT);
  }
  delay(10);
}

void loop() {
  while(bt.available()){
    char c=bt.read();                 //Store chars
    s.concat(c);                      //Combine chars received
    switch(c){
      case 's':speedratio=s.toInt()-mxspd; s=""; break;  //Input ranges from 0 to 20
      case 't':speedratio=s.toInt()-mxtrn; s=""; break;  //Ranges from 0 to 102
      case 'c':code=s.toInt(); s=""; break;
    }
  }
  normal=255*speedratio/10;                //Maximum speed tweaked by speedratio
  reducer=turnratio*(255*2/mxtrn);
  slower=(255-reducer)*speedratio/mxspd;   //Reduced maximum speed for turning purpose
  motor((turnratio>=0)*normal+(turnratio<0)*slower,(turnratio<=0)*normal+(turnratio>0)*slower);
  control(code);
}

void control(int index) {
  switch (index) {
    case -1: motor(0,0); return;           //Stop
  }
}

void motor(int left,int right) {
  Serial.print(left);Serial.print(' ');Serial.print(right);Serial.print(' ');
  if (idle) return;
  //If using an inverter for each direction, remove the 2nd and 4th line
  digitalWrite(motorpin[2],(left>=0));  //If this is high
  digitalWrite(motorpin[3],!(left>=0)); //this is low
  digitalWrite(motorpin[2],(right>=0));
  digitalWrite(motorpin[3],!(right>=0));
  //PWM output, allowing speed control
  analogWrite(motorpwmpin[0],abs(left));
  analogWrite(motorpwmpin[1],abs(right));
}
