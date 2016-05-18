//Libraries
#include <SoftwareSerial.h>

//Structures

//Hardware specs
SoftwareSerial bt(8,12);          //RX, TX; thus the opposite for the BT module
const int motorpin[] {5,6,9,10}; //Motors (PWM)

//App specs
const int mxspd=10;
const int mxtrn=51;
//Avoids negative

//Settings
const bool idle=false;            //For debugging, turning off motor
int speedratio=0;                 //-mcspd until mcspd
int turnratio=0;                  //Ranges from -mxtrn to mxtrn, with mxtrn being centered right turn and 0 being straight.

//Calculation

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
  for (int i=0;i<sizeof(motorpin)/sizeof(const int);i++) pinMode(motorpin[i],OUTPUT);
  delay(10);
}

void loop() {
  while(bt.available()){
    char c=bt.read();                 //Store chars
    s.concat(c);                      //Combine chars received
    switch(c){
      case 's':speedratio=s.toInt()-mxspd; Serial.print(s); s=""; break;  //Input ranges from 0 to 20
      case 't':turnratio=s.toInt()-mxtrn; Serial.print(s); s=""; break;  //Ranges from 0 to 102
      case 'c':code=s.toInt(); Serial.print(s); s=""; break;
    }
  }
  Serial.println();
  speedratio=10;
  turnratio=51;
  int normal=255*speedratio/mxspd;                  //Maximum speed tweaked by speedratio
  int reducer=255*2*turnratio/mxtrn;
  int slower=(255-abs(reducer))*speedratio/mxspd;   //Reduced maximum speed for turning purpose
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
  //PWM into input pins
  analogWrite(motorpin[0],(left>=0)*abs(left));  //If this is PWM left
  analogWrite(motorpin[1],!(left>=0)*abs(left)); //this is 0
  analogWrite(motorpin[2],(right>=0)*abs(right));
  analogWrite(motorpin[3],!(right>=0)*abs(right));
}
