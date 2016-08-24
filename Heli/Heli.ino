//Libraries
#include <SoftwareSerial.h>
#include <PID_v1.h>

//Structures

//Hardware specs
const int motorpin[] {3,9,5}; //Motors (PWM)

//App specs (avoid negative)
const int mxup=255;
const int mxyaw=510;
const int mxptc=510;

//Settings
const bool idle=false;      //For debugging, turning off motor
int up=0;                   //-mxup until mxup
int yaw=0;                  //Ex reduce clockwise blade speed to rotate counter-clockwise
int ptc=0;                  //Rotate front back

//Calculation
  //PID for gyro
  PID gyroz(&gzin,&gzout,&gzset,2,5,1,REVERSE); //Yaw
  PID gyroy(&gyin,&gyout,&gyset,2,5,1,REVERSE); //Pitch
  /*PID gyrox(&gxin,&gxout,&gxset,2,5,1,REVERSE); Usable if there's roll movement*/
  //PID for accelerometer
  PID accx(&axin,&axout,&axset,2,5,1,REVERSE); //Forward-backward
  PID accy(&ayin,&ayout,&ayset,2,5,1,REVERSE); //Usable if there's side movement
  PID accz(&azin,&azout,&azset,2,5,1,REVERSE); //Vertical

int code=0;
String s;
unsigned long prevmil[2]={0};
unsigned long mil[2]={0};

void setup() {
  Serial.begin(9600); //For native USB port only
  
  pinMode(13,OUTPUT); //Indicator light
  for (int i=0;i<sizeof(motorpin)/sizeof(const int);i++) pinMode(motorpin[i],OUTPUT);
  delay(10);
}

void loop() {
  while(Serial.available()){
    char c=Serial.read();                 //Store chars
    s.concat(c);                      //Combine chars received
    switch(c){
      case 'u':up=s.toInt(); Serial.print(s); s=""; break;  //Input ranges from 0 to 255
      case 'y':yaw=s.toInt()-mxyaw/2; Serial.print(s); s=""; break;  //Input ranges from 0 to 510*
      case 'p':ptc=s.toInt()-mxptc/2; Serial.print(s); s=""; break;  //Ranges from 0 to 510*
      case 'c':code=s.toInt(); Serial.print(s); s=""; break;
      //*transformed into -255 to 255
    }
  }
  Serial.println();
  control(code);
  int mup=255*up/mxup;                  //Maximum speed tweaked by speedratio
  int myaw=255*2*yaw/mxyaw;
  int mptc=255*2*yaw/mxyaw;
  int slower=(255-abs(reducer))*up/mxup;   //Reduced maximum speed for turning purpose
  motor((yaw>=0)*normal+(yaw<0)*slower,(yaw<=0)*normal+(yaw>0)*slower,ptc);
}

void motor(int clockwise,int counter,int tail) {   //Upper blade and lower blade respectively for this
  Serial.print(clockwise);Serial.print(' ');Serial.print(counter);Serial.print(' ');Serial.print(tail);
  if (idle) return;
  //PWM into input pins
  analogWrite(motorpin[0],clockwise);
  analogWrite(motorpin[1],counter);
  analogWrite(motorpin[2],tail);
}

void control(int index) {
  switch (index) {
    case -1: motor(0,0,0); return;           //Stop
  }
}
