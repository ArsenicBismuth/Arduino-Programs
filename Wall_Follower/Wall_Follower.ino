
//Libraries
#include <NewPing.h>
#include <PID_v1.h>

//Structures

//Hardware specs
const int maxdis=300;   //cm
NewPing sonar[]={       //Digital pin
  NewPing(3,4,maxdis),  //Left
  NewPing(7,8,maxdis), //Front
  //NewPing(11,12,maxdis) //Right
};
const int snr=sizeof(sonar)/sizeof(NewPing)-1;    //Don't erase the -1
const int motorpin[] {5,6,9,10};   //Motors (PWM)
const int prioritypin=2;           //Priority switch (digital)

//Settings
const int dis[]={500,300,500};  //Asked distance (in ms). Left, front, right.
double maxturn=0.6;       //Max ratio of spd in PID limit
int spd=100;                     //Maximum speed tweaked by spd
const bool idle=false;          //For debugging lights, turning off motor

//Calculation
double in,out,setpoint;
double kp(0.5),ki(0),kd(0);
PID pid(&in,&out,&setpoint,kp,ki,kd,REVERSE);

int priority=1;
unsigned long prevmil[1]={0};
unsigned long mil[1]={0};
int ms[snr+1];                //Result of ultrasonic sensor in milisecond
String s;

void setup() {
  Serial.begin(9600);
  Serial.println();
  for (int i=0;i<sizeof(motorpin)/sizeof(int);i++) pinMode(motorpin[i],OUTPUT);
  pid.SetMode(AUTOMATIC);
  pid.SetOutputLimits(0,spd*maxturn);
  setpoint=0;

  /* Using internal pull-up resistors, no addiional resistor needed.
   * Configuring the switch using this method: pin-switch-ground, or just short/unshort it.
   * This causes the pin to read HIGH when the switch is open, instead of the opposite. */
  pinMode(prioritypin,INPUT);
  digitalWrite(prioritypin,HIGH); //Turns on pull-up resistors in the chip
  
  delay(10);

  //Priority can only changes at initialization. Just restart Arduino to change
  priority=2*!digitalRead(prioritypin); //Priority switch. Making priority 0 or 2 (left or right). Default is left.
  Serial.println(priority);
}

void loop() {
  tuning();
  ussensor();
  control();
  Serial.println();
}

void tuning(){
  while(Serial.available()){
    char c=Serial.read();                 //Store chars
    s.concat(c);                      //Combine chars received
    switch(c){
      case 's':spd=s.toInt(); Serial.print(s); s=""; break;  //Input ranges from 0 to 10
      case 't':maxturn=s.toInt()/10; Serial.print(s); s=""; break;  //Input ranges from 0 to 10
      case 'p':kp=s.toInt(); Serial.print(s); s=""; break;
      case 'i':ki=s.toInt(); Serial.print(s); s=""; break;
      case 'd':kd=s.toInt(); Serial.print(s); s=""; break;
    }
    pid.SetTunings(kp,ki,kd);
    pid.SetOutputLimits(0,spd*maxturn);
  }
} 

void ussensor() {ussensor(-1);} //Overloading, allows for default value
void ussensor(int single) {
  int i=0;
  int last=snr;
  if (single!=-1) {
    i=single;
    last=single;
  }
  for (i;i<=last;i++){
    delay(30);
    ms[i]=sonar[i].ping();
    Serial.print(ms[i]); Serial.print(' ');
    /*  For defective sensor in which it will wait forever
     *  until the sonar is back. Thus resulting 0. */
    if (ms[i]==0) {
      //Echo pins, in this case, it's 4,8,12
      pinMode(4*(i+1),OUTPUT);
      digitalWrite(4*(i+1),LOW);
      pinMode(4*(i+1),INPUT);
    }
  }
}

void control() {
  //If there's wall upfront or it isn't straight
  if (ms[1]<=dis[1]&&ms[1]>0){
    do {
      motor(spd,-spd);
      delay(500);
      ussensor(1);                //Recheck only front
    } while (ms[1]!=0&&ms[1]<=dis[0]);
    return;
  }
  in=constrain(abs(ms[priority]-dis[priority]),0,ms[priority]);  //Using the difference as PID instead of directly using input
  pid.Compute();
  
  if (ms[priority]<=dis[priority]) motor(spd,spd-out);
  else motor(spd-out,spd);
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
