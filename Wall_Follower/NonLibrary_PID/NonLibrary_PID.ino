//Libraries
#include <NewPing.h>

//Structures

//Hardware specs
const int maxdis=300;   //cm
const int maxdis2=15;   //cm
NewPing sonar[]={       //Digital pin
  NewPing(3,4,maxdis2),  //Left
  NewPing(7,8,maxdis),  //Front
  NewPing(11,12,maxdis2) //Right
};
const int snr=sizeof(sonar)/sizeof(NewPing)-1;    //Don't erase the -1
const int motorpin[] {5,6,9,10};   //Motors (PWM)
const int prioritypin=2;           //Priority switch (digital)

//Settings
const int dis[]={500,400,500};  //Asked distance (in ms). Left, front, right.
const int speedratio=10;      //0 until 10
const int turncratio=0;       //The sharpness of turning. Range of 0-510, with 510 being centered turn and 0 being straight
const int delayc=50;          //The repetition of 10ms delay before turning when no line is detected. Ex for dotted lines
const bool idle=false;        //For debugging lights, turning off motor
const int controldur=0;       //Duration for control procedure to change it's behavior

//Calculation
int normal=255*speedratio/10;                     //Maximum speed tweaked by speedratio
int reduce(int x) {return (255-x)*((double)speedratio/10);} //Function for slower speed, with 255 being pivoted and 510 being centered
int slower=reduce(0);
double Ee,inp,out;            //Error sum, previous input, & output
double kp (0.3),ki (0.03),kd (0.4);  //PID Constants
int tsample (100);            //PID sample time, 1 sec

int priority=1;
int temp;
int counter=0;
int previndex=-1;
unsigned long prevmil[1]={0};
unsigned long mil[1]={0};
int ms[snr+1];                //Result of ultrasonic sensor in milisecond

void setup() {
  Serial.begin(9600);
  Serial.println();
  for (int i=0;i<sizeof(motorpin)/sizeof(int);i++) pinMode(motorpin[i],OUTPUT);

  /* Using internal pull-up resistors, no addiional resistor needed.
   * Configuring the switch using this method: pin-switch-ground, or just short/unshort it.
   * This causes the pin to read HIGH when the switch is open, instead of the opposite.
   */
  pinMode(prioritypin,INPUT);
  digitalWrite(prioritypin,HIGH); //Turns on pull-up resistors in the chip
  
  delay(10);

  //Priority can only changes at initialization. Just restart Arduino to change
  priority=2*!digitalRead(prioritypin); //Priority switch. Making priority 0 or 2 (left or right). Default is left.
  Serial.println(priority);
}

void loop() {
  ussensor();
  control();
  Serial.println();
}

void ussensor() {
  for (int i=0;i<=snr;i++){
    delay(30);
    ms[i]=sonar[i].ping();
    Serial.print(ms[i]); Serial.print(' ');
    /*  For defective sensor in which it will wait forever
     *  until the sonar is back. Thus resulting 0.
     */
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
  if (ms[1]<=dis[1]) {
    motor(normal,reduce(510));
    return;
  };
  slower=reduce(-1*pid(dis[priority],ms[priority]));
  
  if (ms[priority]<=dis[priority]) motor(normal,slower);
  else motor(slower,normal);
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

//Currently can only used by a single sensor
double pid(int asked, int in){
  //Change in time
  mil[0]=millis();
  int dt=(mil[0]-prevmil[0]);
  if (dt>=tsample){
    //Calculate error vars
    double e=asked-in;
    Ee+=e;
    double din=(in-inp);

    //Result
    out=kp*e + ki*Ee + kd*din;
    
    inp=in;               //Save input into previous input
    prevmil[0]=mil[0];
  }
  return out;
}

//For changing the tunings outside IDE
void tunings(double nkp, double nki, double nkd){
  double tsamplesec = ((double)tsample)/1000;
  kp=nkp; ki=nki; kd=nkd;
}

//For changing the sample time outside IDE
void setsamplet(int ntsample){
  if (ntsample>0){
    double ratio=(double)ntsample/(double)tsample;
    ki*=ratio;
    kd/=ratio;
    tsample=(unsigned long)ntsample;
  }
}

