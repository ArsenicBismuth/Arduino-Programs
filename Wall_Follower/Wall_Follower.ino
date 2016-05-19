//Libraries
#include <NewPing.h>

//Structures

//Hardware specs
NewPing sonar[]={      //Analog pin
  {7,8,400},  //Front
  {11,12,400},//Right
  {3,4,400},  //Left
};
const int snr=sizeof(sonar)/sizeof(NewPing)-1;    //Don't erase the -1
const int motorpin[] {5,6,9,10};   //Motors (PWM)
const int prioritypin=2;           //Priority switch (digital)
//const int calibratepin=3;        //Calibrate switch or button (digital)

//Settings
const int speedratio=10;      //0 until 10
const int turnc=255;          //The sharpness of turning. Range of 0-510, with 510 being centered turn and 0 being straight
const int delayc=50;          //The repetition of 10ms delay before turning when no line is detected. Ex for dotted lines
const int ltrange=1;          //The count of light sensor data evaluated before averaged, remember 32767.
const int calibrange=30;      //The count of light sensor data evaluated for calibrating, remember 32767.
const bool idle=false;        //For debugging lights, turning off motor
const int controldur=0;       //Duration for control procedure to change it's behavior

//Calculation
int normal=255*speedratio/10;           //Maximum speed tweaked by speedratio
int slower(int reduce) {return (255-reduce)*speedratio/10;}

int priority=1;
int temp;
int counter=0;
int previndex=-1;
unsigned long prevmil[1]={0};
unsigned long mil[1]={0};
double kp,ki,kd;

void setup() {
  Serial.begin(9600);
  Serial.println();
  for (int i=0;i<sizeof(motorpin)/sizeof(int);i++) pinMode(motorpin[i],OUTPUT);

  /* Using internal pull-up resistors, no addiional resistor needed.
   * Configuring the switch using this method: pin-switch-ground, or just short/unshort it.
   * This causes the pin to read HIGH when the switch is open, instead of the opposite.
   */
  //pinMode(calibratepin,INPUT);
  pinMode(prioritypin,INPUT);
  //digitalWrite(calibratepin,HIGH);  //Turns on pull-up resistors in the chip
  digitalWrite(prioritypin,HIGH);
  
  delay(10);

  //Priority can only changes at initialization. Just restart Arduino to change
  if (!digitalRead(prioritypin)) priority=snr-1;  //Priority switch. Making priority 1 or lts-1
  else priority=1;
  Serial.println(priority);
}

void loop() {
  
}

void control(int index) {
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

void pid(){
  //Change in time
  mil[0]=millis();
  double dt=(double)(mil[0]-prevmil[0]);

  //Calculate error vars
  double e=ask-in;
  E+=
}

void nowall() {
}

