//Libraries
#include <NewPing.h>

//Structures

//Hardware specs
NewPing sonar[]={      //Digital pin
  NewPing(7,8,maxdis),  //Left
  NewPing(3,4,maxdis),  //Front
  NewPing(11,12,maxdis),//Right
};
const int snr=sizeof(sonar)/sizeof(NewPing)-1;    //Don't erase the -1
const int motorpin[] {5,6,9,10};   //Motors (PWM)

//Settings
const int maxdis=100; //cm

void setup() {
  Serial.begin(9600);
  Serial.println();
  for (int i=0;i<sizeof(motorpin)/sizeof(int);i++) pinMode(motorpin[i],OUTPUT);
}

void loop() {
  delay(50);
  for (int i=0;i<=snr;i++){
    Serial.print(sonar[i].ping()); Serial.print(' ');
    if (cm==0) {
      pinMode(sonar[i].echo_pin,OUTPUT);
      digitalWrite(sonar[i].echo_pin,LOW);
      pinMode(sonar[i].echo_pin,INPUT);
    }
  }
}
