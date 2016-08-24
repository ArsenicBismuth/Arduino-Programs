//Libraries
#include <NewPing.h>

//Structures

//Hardware specs
const int maxdis=400;   //cm
NewPing sonar[]={       //Digital pin
  NewPing(3,4,maxdis),  //Left
  NewPing(7,8,maxdis),  //Front
  //NewPing(11,12,maxdis) //Right
};
const int snr=sizeof(sonar)/sizeof(NewPing)-1;    //Don't erase the -1
const int motorpin[] {5,6,9,10};   //Motors (PWM)

//Settings

void setup() {
  Serial.begin(9600);
  Serial.println();
  for (int i=0;i<sizeof(motorpin)/sizeof(int);i++) pinMode(motorpin[i],OUTPUT);
}

void loop() {
  for (int i=0;i<=snr;i++){
    delay(30);
    int ms=sonar[i].ping();
    Serial.print(ms); Serial.print(' ');
    /*  For defective sensor in which it will wait forever
     *  until the sonar is back. Thus resulting 0.
     */
    if (ms==0) {
      //Echo pins, in this case, it's 4,8,12
      pinMode(4*(i+1),OUTPUT);
      digitalWrite(4*(i+1),LOW);
      pinMode(4*(i+1),INPUT);
    }
  }
  Serial.println();
}
