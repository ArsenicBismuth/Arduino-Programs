//Libraries

//Structures
struct sensor{
  int pin;
  int thold;
};

//Hardware specs
sensor light[]={      //Analog pin
  {2,300},  //Middle
  {1,400},  //Left
  {3,400},  //Right
  {2,300}   //Middle
  //This way, the last priority sensor will always be the middle one
};
const int lts=sizeof(light)/sizeof(sensor)-1;    //Don't erase the -1
sensor color={5,200};
const int motorpin[] {4,5,10,11};   //Motors (digital), use inverter to get 2 more spaces
const int motorpwmpin[] {6,9};      //Enable pin for IC (PWM), smooth voltage change.
const int redpin=8;                 //red LED (digital)
const int greenpin=7;               //green LED (digital)
const int prioritypin=2;            //Priority switch (digital)
const int calibratepin=3;           //Calibrate switch or button (digital)

//Settings
const int speedratio=10;      //0 until 10
const int turnc=255;          //The sharpness of turning. Range of 0-510, with 510 being centered turn and 0 being straight
const int delayc=50;          //The repetition of 10ms delay before turning when no line is detected. Ex for dotted lines
const int clinterval=60;      //interval of coloured led blink (ms)
const int linec=-1;           //1 or -1, -1 for white line. Update: Not necessarily true.
                              //Just do a check with computer once to make sure which area returns higher value.
                              //Thus, 1 if line value < surrounding and the opposite for -1.
const int ltrange=1;          //The count of light sensor data evaluated before averaged, remember 32767.
const int calibrange=30;      //The count of light sensor data evaluated for calibrating, remember 32767.
const bool idle=false;        //For debugging lights, turning off motor
const int controldur=0;       //Duration for control procedure to change it's behavior

//Calculation
int analogsum[lts+1]={0};
int analogavg[lts+1]={999};
int normal=255*speedratio/10;           //Maximum speed tweaked by speedratio
int slower=(255-turnc)*speedratio/10;   //Reduced maximum speed for turning purpose

int priority=1;                         //Light sensor priority, refers to the index from lightpin
int temp;
int dored=true;
bool docolor=false;
int prevcl=color.thold+1;
int counter=0;
int previndex=-1;
unsigned long prevmil[2]={0};
unsigned long mil[2]={0};

void setup() {
  Serial.begin(9600);
  Serial.println();
  pinMode(redpin,OUTPUT);
  pinMode(greenpin,OUTPUT); 
  pinMode(13,OUTPUT);               //Indicator light
  for (int i=0;i<sizeof(motorpin)/sizeof(int);i++){
    pinMode(motorpin[i],OUTPUT);
  }
  for (int i=0;i<sizeof(motorpwmpin)/sizeof(int);i++){
    pinMode(motorpwmpin[i],OUTPUT);
  }

  /* Using internal pull-up resistors, no addiional resistor needed.
   * Configuring the switch using this method: pin-switch-ground, or just short/unshort it.
   * This causes the pin to read HIGH when the switch is open, instead of the opposite.
   */
  pinMode(calibratepin,INPUT);
  pinMode(prioritypin,INPUT);
  digitalWrite(calibratepin,HIGH);  //Turns on pull-up resistors in the chip
  digitalWrite(prioritypin,HIGH);
  
  delay(10);

  //Priority can only changes at initialization. Just restart Arduino to change
  if (!digitalRead(prioritypin)) priority=lts-1;  //Priority switch. Making priority 1 or lts-1
  else priority=1;
  Serial.println(priority);
}

void loop() {
  if (!digitalRead(calibratepin)) {
    control(-1);
    calibrate();
    return;     //Ignore another command this loop
  }
  if ((temp=ltsens())!=99) control(temp); //Pass the light sensor index into control if ltsens is other than 99.
  else noline();                          //If ltsens returns 99, light sensor doesn't catch anything.
  clcontrol();                            //Process color control
}

int ltsens() {
  //Returns the light index
  //Error handling
  for (int i=0;i<=lts;i++) {
    analogsum[i]+=analogRead(light[i].pin);
    Serial.print(analogsum[i]); Serial.print(' ');
    Serial.print(analogavg[i]); Serial.print(" | ");
  }
  Serial.println();
  counter+=1;
  //Gather 10 data, then average them. Reducing fluctation.
  if (counter>=ltrange) {
    for (int i=0;i<=lts;i++) analogavg[i]=analogsum[i]/counter;
    memset(analogsum,0,sizeof(analogsum));  //Fill with zeros
    counter=0;
  }
  //If there's no 10 new data yet, use the previous
  //Checks light sensors. Starts from the highest priority.
  int i=priority;
  //Remember that priority is either the second from front or back
  //thus only the last middle being evaluated.
  while ((i>=0) and (i<=lts)) {
    if ((analogavg[i]*linec)<(light[i].thold*linec)) return i; //If current sensor is true, directly returns the sensor index.
    if (priority != 1) i--; else i++;
  }
  return 99;  //If nothing found
}

void control(int index) {
  mil[1]=millis();
  if (index==-1) { //Stop command is absolute, no need to wait
    motor(0,0);
    return;
  }
  //Remember that light indexes are 0, .. ,x, .. ,lts-1 or direction-wise leftmost, .. ,middle, .. ,rightmost
  if (mil[1]-prevmil[1]>=controldur) {  //Change control behavior only after a duration of time
    prevmil[1]=mil[1];
    previndex=index;
  } else index=previndex;
  switch (index) {
      case 1: motor(slower,normal); break;  //Leftmost
      case 2: motor(normal,slower); break;  //Rightmost
      default: motor(normal,normal); break; //Middle, 0 and 3 as index
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

void noline() {
  //Process if no line is detected
  for (int i=0;i<delayc;i++) {
    //Delay 10ms before rechecking, thus there'll be a total delay of <10ms x delayc>
    delay(10);
    control(previndex);       //Continue previous action
    if (ltsens()!=99) return; //until it detects new line
    Serial.print("No ");
  }                           //or the specified time passed
  //If loop ends without finding anything
  //turns in the direction of priority until it actually catches anything
  while (ltsens()==99) control(priority);
}

void clcontrol() {
  mil[0] = millis();
  if (mil[0]-prevmil[0]>=clinterval){
    prevmil[0]=mil[0]; //Store prev time it blinked
    //Since only one LED at a time, this is the algorithm used
    digitalWrite(redpin,dored);
    digitalWrite(greenpin,!dored);
    if (docolor) {
      delay(10);
      clsens();
    }
    dored=!dored; //Switch color
  }
}

void clsens() {
  /* If current red light is on and current color sensor passed the red threshold meanshile the
   *  previous color sensor data showed a lower-than-threshold value, then it's genuinely in a red
   *  area. */
  Serial.write(analogRead(color.pin));
  if((dored)&&(analogRead(color.pin)>color.thold)&&(prevcl<color.thold)) {
    //If it's in red area, delay for 4s.
    control(-1);  //Making sure the motors are stopped
    delay(4000);
    //Disable further color control until new session, since there's only one red area.
    docolor=false;
  }
  prevcl=analogRead(color.pin);
}

void calibrate(){
  /* The order is free, but make sure to start with the area that
   * makes all motors stop.
   * Continously moving motor(s) is caused by the noline procedure
   * in which it'll only stop if it's detecting a line.
   */
  /* Calibrating light threshold. Procedure:
   * 1.Face all sensors into one of the areas (dark or light)
   * 2.Turn on switch
   * 3.Make sure no motor is moving
   * 4.If it's still moving, restart the procedure but with the other area
   * 4.Wait until Arduino built-in LED blinks 3 times
   * 5.Face all sensors into another area
   * 6.Turn off switch
   * 7.Wait until robot somehow moves
   */
  Serial.println("Gathering data for calibration.");
  
  //Make sure the sensors are facing the light surface
  int lightavg[lts+1]={0};
  Serial.println("First area");
  getanalogavg(calibrange);
  memcpy(lightavg,analogavg,sizeof(analogavg)); //Copying array

  //Indicator
  while (!digitalRead(calibratepin)){ //Remember it's inverted
    //Waiting for the calibrate switch to be turned off
    //Make sure the sensors are facing the dark surface
    for (int i=0;i<3;i++){  //Blink 3 times
      digitalWrite(13,HIGH);
      delay(10);
      digitalWrite(13,LOW);
      delay(10);
    }
    delay(100);
  }
  
  Serial.println("Second area");
  getanalogavg(calibrange);

  Serial.println("Result:");
  //Setting new light thresholds value, current method: Getting the middle value between averages
  for (int i=0;i<=lts;i++) {
    light[i].thold=(lightavg[i]+analogavg[i])/2;
    Serial.println(light[i].thold);
  }
  
  //Currently, the calibrating evaluation method is pretty simple. Might changes if it isn't good enough.

  //Resetting
  memset(analogsum,0,sizeof(analogsum));  //Fill with zeros
  memset(analogavg,0,sizeof(analogavg));
}

void getanalogavg(int avgrange){  //Fill in analogsum and analogavg
  //Resetting used variables
  memset(analogsum,0,sizeof(analogsum));  //Fill with zeros
  memset(analogavg,0,sizeof(analogavg));
  
  for (int j=0;j<=avgrange;j++) {
    for (int i=0;i<=lts;i++) {
      analogsum[i]+=analogRead(light[i].pin);
      Serial.print(analogsum[i]); Serial.print(' ');
      if (j>=avgrange) analogavg[i]=analogsum[i]/j;
    }
    Serial.println();
    delay(30);
  }
}
