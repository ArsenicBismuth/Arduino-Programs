//Hardware specs
const int lightpin[]={1,2,3};       //Left, middle, & right light sensors (analog)
const int lts=sizeof(lightpin)/sizeof(int)-1;   //Total element in lightpin minus by 1
const int motorpin[]={5,6,9,10};    //Motors (PWM, digital)
const int colorpin=5;               //Color sensor (analog)
const int redpin=8;                 //red LED (digital)
const int greenpin=7;               //green LED (digital)
const int prioritypin=2;            //Priority switch (digital)
const int calibratepin=3;           //Calibrate switch or button (digital)

//Settings
const int speedratio=10;      //0 until 10
const int turnc=150;          //The sharpness of turning. Range of 0-510, with 510 being centered turn and 0 being straight
const int delayc=50;          //The repetition of 10ms delay before turning when no line is detected. Ex for dotted lines
const int clinterval=60;      //interval of coloured led blink (ms)
const int linec=1;            //1 or -1, -1 for white line
const int ltrange=20;         //The count of light sensor data evaluated before averaged, remember 32767.
const int calibrange=30;      //The count of light sensor data evaluated for calibrating, remember 32767.
const bool idle=false;        //For debugging lights, turning off motor
const int controldur=200;     //Duration for control procedure to change it's behavior

//Calibration
int whitethreshold[lts+1]={980,375,980};  //Left, middle, and right threshold respectively
int redthreshold=200;

//Calculation variables
int analogsum[lts+1]={0};
int analogavg[lts+1]={999};
int normal=255*speedratio/10;           //Maximum speed tweaked by speedratio
int slower=(255-turnc)*speedratio/10;   //Reduced maximum speed for turning purpose

int priority=0;                         //Light sensor priority, refers to the index from lightpin
int temp;
int dored=true;
bool docolor=false;
int prevcl=redthreshold+1;
int counter=0;
int previndex=-1;
unsigned long prevmil[2]={0};
unsigned long mil[2]={0};

void setup() {
  Serial.begin(9600);
  Serial.println();
  pinMode(redpin,OUTPUT);
  pinMode(greenpin,OUTPUT); 

  /* Using internal pull-up resistors, no addiional resistor needed.
   * Configuring the switch using this method: pin-switch-ground, or just short/unshort it.
   * This causes the pin to read HIGH when the switch is open, instead of the opposite.
   */
  pinMode(calibratepin,INPUT);
  pinMode(prioritypin,INPUT);
  digitalWrite(calibratepin,HIGH);  //Turns on pull-up resistors in the chip
  digitalWrite(prioritypin,HIGH);
  
  delay(10);
  priority=lts*!digitalRead(prioritypin);  //Priority switch. Making priority 0 or lts
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
    analogsum[i]+=analogRead(lightpin[i]);
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
  while ((i>=0) and (i<=lts)) {
    if ((analogavg[i]*linec)<(whitethreshold[i]*linec)) return i; //If current sensor is true, directly returns the sensor index.
    if (priority == lts) i--; else i++;
  }
  return 99;  //If nothing found
}

void control(int index) {
  mil[1]=millis();
  //Remember that light indexes are 0, .. ,x, .. ,lts-1 or direction-wise leftmost, .. ,middle, .. ,rightmost
  if (mil[1]-prevmil[1]>=controldur) {  //Change control behavior only after a duration of time
    prevmil[1]=mil[1];
    previndex=index;
  } else index=previndex;
  switch (index) {
      case -1: motor(0,0); break;           //Stop command
      case 0: motor(slower,normal); break;  //Leftmost
      case 2: motor(normal,slower); break;  //Rightmost
      default: motor(normal,normal); break; //Middle, or 1 as index
  }
}

void motor(int left,int right) {
  Serial.print(left);Serial.print(' ');Serial.print(right);Serial.print(' ');
  if (idle) return;
  if (left>=0) {
    analogWrite(motorpin[0], abs(left));  //PWM output, allowing speed control
    digitalWrite(motorpin[1], LOW);       //Turns off the opposing H-Bridge gate
  } else {
    analogWrite(motorpin[1], abs(left));  //Similar with above
    digitalWrite(motorpin[0], LOW);
  }
  if (right>=0) {
    analogWrite(motorpin[2], abs(right));
    digitalWrite(motorpin[3], LOW);
  } else {
    analogWrite(motorpin[3], abs(right));
    digitalWrite(motorpin[2], LOW);
  }
}

void noline() {
  //Process if no line is detected
  for (int i=0;i<delayc;i++) {
    //Delay 10ms before rechecking, thus there'll be a total delay of <10ms x delayc>
    delay(10);
    control(1);
    if (ltsens()!=99) return; //If any light sensor catches dark area, directly stops
    Serial.print("No ");
  }
  //If loop end without finding anything
  //Turn in the direction of priority until it actually catches anything
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
  Serial.write(analogRead(colorpin));
  if((dored)&&(analogRead(colorpin)>redthreshold)&&(prevcl<redthreshold)) {
    //If it's in red area, delay for 4s.
    control(-1);  //Making sure the motors are stopped
    delay(4000);
    //Disable further color control until new session, since there's only one red area.
    docolor=false;
  }
  prevcl=analogRead(colorpin);
}

void calibrate(){
  /* Calibrating light threshold. Procedure:
   * 1.Put in light area
   * 2.Turn on switch
   * 3.Wait until Arduino built-in LED blinks 3 times
   * 4.Put in dark area
   * 5.Turn off switch
   * 6.Wait until robot somehow moves
   */
  Serial.println("Gathering data for calibration.");
  
  //Make sure the sensors are facing the light surface
  int lightavg[lts+1]={0};
  Serial.println("Light area");
  getanalogavg(calibrange);
  memcpy(lightavg,analogavg,sizeof(analogavg)); //Copying array

  //Indicator
  while (!digitalRead(calibratepin)){ //Remember it's inverted
    //Waiting for the calibrate switch to be turned off
    //Make sure the sensors are facing the dark surface
    for (int i=0;i<3;i++){  //Blink 3 times
      digitalWrite(13,HIGH);
      delay(20);
      digitalWrite(13,LOW);
      delay(20);
    }
    delay(500);
  }
  
  Serial.println("Dark area");
  getanalogavg(calibrange);

  Serial.println("Result:");
  //Setting new light thresholds value, current method: Getting the middle value between averages
  for (int i=0;i<=lts;i++) {
    whitethreshold[i]=(lightavg[i]+analogavg[i])/2;
    Serial.println(whitethreshold[i]);
  }
  
  //Currently, the calibrating evaluation method is pretty simple. Might changes if it isn't good enough.

  //Resetting
  memset(analogsum,0,sizeof(analogsum));  //Fill with zeros
  memset(analogavg,0,sizeof(analogavg));
}

void getanalogavg(int avgrange){
  //Resetting used variables
  memset(analogsum,0,sizeof(analogsum));  //Fill with zeros
  memset(analogavg,0,sizeof(analogavg));
  
  for (int j=0;j<=avgrange;j++) {
    for (int i=0;i<=lts;i++) {
      analogsum[i]+=analogRead(lightpin[i]);
      Serial.print(analogsum[i]); Serial.print(' ');
      if (j>=calibrange) analogavg[i]=analogsum[i]/j;
    }
    Serial.println();
  }
}
