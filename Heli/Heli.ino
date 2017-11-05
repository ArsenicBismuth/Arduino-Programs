//Libraries
#include <SoftwareSerial.h>
#include <PID_v1.h>
#include <Wire.h>

/*The output scale for any setting is [-32768, +32767] for each of the six axes. The 
default setting in the I2Cdevlib class is +/- 2g for the accel and +/- 250 deg/sec for 
the gyro. If the device is perfectly level and not moving, then:
  X/Y accel axes should read 0
  Z accel axis should read 1g, which is +16384 at a sensitivity of 2g
  X/Y/Z gyro axes should read 0
In reality, the accel axes won't read exactly 0 since it is difficult to be perfectly 
level and there is some noise/error, and the gyros will also not read exactly 0 for the 
same reason (noise/error).
*/

//Structures

//Hardware specs
const int motorpin[] {9, 10, 13, 14}; //Motors (PWM). CW blade (Top), CCW blade, tail up, tail down
const int MPU_addr = 0x68;  //I2C address of the MPU-6050
int16_t sen[7]; //AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ

//Remapping sensor, both gyro and accelero
const int rex = -1;
const int rey = -1;
const int rez = 1;
/*Raw Layout
   <-- X *Z   Front -->
        Y
        |
        V
 *Remapped
   /\
   |
   Y
   *Z X -- Front -->
*/

//App specs - Half of input range (avoiding negative)
const int midup = 500;  //There won't be negative since descending uses gravity alone
const int midyaw = 500; //Rotate along Z-axis
const int midptc = 500; //Rotate along Y-axis
//Rotate along X-axis, applicable only on heli with swashplate

//Settings
const bool idle = false;    //For debugging, turning off motor
const double kp = 1;
const double ki = 0;
const double kd = 0;
int up = 0;                 //0 until mxup
int yaw = 0;                //Ex reduce clockwise blade speed to rotate counter-clockwise
int ptc = 0;                //Rotate forward backward

//Calculation
double ac[3][3];
double gy[3][3];
PID pids[] = {
  //PID for accelerometer (input, output, setpoint)
  PID /*accx*/(&ac[0][0], &ac[0][1], &ac[0][2], kp, ki, kd, DIRECT), //Forward-backward
  PID /*accy*/(&ac[1][0], &ac[1][1], &ac[1][2], kp, ki, kd, DIRECT), //Fully usable only if there's side movement
  PID /*accz*/(&ac[2][0], &ac[2][1], &ac[2][2], kp, ki, kd, DIRECT), //Vertical

  //PID for gyro
  PID /*gyrox*/(&gy[0][0], &gy[0][1], &gy[0][2], kp, ki, kd, DIRECT),  //Fully usable if there's roll movement
  PID /*gyroy*/(&gy[1][0], &gy[1][1], &gy[1][2], kp, ki, kd, DIRECT),  //Pitch
  PID /*gyroz*/(&gy[2][0], &gy[2][1], &gy[2][2], kp, ki, kd, DIRECT),  //Yaw
};
/*System (Mostly)
   -Setpoint > Sensor, => Bigger output
   -Setpoint < Sensor, => Smaller output
   -Except Yaw, which reduces opposing motor
 *Direct: Sensor > Setpoint => Decrease (Thus matched)
*/

int code = 0;
String s;
unsigned long prevmil[2] = {0};
unsigned long mil[2] = {0};

void setup() {
  //Sensors Setup
  Wire.begin();
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);

  //PID Setup
  for (int i=0; i<sizeof(pids)/sizeof(PID); i++) {
    pids[i].SetMode(AUTOMATIC);
    pids[i].SetOutputLimits(-255, 255);
  }
  //Setting PIDs setpoint
  for (int i=0; i<3; i++) {
    gy[i][2]=0;
    ac[i][2]=0;
  }

  Serial.begin(9600); //For native USB port only

  pinMode(13, OUTPUT); //Indicator light
  for (int i = 0; i < sizeof(motorpin) / sizeof(const int); i++) pinMode(motorpin[i], OUTPUT);
  delay(10);
}

void loop() {
  Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr, 14, true); // request a total of 14 registers
  for (int i=0; i<7; i++){
     //AcX, AcY, AcZ, Tmp, GyX, GyY, GyZ
    sen[i]=Wire.read() << 8 | Wire.read();
  }

  //Getting data, directions remapped
  ac[0][0]=sen[0]*rex;
  ac[1][0]=sen[1]*rey;
  ac[2][0]=sen[2]*rez;
  gy[0][0]=sen[4]*rex;
  gy[1][0]=sen[5]*rey;
  gy[2][0]=sen[6]*rez;

  //Getting command
  while (Serial.available()) {
    char c = Serial.read();             //Store chars
    s.concat(c);                        //Combine chars received
    switch (c) {
      case /*gy-*/ 'f': gy[1][2]=s.toInt()-midptc; Serial.print(s); s=""; break;
      case /*gz-*/ 'r': gy[2][2]=s.toInt()-midyaw; Serial.print(s); s=""; break;
      case /*az+*/ 'a': ac[2][2]=s.toInt()-midup; Serial.print(s); s=""; break;
    }
  }

  //Process
  for (int i=0; i<sizeof(pids)/sizeof(PID); i++) {
    pids[i].Compute();
  }

  //Debugging
  Serial.print("Input"); Serial.print("Output"): Serial.println("Setpoint");
  for (int i=0; i<3; i++){
    Serial.print("ac"); Serial.print(i); Serial.print(ac[i][0]); Serial.print(" "):
      Serial.print(ac[i][1]); Serial.print(" "): Serial.print(ac[i][2]); Serial.print(" ");
    Serial.print("gy"); Serial.print(i); Serial.print(gy[i][0]); Serial.print(" "):
      Serial.print(gy[i][1]); Serial.print(" "): Serial.println(gy[i][2]);
  }

  //Reaction
  Serial.println();
  control(code);                        //Default code = 0
}

/*Reminder
 *AcX, AcY, AcZ, GyX, GyY, GyZ
 *ac0, ac1, ac2, gy0, gy1, gy2
 *PID (input, output, setpoint)
 *Layout
   /\
   |
   Y
   *Z X -- Front -->
 *Motors: CW blade (Top), CCW blade, tail up, tail down
 *Process
	-Read sensor data
	-Input 0 - 1000, received as (-500) - 500 => Setpoint
	-PID Process, output all real. Must be tuned so that it'll give perfect value for every condition.
	-Output > 255 is fine, since it'll treated as 255. This is where tuning comes.
	-Different case for negative and positive value
	-PWM outputs PID without any reprocessing (except abs(x))
 *System:
	-Setpoint > Sensor, => Bigger output
	-Setpoint < Sensor, => Smaller output
*/

void control(int index) {
  switch (index) {
    case -1: motor(0, 0, 0); return;         //Stop
  }
  //PID output<0 or smaller means it's asking you to reverse
  
  //Forward-backward
  if (ac[0][1]<0){
	analogWrite(motorpin[2],abs(ac[0][1]));
	digitalWrite(motorpin[3],LOW);
  } else {
	analogWrite(motorpin[3],abs(ac[0][1]));
	digitalWrite(motorpin[2],LOW); 
  }
  
  //Pitch
  if (gy[1][1]<0){
	analogWrite(motorpin[2],abs(gy[1][1]));
	digitalWrite(motorpin[3],LOW);
  } else {
	analogWrite(motorpin[3],abs(gy[1][1]));
	digitalWrite(motorpin[2],LOW); 
  }
  
  //Vertical
  if (ac[2][1]<0){
	//Fall by gravity
	  digitalWrite(motorpin[0],LOW);
	  digitalWrite(motorpin[1],LOW);
  } else {
	  analogWrite(motorpin[0],abs(ac[2][1]));
	  analogWrite(motorpin[1],abs(ac[2][1]));
  }
  
  //Yaw
  if (gy[2][1]<0){
	  analogWrite(motorpin[0],255);
	  analogWrite(motorpin[1],abs(gy[2][1]));
  } else {
	  analogWrite(motorpin[0],abs(gy[2][1]));
	  analogWrite(motorpin[1],255);  
  }
}
