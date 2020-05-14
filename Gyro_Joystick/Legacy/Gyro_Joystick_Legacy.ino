
// Find [Config] tags for configurable codes

// I2C device class (I2Cdev) demonstration Arduino sketch for MPU6050 class using DMP (MotionApps v2.0)
// 6/21/2012 by Jeff Rowberg <jeff@rowberg.net>
// Updates should (hopefully) always be available at https://github.com/jrowberg/i2cdevlib

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include <PID_v1.h>
#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;
//MPU6050 mpu(0x69); // <-- use for AD0 high

// [Config] Output mode and general data description

// uncomment "OUTPUT_READABLE_QUATERNION" if you want to see the actual
// quaternion components in a [w, x, y, z] format (not best for parsing
// on a remote host such as Processing or something though)
//#define OUTPUT_READABLE_QUATERNION

// uncomment "OUTPUT_READABLE_EULER" if you want to see Euler angles
// (in degrees) calculated from the quaternions coming from the FIFO.
// Note that Euler angles suffer from gimbal lock (for more info, see
// http://en.wikipedia.org/wiki/Gimbal_lock)
//#define OUTPUT_READABLE_EULER

// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL

// uncomment "OUTPUT_READABLE_REALACCEL" if you want to see acceleration
// components with gravity removed. This acceleration reference frame is
// not compensated for orientation, so +X is always +X according to the
// sensor, just without the effects of gravity. If you want acceleration
// compensated for orientation, us OUTPUT_READABLE_WORLDACCEL instead.
//#define OUTPUT_READABLE_REALACCEL

// uncomment "OUTPUT_READABLE_WORLDACCEL" if you want to see acceleration
// components with gravity removed and adjusted for the world frame of
// reference (yaw is relative to initial orientation, since no magnetometer
// is present in this case). Could be quite handy in some cases.
#define OUTPUT_READABLE_WORLDACCEL

// uncomment "OUTPUT_TEAPOT" if you want output that matches the
// format used for the InvenSense teapot demo
// #define OUTPUT_TEAPOT

#define INTERRUPT_PIN 7  // The only free Interrupt pin for Micro
#define LED_PIN 17 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float euler[3];         // [psi, theta, phi]    Euler angle container
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector

// packet structure for InvenSense teapot demo
uint8_t teapotPacket[14] = { '$', 0x02, 0,0, 0,0, 0,0, 0,0, 0x00, 0x00, '\r', '\n' };

// Check processor type. Other than At32u4, HID won't enabled
#ifdef __AVR_ATmega32U4__
  #include <Joystick.h>
  #define HID
#endif

#define INT16X 32767.0      // Maximum value of INT16

/* Configs
	MPU6050
	 SCL -> SCL [ProMicro D3;			    	NANO A5; 	 UNO A5, Above AREF & SDA]
	 SDA -> SDA [ProMicro D2;			    	NANO A4; 	 UNO A4, Above AREF]
	 INT -> INT [ProMicro TXO, RXI, D2, D3, D7; NANO D2, D3; UNO D2, D3]
*/

// [Config] Parameter
#define DEBUG_PROCESSING 1  // Check if debugging using processing
#define DISTANCE 2          // Distance for sine calculation. Basically just a multiplier.
#define DEADP 10            // Deadzone in percent
// Using define gives a very grave bug, totally wrong calculation results
const double RANGE = INT16X/2;      // Analog Range, peak not peak-to-peak
const double CLIP_D = 90;           // Clip to -CLIP_D to CLIP_D in degrees
const double CLIP_A = INT16X/4;     // Clip to -CLIP_A to CLIP_A in acceleration unit
                                    // Acceleration is given in [-int16, int16] that maps [-2g, 2g]
#define HOLDPREV 0          // How long (loop) previous data is kept. Giving higher magnitude for longer gesture
#define HOLDMODE 0          // Enable hold mode, thus equivalent to direct YPR data (not relative to previous). Equiv to inf HOLDPREV
#define HOLDOUT 0           // How long joystick output is kept, outputting the max. Significantly reduce jittering by HOLDPREV

// [Config] PID
#define SAMPLETIME 5        // Set ki to 1 (else 0) and see how good the pids match the input
double kp = 0, ki = 3, kd = 0;  // Init tunings
double setpoint = 0;        // Since only the deviation used here
double in_pid_x, in_pid_y, out_pid_x, out_pid_y;
PID pidx(&in_pid_x, &out_pid_x, &setpoint, kp, ki, kd, REVERSE);
PID pidy(&in_pid_y, &out_pid_y, &setpoint, kp, ki, kd, REVERSE);

const int MPU_addr = 0x68;  // I2C address of the MPU-6050

// Variables to store the values from the sensor readings
VectorFloat a = {0, 0, 0};
VectorFloat g = {0, 0, 0};  // Store relative ypr data, current - previous

// Previous management variables
VectorFloat pa = {0, 0, 0};
VectorFloat pg = {0, 0, 0}; // Store previous ypr data, so != previous g since g is curr - prev. This is prev raw DMP
bool start = 1;
int loop_counter1 = HOLDPREV;
int loop_counter2 = HOLDOUT;

// Variables sent as joystick
int x, y;
int px = 0, py = 0;

//  Use the following global variables 
//  to calibrate the gyroscope sensor and accelerometer readings
float base_x_gyro = 0;
float base_y_gyro = 0;
float base_z_gyro = 0;
float base_x_accel = 0;
float base_y_accel = 0;
float base_z_accel = 0;

/*	Only 2 axis required for up-down and left-right view
	In actual Joystick, those would be Z-Axis and Z-Rotation respectively.
	But decided to use X-axis and Y-axis for ease of visualization.
*/
#ifdef HID
	Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
	  0, 0,                  // Button Count, Hat Switch Count
	  true, true, false,     // X and Y, but no Z Axis
	  false, false, false,   // No Rx, Ry, or Rz
	  false, false,          // No rudder or throttle
	  false, false, false	 // No accelerator, brake, or steering
	);
#endif

// Interrupt detection routine

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
    mpuInterrupt = true;
}



void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
  #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
      Wire.begin();
      Wire.setClock(400000); // 400kHz I2C clock. Comment this line if having compilation difficulties
  #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
      Fastwire::setup(400, true);
  #endif

  // initialize serial communication
  // (115200 chosen because it is required for Teapot Demo output, but it's
  // really up to you depending on your project)
  Serial.begin(115200);
  //while (!Serial); // wait for Leonardo enumeration, others continue immediately

  // NOTE: 8MHz or slower host processors, like the Teensy @ 3.3V or Arduino
  // Pro Mini running at 3.3V, cannot handle this baud rate reliably due to
  // the baud timing being too misaligned with processor ticks. You must use
  // 38400 or slower in these cases, or use some kind of external separate
  // crystal solution for the UART timer.

  // initialize device
  Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  Serial.println(F("Testing device connections..."));
  Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  while (Serial.available() && Serial.read()); // empty buffer
  // while (!Serial.available());                 // wait for data
  while (Serial.available() && Serial.read()); // empty buffer again

  // load and configure the DMP
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // [Config] supply your own gyro offsets here, scaled for min sensitivity
  // Latest update: 04/02/18 02:08 Fully based on IMU_Zero, everything
  // Yes, really. Just by changing something out of ordinary will mess the others.
  mpu.setXAccelOffset(365);
  mpu.setYAccelOffset(4426);
  mpu.setZAccelOffset(5802);
  mpu.setXGyroOffset(94);
  mpu.setYGyroOffset(16);
  mpu.setZGyroOffset(4);

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // turn on the DMP, now that it's ready
    Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    Serial.println(F("Enabling interrupt detection (Arduino external interrupt 0)..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);

  // PID
  pidx.SetOutputLimits(-RANGE, RANGE);
  pidy.SetOutputLimits(-RANGE, RANGE);
  pidx.SetSampleTime(SAMPLETIME);   // Set ki to 1 (else 0) and see how good the pids match the input
  pidy.SetSampleTime(SAMPLETIME);
  
  // Turn the PID on
  pidx.SetMode(AUTOMATIC);
  pidy.SetMode(AUTOMATIC);

  // Setup Joystick HID
	#ifdef HID
		// Initialize Joystick
		Joystick.begin();
		
		// Mapping data representing 16-bit Joystick (-32k to 32k)
		Joystick.setXAxisRange(-RANGE, RANGE);
		Joystick.setYAxisRange(-RANGE, RANGE);
    
    Serial.println("At32u4 detected\n");
	#endif
  
  delay(5000);
}



void loop() {
	// Acquiring data
	mpuAcquire();

  // [Config] Map DMP data to used data, based input type
  // IT MUST BE CHECKED FROM SEEING DATA,
  // because for ex can't differentiate gy with gx for which is which
  // since you don't know where front is pointing at
  in_pid_x = deadzone(aaWorld.y, 10, RANGE);  // Determine which data used in PID input for x-axis movement
  in_pid_y = deadzone(aaWorld.z, 10, RANGE);
  pidx.Compute();
  pidy.Compute();
  
  a = {0, out_pid_x, out_pid_y};
  g = {ypr[2] * 180/M_PI - pg.x,
      ypr[1] * 180/M_PI - pg.y,
      ypr[0] * 180/M_PI - pg.z};
  
  // Basically g will contains the change of ypr in degrees

//  Serial.print("\trgz");
//  Serial.print(g.z);
//  Serial.print("\trgy");
//  Serial.print(g.y);
//  Serial.print("\t");
  
  processData();
  
	// Process the joystick data from acquired data
	processJoystick();

  Serial.print("pidx\t");
  Serial.print(out_pid_x);
  Serial.print("\tpidy\t");
  Serial.print(out_pid_y);
  Serial.print("\t");

	#ifdef HID
		// Executing
		Joystick.setXAxis(x);
		Joystick.setYAxis(y);
	#endif

  // How long before a previous reference updated
  if ((loop_counter1 <= 0) || (HOLDMODE && start)) {
    pa = {a.x, a.y, a.z};
    pg = {ypr[2] * 180/M_PI,
          ypr[1] * 180/M_PI,
          ypr[0] * 180/M_PI};
    start = 0;
    loop_counter1 = HOLDPREV;
  } else 
    if (!HOLDMODE) loop_counter1--;

  // Debugging
  //sendSerial(DEBUG_PROCESSING);
}



void processData() {
  // Avoid changing 179 to -179 when it's actually 2 deg change
  if (pg.x >= 90 && (ypr[2] * 180/M_PI) <= -90) g.x += 360;
  else if (pg.x < -90 && (ypr[2] * 180/M_PI) > 90) g.x -= 360;
  
  if (pg.y >= 90 && (ypr[1] * 180/M_PI) <= -90) g.y += 360;
  else if (pg.y < -90 && (ypr[1] * 180/M_PI) > 90) g.y -= 360;
  
  if (pg.z >= 90 && (ypr[0] * 180/M_PI) <= -90) g.z += 360;
  else if (pg.z < -90 && (ypr[0] * 180/M_PI) > 90) g.z -= 360;

  // Getting a translation reaction through a rotating motion, sin angle is in radian
  /*g.x = sin(g.x * M_PI/180) * DISTANCE;
  g.y = sin(g.y * M_PI/180) * DISTANCE;
  g.z = sin(g.z * M_PI/180) * DISTANCE;*/

  a.x = clip(a.x, -CLIP_A, CLIP_A);
  a.y = clip(a.y, -CLIP_A, CLIP_A);
  a.z = clip(a.z, -CLIP_A, CLIP_A);
  g.x = clip(g.x, -CLIP_D, CLIP_D);
  g.y = clip(g.y, -CLIP_D, CLIP_D);
  g.z = clip(g.z, -CLIP_D, CLIP_D);

  // [Config] Re-Align based on IMU placement
  // Divided to match needed representation & avoid overflow if necessary
  // Basically [-CLIP, CLIP] * RANGE / CLIP = [-RANGE, RANGE]
  
  a.x *= -1 * RANGE / CLIP_A;
  a.y *= -1 * RANGE / CLIP_A;
  a.z *= -1 * RANGE / CLIP_A;
  g.x *= 1 * RANGE / CLIP_D;
  g.y *= 1 * RANGE / CLIP_D;
  g.z *= 1 * RANGE / CLIP_D;

  // DirectInput positive is bottom right corner
  /* Status after re-aligment:
    X = back - front
    Y = left - right
    Z = up - down
    Xrot = clockwise
    Yrot = front roll
    Zrot = steering right
  */
}



void processJoystick() {
  // Remove offsets post internal offsets
  //  ax -= base_x_accel;
  //  ay -= base_y_accel;
  //  az -= base_z_accel;
  //  gx -= base_x_gyro;
  //  gy -= base_y_gyro;
  //  gz -= base_z_gyro;
  
  // [Config] Mapping to joystick analog axis
  /* Gyro is used since it's a relative positional (angular velocity), not related to gravity or anything,
    which is what we wanted. Accelerometer becoming very pain in the ass when gravity involved.
    Basically with accel a different tilt will results in different reading even with the same gesture --
    it's just a tilt sensor in its raw form */
  /* Turns out accelerometer data is very beautiful now after setting using the correct calibration (offsets) */
  
  x = a.y * 1 + g.z * 0;
  y = a.z * 1 + g.y * 0;

//  Serial.print("gz");
//  Serial.print(g.z);
//  Serial.print("\tgy");
//  Serial.print(g.y);
//  Serial.print("\tpgz");
//  Serial.print(pg.z);
//  Serial.print("\tpgy");
//  Serial.print(pg.y);
//  Serial.print("\t");

  x = deadzone(x, DEADP, RANGE);
  y = deadzone(y, DEADP, RANGE);

  if (loop_counter2 <= 0) {
    px = x;
    py = y;
    loop_counter2 = HOLDOUT;
  } else {
    // Check if bigger (magnitude)
    if (abs(px) > abs(x)) x = px;
    if (abs(py) > abs(y)) y = py;
    loop_counter2 --;
  }

  Serial.print(x);
  Serial.print('\t');
  Serial.print(y);
  Serial.print('\t');
}



// Using double to allow for full integer coverage
double clip(double val, double clipDown, double clipUp) {
  if (val > clipUp) return clipUp;
  else if (val < clipDown) return clipDown;
  else return val;
}

double deadzone(double val, double dead, double range) {
  // Deadzone in percent, 0.1*range => 10
  dead = dead / 2;
  
  if (abs(val) * 100 / range <= dead) {
    return 0;
  } else {
    // value * equivalent value percent in new range
    return val / abs(val) * (abs(val) - dead / 100.0 * range) * 50.0 / (50.0 - dead);
  }
}



void mpuAcquire() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;

  // wait for MPU interrupt or extra packet(s) available
  while (!mpuInterrupt && fifoCount < packetSize) {
    // other program behavior stuff here
    // .
    // if you are really paranoid you can frequently test in between other
    // stuff to see if mpuInterrupt is true, and if so, "break;" from the
    // while() loop to immediately process the MPU data
  }

  // reset interrupt flag and get INT_STATUS byte
  mpuInterrupt = false;
  mpuIntStatus = mpu.getIntStatus();

  // get current FIFO count
  fifoCount = mpu.getFIFOCount();

  // check for overflow (this should never happen unless our code is too inefficient)
  if ((mpuIntStatus & 0x10) || fifoCount == 1024) {
    // reset so we can continue cleanly
    mpu.resetFIFO();
    Serial.println(F("FIFO overflow!"));

  // otherwise, check for DMP data ready interrupt (this should happen frequently)
  } else if (mpuIntStatus & 0x02) {
    // wait for correct available data length, should be a VERY short wait
    while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();

    // read a packet from FIFO
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    
    // track FIFO count here in case there is > 1 packet available
    // (this lets us immediately read more without waiting for an interrupt)
    fifoCount -= packetSize;

    #ifdef OUTPUT_READABLE_QUATERNION
      // display quaternion values in easy matrix form: w x y z
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      Serial.print("quat\t"); Serial.print(q.w);
      Serial.print("\t");
      Serial.print(q.x);
      Serial.print("\t");
      Serial.print(q.y);
      Serial.print("\t");
      Serial.print(q.z);
      Serial.print("\t");
    #endif

    #ifdef OUTPUT_READABLE_EULER
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetEuler(euler, &q);
      Serial.print("euler\t");
      Serial.print(euler[0] * 180/M_PI);
      Serial.print("\t");
      Serial.print(euler[1] * 180/M_PI);
      Serial.print("\t");
      Serial.print(euler[2] * 180/M_PI);
      Serial.print("\t");
    #endif

    #ifdef OUTPUT_READABLE_YAWPITCHROLL
      // display Euler angles in degrees
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
//      Serial.print("ypr\t");
//      Serial.print(ypr[0] * 180/M_PI);
//      Serial.print("\t");
//      Serial.print(ypr[1] * 180/M_PI);
//      Serial.print("\t");
//      Serial.print(ypr[2] * 180/M_PI);
//      Serial.print("\t");
    #endif

    #ifdef OUTPUT_READABLE_REALACCEL
      // display real acceleration, adjusted to remove gravity
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      Serial.print("areal\t");
      Serial.print(aaReal.x);
      Serial.print("\t");
      Serial.print(aaReal.y);
      Serial.print("\t");
      Serial.print(aaReal.z);
      Serial.print("\t");
    #endif

    #ifdef OUTPUT_READABLE_WORLDACCEL
      // display initial world-frame acceleration, adjusted to remove gravity
      // and rotated based on known orientation from quaternion
      mpu.dmpGetQuaternion(&q, fifoBuffer);
      mpu.dmpGetAccel(&aa, fifoBuffer);
      mpu.dmpGetGravity(&gravity, &q);
      mpu.dmpGetLinearAccel(&aaReal, &aa, &gravity);
      mpu.dmpGetLinearAccelInWorld(&aaWorld, &aaReal, &q);
      Serial.print("aworld\t");
      Serial.print(aaWorld.x);
      Serial.print("\t");
      Serial.print(aaWorld.y);
      Serial.print("\t");
      Serial.print(aaWorld.z);
      Serial.print("\t");
    #endif

    #ifdef OUTPUT_TEAPOT
      // display quaternion values in InvenSense Teapot demo format:
      teapotPacket[2] = fifoBuffer[0];
      teapotPacket[3] = fifoBuffer[1];
      teapotPacket[4] = fifoBuffer[4];
      teapotPacket[5] = fifoBuffer[5];
      teapotPacket[6] = fifoBuffer[8];
      teapotPacket[7] = fifoBuffer[9];
      teapotPacket[8] = fifoBuffer[12];
      teapotPacket[9] = fifoBuffer[13];
      Serial.write(teapotPacket, 14);
      teapotPacket[11]++; // packetCount, loops at 0xFF on purpose
    #endif

    Serial.println();

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }
}
