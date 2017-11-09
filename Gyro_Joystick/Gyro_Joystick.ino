#include <Wire.h>

// Check processor type. Other than At32u4, HID won't enabled
#ifdef __AVR_ATmega32U4__
  #include <Joystick.h>
  #define HID
#endif

// Find [Config] tags for configurable codes

/* Configs
	MPU6050
	 SCL -> SCL [ProMicro D3;			    	NANO A5; 	 UNO A5, Above AREF & SDA]
	 SDA -> SDA [ProMicro D2;			    	NANO A4; 	 UNO A4, Above AREF]
	 INT -> INT [ProMicro TXO, RXI, D2, D3, D7; NANO D2, D3; UNO D2, D3]
*/

// [Config] Parameter
#define DEBUG_PROCESSING 1  // Check if debugging using processing
#define RANGE 16383         // Analog Range, peak not peak-to-peak

const int MPU_addr = 0x68;  // I2C address of the MPU-6050

// Variables to store the values from the sensor readings
int16_t ax, ay, az, tm, gx, gy, gz;

// Variables sent as joystick
int x, y;

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

void setup() {
	// Initialize I2C for MPU6050
	Wire.begin();
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x6B); // PWR_MGMT_1 register
	Wire.write(0); // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
	
	#ifdef HID
		// Initialize Joystick
		Joystick.begin();
		
		// Mapping data representing 16-bit Joystick (-32k to 32k)
		Joystick.setXAxisRange(-RANGE, RANGE);
		Joystick.setYAxisRange(-RANGE, RANGE);
    
    Serial.println("At32u4 detected\n");
	#endif
	
	Serial.begin(57600);
  
  delay(1000);
	calibrateSensors();
}

void loop() {
	// Acquiring data
	mpuAcquire();
	
	// Process the joystick data from acquired data
	processJoystick();
	
	#ifdef HID
		// Executing
		Joystick.setXAxis(x);
		Joystick.setYAxis(y);
	#endif

  // Debugging
  sendSerial(DEBUG_PROCESSING);
}

void mpuAcquire() {
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);

	Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
	// Getting 1 byte each reading, appending into 2 bytes of complete data
	ax = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
	ay = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	az = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	tm = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	gx = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	gy = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	gz = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  
  // Each data is int16_t, -32k to 32k
  
  // Process raw data
  processRaw();
  
	delay(10);
}

void processRaw() {
  // [Config] Re-Align based on IMU placement
  // Divided to match needed representation & avoid overflow if necessary
  ax *= (float) -1/2;
  ay *= (float) -1/2;
  az *= (float) -1/2;
  gx *= (float) -1/2;
  gy *= (float) 1/2;
  gz *= (float) 1/2;

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
  // Remove offsets
  ax -= base_x_accel;
  ay -= base_y_accel;
  az -= base_z_accel;
  gx -= base_x_gyro;
  gy -= base_y_gyro;
  gz -= base_z_gyro;
  
  // [Config] Mapping to joystick analog axis
  /*  Just making sure -127 to 127 reached.
    Passing those is preferable, allow 
    for easier editing just by using
    multiplier outside. */

  /* Gyro is used since it's a relative positional (angular velocity), not related to gravity or anything,
    which is what we wanted. Accelerometer becoming very pain in the ass when gravity involved.
    Basically with accel a different tilt will results in different reading even with the same gesture --
    it's just a tilt sensor in its raw form */
    
  x = ay * 0 + gz * 1;
  y = az * 0 + gy * 1;
}

// Simple calibration - just average first few readings to subtract
// from the later data
void calibrateSensors() {
  int num_readings = 10; // [Config] n first numbers sampled

  // Discard the first reading (don't know if this is needed or
  // not, however, it won't hurt.)
  mpuAcquire();
  
  // Read and average the raw values
  for (int i = 0; i < num_readings; i++) {
    mpuAcquire();

    // Division added inside to avoid overflow as long as the raw data isn't overflowed
    base_x_accel += (float) ax / num_readings;
    base_y_accel += (float) ay / num_readings;
    base_z_accel += (float) az / num_readings;
    base_x_gyro += (float) gx / num_readings;
    base_y_gyro += (float) gy / num_readings;
    base_z_gyro += (float) gz / num_readings;

    sendSerial(DEBUG_PROCESSING);
  }
}

void sendSerial(bool processing) {
	// Formatting x:int\ny:int\n
	Serial.print("\nx:");
	Serial.print(x);
  if (processing) Serial.print("\n");
  else Serial.print("\t");
	Serial.print("y:");
	Serial.print(y);

  if(!processing) {
    /*Serial.print("\t");
    Serial.print(ax);
    Serial.print("\t");
    Serial.print(ay);
    Serial.print("\t");
    Serial.print(az);*/
    Serial.print("\t");
    Serial.print(gx);
    Serial.print("\t");
    Serial.print(gy);
    Serial.print("\t");
    Serial.print(gz);

    Serial.print("\t");
    Serial.print(base_x_gyro);
    Serial.print("\t");
    Serial.print(base_y_gyro);
    Serial.print("\t");
    Serial.print(base_z_gyro);
  }
}
