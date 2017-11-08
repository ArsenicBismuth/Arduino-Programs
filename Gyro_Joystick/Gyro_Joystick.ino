#include <Wire.h>

// Check processor type. Other than At32u4, HID won't enabled
#ifdef __AVR_ATmega32u4__
  #include <Joystick.h>
  #define HID
#endif

// Find [Config] tags for configurable codes

/* Configs
	MPU6050
	 SCL -> SCL [ProMicro D3;			    NANO A5; 	 UNO A5, Above AREF & SDA]
	 SDA -> SDA [ProMicro D2;			    NANO A4; 	 UNO A4, Above AREF]
	 INT -> INT [ProMicro TXO, RXI, D2, D3; NANO D2, D3; UNO D2, D3]
*/

const int MPU_addr = 0x68;  // I2C address of the MPU-6050

// Variables to store the values from the sensor readings
int16_t ax, ay, az, tm, gx, gy, gz;

// Variables sent as joystick
int x, y;

//  Use the following global variables 
//  to calibrate the gyroscope sensor and accelerometer readings
int base_x_gyro = 0;
int base_y_gyro = 0;
int base_z_gyro = 0;
int base_x_accel = 0;
int base_y_accel = 0;
int base_z_accel = 0;

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
		// Joystick analog ranges from -127 to 127
		Joystick.setXAxisRange(-127, 127);
		Joystick.setYAxisRange(-127, 127);
	#endif
	
	Serial.begin(57600);
	
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
	sendSerial();
	
	delay(10);
}

void processJoystick() {
	// Remove offsets and scale gyro data  
	gx -= base_x_gyro;
	gy -= base_y_gyro;
	gz -= base_z_gyro;
	ax -= base_x_accel;
	ay -= base_y_accel;
	az -= base_z_accel;
	
	// [Config] Mapping to joystick analog axis
	/*	Just making sure -127 to 127 reached.
		Passing those is preferable, allow 
		for easier editing just by using
		multiplier outside
	*/
	x = ay * 1 + gz * 0;
	y = az * 1 + gy * 0;
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
	
	// [Config] Re-Align based on IMU placement
	ax *= -1;
	ay *= -1;
	az *= -1;
	gx *= -1;
	gy *= -1;
	gz *= -1;
	
	/* Status after re-aligment:
		X = back - front
		Y = left - right
		Z = up - down
		Xrot = clockwise
		Yrot = back roll
		Zrot = steering right
	*/
	
	//delay(333);
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
    base_x_gyro += gx;
    base_y_gyro += gy;
    base_z_gyro += gz;
    base_x_accel += ax;
    base_y_accel += ay;
    base_y_accel += az;
  }
  
  base_x_gyro /= num_readings;
  base_y_gyro /= num_readings;
  base_z_gyro /= num_readings;
  base_x_accel /= num_readings;
  base_y_accel /= num_readings;
  base_z_accel /= num_readings;
}

void sendSerial() {
	// Formatting x:int:y:int
	Serial.print("x:");
	Serial.print(x);
	Serial.print("y:");
	Serial.print(y);
}
