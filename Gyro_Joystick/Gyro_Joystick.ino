#include <Wire.h>
#include <Joystick.h>

/* Configs
	MPU6050
	 SCL -> SCL [ProMicro D3;			    NANO A5; 	 UNO A5, Above AREF & SDA]
	 SDA -> SDA [ProMicro D2;			    NANO A4; 	 UNO A4, Above AREF]
	 INT -> INT [ProMicro TXO, RXI, D2, D3; NANO D2, D3; UNO D2, D3]
*/

const int MPU_addr = 0x68;  // I2C address of the MPU-6050
int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int x, y;

/*	Only 2 axis required for up-down and left-right view
	In actual Joystick, those would be Z-Axis and Z-Rotation respectively.
	But decided to use X-axis and Y-axis for ease of visualization.
*/
Joystick_ Joystick(JOYSTICK_DEFAULT_REPORT_ID,JOYSTICK_TYPE_GAMEPAD,
  0, 0,                  // Button Count, Hat Switch Count
  true, true, false,     // X and Y, but no Z Axis
  false, false, false,   // No Rx, Ry, or Rz
  false, false,          // No rudder or throttle
  false, false, false	 // No accelerator, brake, or steering
);

void setup() {
	// Initialize I2C for MPU6050
	Wire.begin();
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x6B); // PWR_MGMT_1 register
	Wire.write(0); // set to zero (wakes up the MPU-6050)
	Wire.endTransmission(true);
	
	// Initialize Joystick
	Joystick.begin();
	// Joystick analog ranges from -127 to 127
	Joystick.setXAxisRange(-127, 127);
	Joystick.setYAxisRange(-127, 127);
	
	Serial.begin(9600);
}

void loop() {
	// Acquiring data
	mpuAcquire();
	
	// Mapping
	x = AcY * 1 - 0;
	y = AcZ * 1 - 0;
	
	// Executing
	Joystick.setXAxis(x);
	Joystick.setYAxis(y);

	delay(10);
}

void mpuAcquire() {
	Wire.beginTransmission(MPU_addr);
	Wire.write(0x3B); // starting with register 0x3B (ACCEL_XOUT_H)
	Wire.endTransmission(false);
	
	Wire.requestFrom(MPU_addr,14,true); // request a total of 14 registers
	// Getting 1 byte each reading, appending into 2 bytes of complete data
	AcX = Wire.read() << 8 | Wire.read(); // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L) 
	AcY = Wire.read() << 8 | Wire.read(); // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
	AcZ = Wire.read() << 8 | Wire.read(); // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
	Tmp = Wire.read() << 8 | Wire.read(); // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
	GyX = Wire.read() << 8 | Wire.read(); // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
	GyY = Wire.read() << 8 | Wire.read(); // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
	GyZ = Wire.read() << 8 | Wire.read(); // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
	
	// Re-Align based on IMU placement
	AcX *= -1;
	AcY *= -1;
	AcZ *= -1;
	GyX *= -1;
	GyY *= -1;
	GyZ *= -1;
	
	/* Status after re-aligment:
		X = back - front
		Y = left - right
		Z = up - down
		Xrot = clockwise
		Yrot = front roll
		Zrot = steering right
	*/
	
	//delay(333);
}
