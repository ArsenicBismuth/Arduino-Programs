// Mapping
// IMU-A        PadTest IMU-G   Joy Ardu    Program
// X = Right    Right   Pitch    |  x Ry    ax0 ax4
// Y = Front    Up      Rolll    |  y Rx    ax1 ax3
// Z = Up       Front   Yaw      |  z Rz    ax2 ax5

// I2Cdev and MPU6050 must be installed as libraries, or else the .cpp/.h files
// for both classes must be in the include path of your project
#include "I2Cdev.h"
#include "MPU6050.h"
#include "Joystick.h"

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
    #include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 accelgyro;
//MPU6050 accelgyro(0x69); // <-- use for AD0 high

int16_t ax, ay, az;
int16_t gx, gy, gz;

#define LED_PIN 13
bool blinkState = false;


// Joystick stuff
Joystick_ Joystick;
#define INT16X 32767

// Call Joystick.sendState() to send in manual mode.
const bool testAutoSendMode = true;
bool buts[32];  // Storing buttons states

void setup() {
    delay(1000);
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif

    // initialize serial communication
    Serial.begin(38400);

    // initialize device
    Serial.println("Initializing I2C devices...");
    accelgyro.initialize();

    // verify connection
    Serial.println("Testing device connections...");
    Serial.println(accelgyro.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");

    // use the code below to change accel/gyro offset values
    
    Serial.println("Updating internal sensor offsets...");
    // [-1587,-1587] --> [-15,42]  [1823,1823] --> [-16,119]   [983,983] --> [16381,16399] [96,97] --> [0,2]   [27,28] --> [-3,1]  [-8,-7] --> [-1,3]
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t");
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t");
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t");
    Serial.print("\n");
    accelgyro.setXAccelOffset(-1587);
    accelgyro.setYAccelOffset(1823);
    accelgyro.setZAccelOffset(983);
    accelgyro.setXGyroOffset(96);
    accelgyro.setYGyroOffset(28);
    accelgyro.setZGyroOffset(-8);
    Serial.print(accelgyro.getXAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getYAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getZAccelOffset()); Serial.print("\t");
    Serial.print(accelgyro.getXGyroOffset()); Serial.print("\t");
    Serial.print(accelgyro.getYGyroOffset()); Serial.print("\t");
    Serial.print(accelgyro.getZGyroOffset()); Serial.print("\t");
    Serial.print("\n");

    // Zeroing array (works for char, int, long, and maybe float)
    memset(buts,0,sizeof(buts));
    
    // Set axis ranges
    Joystick.setXAxisRange(-INT16X, INT16X);
    Joystick.setYAxisRange(-INT16X, INT16X);
    Joystick.setZAxisRange(-INT16X, INT16X);
    Joystick.setRxAxisRange(-INT16X, INT16X);
    Joystick.setRyAxisRange(-INT16X, INT16X);
    Joystick.setRzAxisRange(-INT16X, INT16X);
    
    if (testAutoSendMode) Joystick.begin();
    else Joystick.begin(false);

    // configure Arduino LED pin for output
    pinMode(LED_PIN, OUTPUT);
}

void loop() {
    // read raw accel/gyro measurements from device
    accelgyro.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);

    Joystick.setXAxis(ax);  // Right
    Joystick.setYAxis(ay);  // Front
    Joystick.setZAxis(az);  // Up

    Joystick.setRxAxis(gy); // Roll
    Joystick.setRyAxis(gx); // Pitch
    Joystick.setRzAxis(gz); // Yaw

    Joystick.sendState();   // Send them together

    Serial.print("a/g:\t");
    Serial.print(ax); Serial.print("\t");
    Serial.print(ay); Serial.print("\t");
    Serial.print(az); Serial.print("\t");
    Serial.print(gx); Serial.print("\t");
    Serial.print(gy); Serial.print("\t");
    Serial.println(gz);

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
}
