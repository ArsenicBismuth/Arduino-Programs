/* The average rotary encoder has three pins, seen from front: A C B
   Clockwise rotation A(on)->B(on)->A(off)->B(off)
   CounterCW rotation B(on)->A(on)->B(off)->A(off)
   
   Rotary Enc Ref : https://playground.arduino.cc/Main/RotaryEncoders
*/

#include <TimerOne.h>

// usually the rotary encoders three pins have the ground pin in the middle
enum PinAssignments {
  encPinA = 2,   // right
  encPinB = 3,   // left
  outPWM = 5,
  dirA = 9,
  dirB = 8
};

volatile unsigned long encPos = 0;  // a counter for the dial
unsigned long lastReportedPos = 1;  // change management
static boolean rotating = false;    // debounce management

float ang = 0.0;
float spd = 0;

volatile int dc = 0;       // Duty-cycle
volatile int sp = 0;        // Set-point (RPM)
volatile float kp = 1;      // proportional
volatile float ki = 0;      // sum
volatile float kd = 0;      // difference

volatile float e = 0;       // Error        
volatile float e_prev = 0;  // Previous error
volatile float e_sum = 0;   // Error sum

unsigned long cmillis = 0;          // Current time
unsigned long pmillis = 0;          // Previous time

// interrupt service routine vars
boolean A_set = false;
boolean B_set = false;

void setup() {
    pinMode(encPinA, INPUT);
    pinMode(encPinB, INPUT);
    pinMode(outPWM, OUTPUT);
    pinMode(dirA, OUTPUT);
    pinMode(dirB, OUTPUT);
    
    // turn on pullup resistors
    digitalWrite(encPinA, HIGH);
    digitalWrite(encPinB, HIGH);

    // H-Bridge direction
    digitalWrite(dirA, HIGH);
    digitalWrite(dirB, LOW);
    
    // encoder pin on interrupt 0 (pin 2)
    attachInterrupt(0, doencA, CHANGE);
    // encoder pin on interrupt 1 (pin 3)
    attachInterrupt(1, doencB, CHANGE);

    Timer1.initialize(1000);            // Time in us
    Timer1.attachInterrupt(pidCalc);    // Calculate error at 1kHz
    // Setting the error calculation rate is equivalent to setting the sampling rate
    
    Serial.begin(9600);  // output
    Serial.println("Angle(n) \tAngle(deg) \tSpeed(RPM)");
}

// main loop, work is done by interrupt service routines, this one only prints stuff
void loop() {
    cmillis = millis();
    rotating = true;  // reset the debouncer

    Serial.print(sp); Serial.print("\t");
    Serial.print(encPos, DEC); Serial.print("\t");
    Serial.print(ang, 2); Serial.print("\t");
    Serial.println(spd, 2);

    if (lastReportedPos != encPos) {
        ang = encPos * 0.15;    // Max 9830.25 deg or 27.30625 rev
        spd = (encPos - lastReportedPos) * 1000 * 0.15 / (cmillis - pmillis) * 360 / 60;
        lastReportedPos = encPos;
        pmillis = cmillis;
    } else 
        spd = 0;

    // Serial, only process if there's data in the buffer
    if (Serial.available() > 0) {
        sp = Serial.readStringUntil('\n').toInt();
    }

    pidCalc();

    // Output
    if (dc < 0) {
      // CCW
      digitalWrite(dirA, LOW);
      digitalWrite(dirB, HIGH);
      dc *= -1;
    } else {
      // CW
      digitalWrite(dirA, HIGH);
      digitalWrite(dirB, LOW);
    }
    
    if (dc > 100) dc = 100;
    analogWrite(outPWM, dc*255/100);
}

// Interrupt at 1kHz
void pidCalc() {
    // PID
    e = sp - spd;
    dc = e*kp; // + e_sum*ki + (e - e_prev)*kd;
    e_prev = e;
    e_sum += e;
}

// Interrupt on A changing state
void doencA() {
    // debounce
    if ( rotating ) delay (1);  // wait a little until the bouncing is done
    // Test transition, did things really change?
    if ( digitalRead(encPinA) != A_set ) { // debounce once more
        A_set = !A_set;
        if ( A_set && !B_set ) encPos += 1; // adjust counter + if A leads B
        rotating = false;  // no more debouncing until loop() hits again
    }
}

// Interrupt on B changing state, same as A above
void doencB() {
    if ( rotating ) delay (1);
    if ( digitalRead(encPinB) != B_set ) {
        B_set = !B_set;
        if ( B_set && !A_set ) encPos -= 1; //  adjust counter - 1 if B leads A
        rotating = false;
    }
}
