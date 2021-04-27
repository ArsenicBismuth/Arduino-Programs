// Example 4 - Receive a number as text and convert it to an int
/*
The simplest case is where you want to type a number into the Serial Monitor (I am assuming you have line-ending set to newline). Let's assume you want to send the number 234. This is a variation on Example 2 and it will work with any integer value.  Note that if you don't enter a valid number it will show as 0 (zero).

src: http://forum.arduino.cc/index.php?topic=396450.0
*/

#include <Servo.h>

const byte numChars = 32;
char receivedChars[numChars];   // an array to store the received data
boolean newData = false;
int dataNumber = 0;             // new for this version

Servo servo;

void setup() {
    servo.attach(D6);
//    analogWriteFreq(50);
    
    Serial.begin(9600);
    Serial.println("<Arduino is ready>");
}

void loop() {
    recvWithEndMarker();
//    analogWrite(D6, showNewNumber());
    servo.write(showNewNumber());
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    if (Serial.available() > 0) {
        rc = Serial.read();

        if (rc != endMarker) {
            receivedChars[ndx] = rc;
            ndx++;
            if (ndx >= numChars) {
                ndx = numChars - 1;
            }
        }
        else {
            receivedChars[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
    
}

int showNewNumber() {
    dataNumber = 1023;             // new for this version
    if (newData == true) {
        dataNumber = atoi(receivedChars);   // new for this version
        Serial.print("This just in ... ");
        Serial.println(receivedChars);
        Serial.print("Data as Number ... ");    // new for this version
        Serial.println(dataNumber);     // new for this version
        newData = false;
    }
    return dataNumber;
}
