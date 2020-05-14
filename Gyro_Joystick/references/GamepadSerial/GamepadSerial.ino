#include "Joystick.h"

// Create Joystick
Joystick_ Joystick;

// Set to true for Auto Send mode.
// Call Joystick.sendState() to send in manual mode.
const bool testAutoSendMode = true;
bool buts[32];  // Storing buttons states

// Serial related parameters
const byte numChars = 32;
char received[numChars];   // an array to store the received data
boolean newData = false;

void setup() {
    // Zeroing array (works for char, int, long, and maybe float)
    memset(buts,0,sizeof(buts));
    
    // Set Range Values
    Joystick.setXAxisRange(-127, 127);
    Joystick.setYAxisRange(-127, 127);
    Joystick.setZAxisRange(0, 360);
    Joystick.setRxAxisRange(-127, 127);
    Joystick.setRyAxisRange(-127, 127);
    Joystick.setRzAxisRange(0, 360);
    Joystick.setThrottleRange(0, 255);
    Joystick.setRudderRange(255, 0);
    
    if (testAutoSendMode) Joystick.begin();
    else Joystick.begin(false);
}

void loop() {
    recvWithEndMarker();
    showNewData();

    // Skip if there's no valid input
    if (newData == false) {
        return;
    }

    // Conver the number to int
    char *num = received + 2;  // Skip 2 chars
    int i = atoi(num);

    // Check code, format: ax123, 2nd char ignored if unneded
    switch (received[0]) {
        case 'b':
            buts[i] = !buts[i];
            Joystick.setButton(i, buts[i]);
            break;
        case 'a':
            switch (received[1]) {
                case 'x': Joystick.setXAxis(i); break;
                case 'y': Joystick.setYAxis(i); break;
                case 'z': Joystick.setZAxis(i); break;
            }
            break;
        case 'r':
            switch (received[1]) {
                case 'x': Joystick.setRxAxis(i); break;
                case 'y': Joystick.setRyAxis(i); break;
                case 'z': Joystick.setRzAxis(i); break;
            } 
            break;
        case 't': Joystick.setThrottle(i); break;
        case 'u': Joystick.setRudder(i); break;
        default: Serial.println("Invalid."); break;
    }

    newData = false;
}

void recvWithEndMarker() {
    static byte ndx = 0;
    char endMarker = '\n';
    char rc;
    
    while (Serial.available() > 0 && newData == false) {
        rc = Serial.read();

        if (rc != endMarker) {
            received[ndx] = rc;
            ndx++;
            if (ndx >= numChars) ndx = numChars - 1;
        } else {
            received[ndx] = '\0'; // terminate the string
            ndx = 0;
            newData = true;
        }
    }
}

void showNewData() {
    if (newData == true) {
        Serial.print("Received: ");
        Serial.println(received);
    }
}
