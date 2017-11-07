import processing.serial.*;

Serial  myPort;
short   portIndex = 0; // Index of serial port in list (varies by computer)
int     lf = 10;       // ASCII linefeed
String  inString;      // String for testing serial communication


// Input data. x, y coordinate 
float rel[] = {0, 0};  // Relative data, raw
float pos[] = {0, 0};  // Positional data

// Drawing data
float rect1[] = {0, 0};  // Starting location
float rect2[] = {0, 0};
float dim = 80.0;

void setup() {
  
  // Set up the main window
  size(900, 400);
  background(250);
  
  // Set up serial port access
  String portName = Serial.list()[portIndex];
  
  myPort = new Serial(this, portName, 57600);
  myPort.clear();
  myPort.bufferUntil(lf);
}

void draw() {
  // Left-half for raw data, right-half accumulated data
  rect1[0] = rel[0] * 0.01 + width * 1 / 4;
  rect1[1] = rel[1] * 0.01 + height / 2;
  rect2[0] = pos[0] * 0.01 + width * 3 / 4;
  rect2[1] = pos[1] * 0.01 + height / 2;
  
  fill(20);
  rect(rect1[0] - dim/2, rect1[1] - dim/2, dim, dim);
  
  fill(20);
  rect(rect2[0] - dim/2, rect2[1] - dim/2, dim, dim);
}


/*
 *  Read and process data from the serial port
 */
void serialEvent(Serial p) {
  inString = myPort.readString();
  
  try {
    // Parse the data
    // println(inString);
    String[] dataStrings = split(inString, ':');
    
    // Formatting x:int:y:int
    if (dataStrings.length == 4) {
      if (dataStrings[0].equals("x")) {
        rel[0] = int(dataStrings[1]);
        pos[0] += rel[0];
      } else if (dataStrings[3].equals("y")) {
        rel[1] = int(dataStrings[4]);
        pos[1] += rel[1];
      }
      // Clean buffer one-by-one until valid
    }
  } catch (Exception e) {
    println("Caught Exception");
  } 
}