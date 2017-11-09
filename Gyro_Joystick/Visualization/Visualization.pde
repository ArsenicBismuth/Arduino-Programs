import processing.serial.*;

Serial  myPort;
short   portIndex = 1; // Index of serial port in list (varies by computer)
int     lf = 10;       // ASCII linefeed
String  inString;      // String for testing serial communication


// Input data. x, y coordinate 
float rel[] = {0, 0};  // Relative data, raw
float pos[] = {0, 0};  // Positional data

// Drawing data
float x1, x2, y1, y2;
float rect1[] = {0, 0};  // Starting location
float rect2[] = {0, 0};
float dim = 80.0;

void setup() {
  
  // Set up the main window
  size(900, 400);
  background(250);
  noStroke();
  fill(20);
  
  // Set up serial port access
  String portName = Serial.list()[portIndex];
  println(portName);
  
  myPort = new Serial(this, portName, 57600);
  myPort.clear();
  myPort.bufferUntil(lf);
}

void draw() {
  // Avoid any calculation here
  
  background(250);
  rect(rect1[0], rect1[1], dim, dim);
  rect(rect2[0], rect2[1], dim, dim);
}


/*
 *  Read and process data from the serial port
 */
void serialEvent(Serial p) {
  inString = myPort.readString();
  print(inString);
  
  try {
    // Parse the data
    String[] dataStrings = split(inString, ':');
    
    // Formatting x:int\ny:int\n
    if (dataStrings.length == 2) {
      if (dataStrings[0].equals("x")) {
        rel[0] = float(dataStrings[1]);
        pos[0] += rel[0];
      } else if (dataStrings[0].equals("y")) {
        rel[1] = float(dataStrings[1]);
        pos[1] += rel[1];
      }
      // Clean buffer one-by-one until valid
      print(rel[0],rel[1],'\t');
    }
  } catch (Exception e) {
    println("Caught Exception");
  }
  
  // Left-half window for raw data, right-half accumulated data
  rect1[0] = rel[0] * 0.1 + width * 0.25 - dim/2;
  rect1[1] = rel[1] * 0.1 + height * 0.5 - dim/2;
  rect2[0] = pos[0] * 0.01 + width * 0.75 - dim/2;
  rect2[1] = pos[1] * 0.01 + height * 0.5 - dim/2;
}