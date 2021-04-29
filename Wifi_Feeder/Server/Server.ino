/*  References:
 *   - https://forum.arduino.cc/t/html-text-box-to-send-text-to-esp8266-web-server/553649/7
 *   - https://www.w3schools.com/tags/tryit.asp?filename=tryhtml5_input_type_time
 *   - https://github.com/PaulStoffregen/Time
 *   - https://circuits4you.com/2018/02/04/esp8266-ajax-update-part-of-web-page-without-refreshing/
 *   - https://tttapa.github.io/ESP8266/Chap10%20-%20Simple%20Web%20Server.html
 *   - https://tttapa.github.io/ESP8266/Chap11%20-%20SPIFFS.html
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
#include <TimeLib.h>
#include <LittleFS.h>

/* Put your SSID & Password */
const char* ssid = "Feeder";    // Enter SSID here
const char* password = "12345678";    //Enter Password here

/* Put IP Address details */
IPAddress local_ip(192,168,1,1);
IPAddress gateway(192,168,1,1);
IPAddress subnet(255,255,255,0);
ESP8266WebServer server(80);

// Definitions
uint8_t LEDpin = LED_BUILTIN;
uint8_t Serpin = D6; 
int Serdur = 5;    // In sec, 100% duration.
int Serrot = 0;     // Servo rotation, 180 = inverse, 90 = stop.

// Form data
bool ledStat = LOW;
int servForm = 90;
int feed[] = {2, 100};  // Daily, each (percentage).
long offTime = 5;       // Offset feed time (sec).

// Process
bool feedNow = false;
bool servNow = false;
int feedn = 1;
time_t feedTime;
time_t servStart;

time_t timeNow;
Servo servo;


void setup() {
    Serial.begin(115200);
    pinMode(LEDpin, OUTPUT);
    servo.attach(Serpin);
    findFeed();
    LittleFS.begin();

    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    delay(100);
    
//    server.on("/", handle_onConnect);
//    server.on("/style.css", handle_css);
    server.on("/readTime", handle_getTime);
    server.on("/setTime",  handle_timeForm);
    server.on("/setLed",   handle_setLed);
    server.on("/readFeed", handle_getFeed);
    server.on("/feedForm", handle_feedForm);
    server.on("/servForm", handle_servForm);
    server.onNotFound(handle_notFound);
    // Not Found will handle all file-related request
    // It'll find and load for the first time,
    // and that file will be usable until restart.

    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    //// Server
    server.handleClient();

    // Update time (software)
    timeNow = now();
    // Using single day format.
    if (day()>1) adjustTime(-24*3600);


    //// Logic
    // Based on range, 2 minutes
    if (timeNow >= feedTime && timeNow < feedTime+2*60) {
        feedNow = true;
        nextFeed(); // Update feedTime to the new one
        // BUG: If feed once a day, servo will run 2minutes.
    }

    // Trigger servo rotation
    if (feedNow) {
        servo.write(Serrot);
        servStart = now();
        feedNow = false;
        servNow = true;
    }

    // Servo running for certain duration
    if (servNow && (now() > servStart + Serdur * feed[1]/100)) {
        servo.write(90); // Stop rotating
        // Servo detach
        servNow = false;
    }
}


//// Process
void findFeed() {
    // Find closest feedn, floored
    long val = (feed[0]+1) * (timeNow-offTime) / (24*3600);
    Serial.printf("%d\n", val);
    feedn = val;
    nextFeed(); // So get the next one
}

void nextFeed() {
    // Divide 24 hours into feed[0]+2 intervals,
    // And feed in all interval except at 0 & 24.
    if ((feedn < feed[0]) && (feedn > 0)) feedn += 1;
    else feedn = 1;
    setFeed(feedn);
}

void setFeed(int n) {
    feedTime = offTime + 24*3600 / (feed[0]+1) * n;
    // Just realized findFeed is this:
    // n = (time-offset) / (24*3600) * daily
}


//// Server
void handle_getFeed() {
    // Get variables on web load, async.
    char msg[20];
    sprintf(msg, "%d,%d,%d", feed[0], feed[1], offTime);

    Serial.println(msg);
    server.send(200, "text/plain", msg);
}

void handle_feedForm() {
    // Traditional, non-ajax method.
    feed[0] = server.arg("val1").toInt(); // Daily
    feed[1] = server.arg("val2").toInt(); // Each
    offTime = server.arg("val3").toInt(); // Delay

    findFeed();
    
    Serial.println("Received: ");
    for (int i = 0; i < server.args(); i++)
        Serial.printf("Arg #%d: %s\n", i, &server.arg(i));

    server.sendHeader("Location","/");  // Redirect to home
    server.send(303);   // 3xx necessary to give the location header
}

void handle_servForm() {
    // Traditional, non-ajax method.
    servForm = server.arg("textValue").toInt();
    servo.write(servForm);
    
    Serial.printf("Received: %d\n", servForm);
    server.sendHeader("Location","/");  // Redirect to home
    server.send(303);
}

void handle_timeForm() {
    // Set internal time, async.
    String timeForm = server.arg("val");

    // Sync time (ending exclusive)
    int hr = timeForm.substring(0,2).toInt();
    int mn = timeForm.substring(3,5).toInt();
    setTime(0 + hr*3600 + mn*60); // setTime(hr, mn, 0,1,1,1970);
    timeNow = now();

    findFeed();

    Serial.printf("Received: %s\n", &timeForm);
    server.send(200, "text/plain", "");
}

void handle_getTime() {
    // Send time values only to client ajax request.
    String msg = strDate(timeNow) +","+ strDate(feedTime);
    server.send(200, "text/plain", msg);
}

void handle_setLed() {
    // From xhttp.open("GET", "setLed?state="+led, true);
    String t_state = server.arg("state");
    if (t_state == "1") ledStat = true; // On
    else ledStat = false;
    
    // Control LED
    digitalWrite(LEDpin, !ledStat);

    Serial.printf("Received: %s\n", &t_state);
    server.send(200, "text/plain", "");
}

void handle_notFound(){
//    server.send(404, "text/plain", "Not found");
    if (!handle_fileRead(server.uri()))  // send it if it exists
        server.send(404, "text/plain", "404: Not Found");
}

//void handle_onConnect() {
//    server.send(200, "text/html", SendHTML()); 
//}
//
//void handle_css() {
//    server.send(200, "text/css", SendCSS()); 
//}

//String SendCSS() {
//    // Read static components from PROGMEM
//    String ptr = FPSTR(STYLE_CSS);
//    return ptr;
//}
//
//String SendHTML(){
//    // Read static components from PROGMEM
//    String ptr = FPSTR(INDEX_HTML);
//    return ptr;
//}

String getContentType(String filename){
    if(filename.endsWith(".html")) return "text/html";
    else if(filename.endsWith(".css")) return "text/css";
    else if(filename.endsWith(".js")) return "application/javascript";
    else if(filename.endsWith(".ico")) return "image/x-icon";
    else if(filename.endsWith(".gz")) return "application/x-gzip";
    return "text/plain";
}

bool handle_fileRead(String path){  // send the right file to the client (if it exists)
  Serial.println("handleFileRead: " + path);
  if(path.endsWith("/")) path += "index.html";           // If a folder is requested, send the index file
  String contentType = getContentType(path);             // Get the MIME type
  String pathWithGz = path + ".gz";
  
  if(LittleFS.exists(pathWithGz) || LittleFS.exists(path)){  // If the file exists, either as a compressed archive, or normal
    if(LittleFS.exists(pathWithGz))                          // If there's a compressed version available
      path += ".gz";                                         // Use the compressed version
    File file = LittleFS.open(path, "r");                    // Open the file
    size_t sent = server.streamFile(file, contentType);    // Send it to the client
    file.close();                                          // Close the file again
    
    Serial.println(String("\tSent file: ") + path);
    return true;
  }
  Serial.println(String("\tFile Not Found: ") + path);
  return false;                                          // If the file doesn't exist, return false
}


//// Utils
String strDate(time_t t) {
    return strTime(hour(t))+
           ":"+strTime(minute(t))+
           ":"+strTime(second(t))+ 
           " "+strTime(day(t))+ 
           "/"+strTime(month(t));
//           "/"+strTime(year(t));
}

String strTime(int i){
    // Leading zero
    char str[2];
    sprintf(str, "%02d", i);
    return str;
}
