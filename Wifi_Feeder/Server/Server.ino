/*  References:
 *   - https://forum.arduino.cc/t/html-text-box-to-send-text-to-esp8266-web-server/553649/7
 *   - https://www.w3schools.com/tags/tryit.asp?filename=tryhtml5_input_type_time
 *   - https://github.com/PaulStoffregen/Time
 *   - https://circuits4you.com/2018/02/04/esp8266-ajax-update-part-of-web-page-without-refreshing/
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <Servo.h>
#include <TimeLib.h>
#include "Index.h"

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

    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    delay(100);
    
    server.on("/", handle_onConnect);
    server.on("/readTime", handle_getTime);
    server.on("/setTime",  handle_timeForm);
    server.on("/setLed",   handle_setLed);
    server.on("/readFeed", handle_getFeed);
    server.on("/feedForm", handle_feedForm);
    server.on("/servForm", handle_servForm);
    server.onNotFound(handle_notFound);

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

    //// Debug
//    Serial.println(strDate(timeNow) +" "+ strDate(feedTime));
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
    server.send(404, "text/plain", "Not found");
}

void handle_onConnect() {
    server.send(200, "text/html", SendHTML()); 
}

String SendHTML(){
    // Read static components from PROGMEM
    String ptr = FPSTR(HtmlPage);
    return ptr;
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
