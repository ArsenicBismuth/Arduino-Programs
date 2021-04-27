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
String timeForm = "00:00";
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
    server.on("/readTime", handle_ajaxTime);
    server.on("/timeForm", handle_timeForm);
    server.on("/feedForm", handle_feedForm);
    server.on("/servForm", handle_servForm);
    server.on("/setLed",   handle_ajaxLed);
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
    if (timeNow >= feedTime && timeNow < feedTime+60*2) {
        feedNow = true;
        nextFeed(); // Update feedTime to the new one
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
    long val = (feed[0]+1) * (timeNow+1) / (24*3600);
    Serial.printf("%d %d\n", val, ((24*3600) / (timeNow+1)));
    feedn = val;
    nextFeed(); // So get the next one
}

void nextFeed() {
    // Divide 24 hours into feed[0]+2 intervals,
    // And feed in all interval except at 0 & 24.
    if (feedn < feed[0]) feedn += 1;
    else feedn = 1;
    setFeed(feedn);
}

void setFeed(int n) {
    feedTime = offTime + 3600*24 / (feed[0]+1) * n;
}


//// Server
void handle_feedForm() {
    feed[0] = server.arg("textValue1").toInt(); // Daily
    feed[1] = server.arg("textValue2").toInt(); // Each
    offTime = server.arg("textValue3").toInt(); // Delay

    findFeed();
    
    Serial.println("Text received: ");
    for (int i = 0; i < server.args(); i++)
        Serial.printf("Arg #%d: %s\n", i, &server.arg(i));
        
    server.send(200, "text/html", SendHTML());
}

void handle_timeForm() {
    timeForm = server.arg("dateValue");

    // Sync time (ending exclusive)
    int hr = timeForm.substring(0,2).toInt();
    int mn = timeForm.substring(3,5).toInt();
    setTime(0 + hr*3600 + mn*60); // setTime(hr, mn, 0,1,1,1970);
    timeNow = now();

    findFeed();

    Serial.printf("Text received: %s\n", &timeForm);
    server.send(200, "text/html", SendHTML());
}

void handle_servForm() {
    // Traditional, non-ajax method.
    servForm = server.arg("textValue").toInt();
    servo.write(servForm);
    
    Serial.printf("Text received: %d\n", servForm);
    server.send(200, "text/html", SendHTML());
}

void handle_ajaxTime() {
    // Send time values only to client ajax request.
    String msg = strDate(timeNow) +","+ strDate(feedTime);
    server.send(200, "text/plane", msg);
}

void handle_ajaxLed() {
    // From xhttp.open("GET", "setLed?state="+led, true);
    String t_state = server.arg("state");
    if (t_state == "1") ledStat = true; // On
    else ledStat = false;
    
    // Control LED
    digitalWrite(LEDpin, !ledStat);

    Serial.printf("Text received: %s\n", &t_state);
    server.send(200, "text/plane", "");
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
