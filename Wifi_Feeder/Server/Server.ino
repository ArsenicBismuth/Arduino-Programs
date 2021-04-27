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

// The R"=== allows for literal string,
// no need for newline, qote, or apostrophe.
const char htmlPage[]=R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name=viewport content=width=device-width,initial-scale=1.5,minimum-scale=1.0>
<title>Feeder Control</title>
</head>
<body>

<h1>FEED</h1>
<form method="post" action="/timeForm">
    <label>Time:</label>
    <input type="time" name="dateValue">
    <input type="submit">
</form>
<br>
<form method="post" action="/feedForm">
    <label>Daily:</label>
    <input type="text" name="textValue1" size="1" value="2">
    <label>Each:</label>
    <input type="text" name="textValue2" size="1" value="100">
    <br>
    <label>Delay:</label>
    <input type="text" name="textValue3" size="1" value="5">
    <input type="submit" value="Submit">
</form>

<h1>SERVO</h1>
<form method="post" action="/servForm">
    <input type="text" name="textValue" value="">
    <input type="submit" value="Submit">
</form>

<h1>LED</h1>
<p>Click to switch LED on and off.</p>
<form method="get">
)=====";


void setup() {
    Serial.begin(9600);
    pinMode(LEDpin, OUTPUT);
    servo.attach(Serpin);
    setFeed(1);

    WiFi.softAP(ssid, password);
    WiFi.softAPConfig(local_ip, gateway, subnet);
    delay(100);
    
    server.on("/", handle_OnConnect);
    server.on("/timeForm", handle_timeForm);
    server.on("/feedForm", handle_feedForm);
    server.on("/servForm", handle_servForm);
    server.on("/ledOn", handle_ledon);
    server.on("/ledOff", handle_ledoff);
    server.onNotFound(handle_NotFound);

    server.begin();
    Serial.println("HTTP server started");
}

void loop() {
    //// Server
    server.handleClient();

    // Control LED
    if(ledStat) digitalWrite(LEDpin, HIGH);
    else digitalWrite(LEDpin, LOW);

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

    // Debug
//    Serial.println(strDate(timeNow) +" "+ strDate(feedTime));
}


// Process
void nextFeed() {
    // Divide 24 hours into feed[0]+2 intervals,
    // And feed in all interval except at 0 & 24.
    if (feedn < feed[0]) feedn += 1;
    else feedn = 1;
    setFeed(feedn);
}

void setFeed(int n) {
    feedTime = offTime + 3600*24 / (feed[0]+1) * n;
    
    // If feed time is lower than now, look for the next one.
    // Until it's the end of the day (next = tomorrow).
    if ((feedTime < timeNow) && (feed[0] > 1) && (n < feed[0]+1)) {
        nextFeed();
    }
}


// Server
void handle_feedForm() {
    feed[0] = server.arg("textValue1").toInt(); // Daily
    feed[1] = server.arg("textValue2").toInt(); // Each
    offTime = server.arg("textValue3").toInt(); // Delay

    setFeed(feedn);
    
    Serial.println("Text received: ");
    for (int i = 0; i < server.args(); i++)
        Serial.printf("Arg #%d: %s\n", i, &server.arg(i));
        
    server.send(200, "text/html", SendHTML(ledStat==LOW));
}

void handle_timeForm() {
    timeForm = server.arg("dateValue");

    // Sync time (ending exclusive)
    int hr = timeForm.substring(0,2).toInt();
    int mn = timeForm.substring(3,5).toInt();
    setTime(0 + hr*3600 + mn*60); // setTime(hr, mn, 0,1,1,1970);
    timeNow = now();

    setFeed(1);

    Serial.printf("Text received: %s\n", &timeForm);
    server.send(200, "text/html", SendHTML(ledStat==LOW));
}

void handle_servForm() {
    servForm = server.arg("textValue").toInt();
    servo.write(servForm);
    
    Serial.printf("Text received: %s\n", &servForm);
    server.send(200, "text/html", SendHTML(ledStat==LOW));
}

void handle_OnConnect() {
    server.send(200, "text/html", SendHTML(ledStat==LOW)); 
}

void handle_ledon() {
    ledStat = LOW;
    server.send(200, "text/html", SendHTML(true)); 
}

void handle_ledoff() {
    ledStat = HIGH;
    server.send(200, "text/html", SendHTML(false)); 
}

void handle_NotFound(){
    server.send(404, "text/plain", "Not found");
}

String SendHTML(uint8_t led){
    // Read static components from PROGMEM
    String ptr = htmlPage;
    
    // Changable button
    if(led) ptr +="<input type=\"button\" value=\"LED OFF\" onclick=\"window.location.href='/ledOff'\">\n";
    else ptr +="<input type=\"button\" value=\"LED ON\" onclick=\"window.location.href='/ledOn'\">\n";
    ptr +="</form>\n";
    ptr +="<p>Current time: "+ strDate(timeNow) +"</p>";
    ptr +="<p>Feeding time: "+ strDate(feedTime) +"</p>";
    
    ptr +="</body>\n";
    ptr +="</html>\n";
    return ptr;
}

// Utils
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
