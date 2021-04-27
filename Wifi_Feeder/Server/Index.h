// The R"=== allows for literal string,
// no need for newline, qote, or apostrophe.
static const char HtmlPage[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
<meta name=viewport content=width=device-width,initial-scale=1.5,minimum-scale=1.0>
<title>Feeder Control</title>
</head>
<body>

<h1>FEED</h1>
<!--<form method="post" action="/timeForm">-->
    <label>Time:</label>
    <input type="time" id="timeValue" name="name">
    <button onclick="sendTime()">Submit</button>
<!--</form-->
<br>

<form method="post" action="/feedForm">
    <label>Daily:</label>
    <input type="text" id="feedVal1" name="val1" size="1" value="2">
    <label>Each:</label>
    <input type="text" id="feedVal2" name="val2" size="1" value="100">
    <br>
    <label>Delay:</label>
    <input type="text" id="feedVal3" name="val3" size="1" value="5">
    <input type="submit" value="Submit">
</form>

<p>Current time: <span id="timeNow">00:00</span></p>
<p>Feeding time: <span id="timeFeed">00:00</span></p>

<h1>SERVO</h1>
<form method="post" action="/servForm">
    <input type="text" name="textValue" value="">
    <input type="submit" value="Submit">
</form>

<h1>LED</h1>
<p>Click to switch LED on and off.</p>
<button id="ledToggle" onclick="sendLed(0)">OFF</button>


<script>
function sendTime() {
    var val = document.getElementById("timeValue").value;
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        // Something after update in the webpage
    };
    xhttp.open("GET", "setTime?val="+val, true);
    xhttp.send();
}

function sendLed(led) {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
//            var resp = this.responseText;
            if (led==1) { // Currently ON
                document.getElementById("ledToggle").
                    innerHTML = "OFF";
                document.getElementById("ledToggle").
                    onclick = function() {
                        // String in HTML, function in JS
                        sendLed(0)
                    };
            } else if (led==0) { // Currently OFF
                document.getElementById("ledToggle").
                    innerHTML = "ON";
                document.getElementById("ledToggle").
                    onclick = function() {
                        sendLed(1)
                    };
            }
        }
    };
    xhttp.open("GET", "setLed?state="+led, true);
    xhttp.send();
}

setInterval(function() {
    // Call a function repetatively with 2 Second interval
    getTime();
}, 2000);

window.onload = function() {
    getTime();
    getFeed();
}

function getTime() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            document.getElementById("timeNow").innerHTML =
            this.responseText.split(",")[0];
            document.getElementById("timeFeed").innerHTML =
            this.responseText.split(",")[1];
        }
    };
    xhttp.open("GET", "readTime", true);
    xhttp.send();
}

function getFeed() {
    var xhttp = new XMLHttpRequest();
    xhttp.onreadystatechange = function() {
        if (this.readyState == 4 && this.status == 200) {
            document.getElementById("feedVal1").value =
            this.responseText.split(",")[0];
            document.getElementById("feedVal2").value =
            this.responseText.split(",")[1];
            document.getElementById("feedVal3").value =
            this.responseText.split(",")[2];
        }
    };
    xhttp.open("GET", "readFeed", true);
    xhttp.send();
}

</script>
</body>
</html>

)=====";
