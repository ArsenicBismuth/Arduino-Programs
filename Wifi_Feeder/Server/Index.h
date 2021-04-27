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
<p>Current time: <span id="timeNow">00:00</span></p>
<p>Feeding time: <span id="timeFeed">00:00</span></p>

<h1>SERVO</h1>
<form method="post" action="/servForm">
    <input type="text" name="textValue" value="">
    <input type="submit" value="Submit">
</form>

<h1>LED</h1>
<p>Click to switch LED on and off.</p>
<button id="ledToggle" type="button" onclick="sendData(0)">OFF</button>


<script>
function sendData(led) {
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
                        sendData(0)
                    };
            } else if (led==0) { // Currently OFF
                document.getElementById("ledToggle").
                    innerHTML = "ON";
                document.getElementById("ledToggle").
                    onclick = function() {
                        sendData(1)
                    };
            }
        }
    };
    xhttp.open("GET", "setLed?state="+led, true);
    xhttp.send();
}

setInterval(function() {
    // Call a function repetatively with 2 Second interval
    getData();
}, 2000);

function getData() {
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
</script>
</body>
</html>

)=====";
