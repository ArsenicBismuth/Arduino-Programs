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

<script>
function sendData(led) {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("LEDState").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "setLED?LEDstate="+led, true);
  xhttp.send();
}

setInterval(function() {
  // Call a function repetatively with 2 Second interval
  getData();
}, 2000); //2000mSeconds update rate

function getData() {
  var xhttp = new XMLHttpRequest();
  xhttp.onreadystatechange = function() {
    if (this.readyState == 4 && this.status == 200) {
      document.getElementById("ADCValue").innerHTML =
      this.responseText;
    }
  };
  xhttp.open("GET", "readADC", true);
  xhttp.send();
}
</script>

)=====";
