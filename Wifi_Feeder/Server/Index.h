// The R"=== allows for literal string,
// no need for newline, qote, or apostrophe.
static const char INDEX_HTML[] PROGMEM = R"=====(
<!DOCTYPE html>
<html>
<head>
  <link rel="stylesheet" href="style.css">
  <!--link rel="stylesheet" href="https://bootswatch.com/4/darkly/bootstrap.min.css"-->
  <meta name=viewport content=width=device-width,initial-scale=1.5,minimum-scale=1.0>
<title>Feeder Control</title>
</head>

<style>
body {
    padding-top: 20px;
}
.mb-3 {
    width: 300px;
}
.rl {
    border-top-left-radius: 0.25rem !important;
    border-bottom-left-radius: 0.25rem !important;
}
.rr {
    border-top-right-radius: 0.25rem !important;
    border-bottom-right-radius: 0.25rem !important;
}

/* Overriding @media min-width */
@media (min-width: 200px) {
  .form-inline .form-control {
      display: inline-block;
      width: auto;
      vertical-align: middle;
  }
}
.form-control {
    text-align: right;
}

</style>

<body class="container">
<h1><a href="/">
    FEED 
</a></h1>
<h3>Main</h3>
<!--<form method="post" action="/timeForm">-->
<div class="input-group mb-3">
    <div class="input-group-prepend">
        <span class="input-group-text">Time</span>
    </div>
    <input size="1" type="time" class="form-control mr-2 rr" id="timeValue" name="name">
    <button class="btn btn-success" onclick="sendTime()">Submit</button>
</div>
<!--</form-->
<br>

<form method="post" action="/feedForm">
    <div class="input-group mb-3">
        <div class="input-group-prepend">
            <label class="input-group-text">Daily</label></div>
        <input class="form-control" type="text"
            id="feedVal1" name="val1" size="2" value="2">
        <div class="input-group-append">
            <label class="input-group-text rr mr-2">X</label> </div>
      
        <div class="input-group-prepend">
            <label class="input-group-text rl">Each</label> </div>
        <input class="form-control" type="text"
            id="feedVal2" name="val2" size="2" value="100">
        <div class="input-group-append">
            <label class="input-group-text">%</label> </div>
    </div>
    
    <div class="input-group mb-3">
        <div class="input-group-prepend">
            <label class="input-group-text">Delay</label></div>
        <input class="form-control" type="text"
            id="feedVal3" name="val3" size="2" value="5">
        <div class="input-group-append">
            <label class="input-group-text rr mr-2">s</label> </div>
      
        <input class="btn btn-success" type="submit" value="Submit">
    </div>
</form>

<h3>Time</h3>
<div class="input-group mb-3">
    <div class="input-group-prepend">
        <label class="input-group-text">Current</label> </div>
    <span class="form-control rr" type="text" 
        id="timeNow">00:00</span>
</div>

<div class="input-group mb-3">
    <div class="input-group-prepend">
        <label class="input-group-text">Feeding</label> </div>
    <span class="form-control rr" type="text" 
        id="timeFeed">00:00</span>
</div>

<h3>Servo</h3>
<form class="form-inline mb-3" method="post" action="/servForm">
    <input type="text" class="form-control mr-2" name="textValue" value="">
    <input type="submit" class="btn btn-success" value="Submit">
</form>

<h3>Light</h3>
<button class="btn btn-danger" id="ledToggle" onclick="sendLed(0)">OFF</button>


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
                    className = "btn btn-danger";
                document.getElementById("ledToggle").
                    onclick = function() {
                        // String in HTML, function in JS
                        sendLed(0)
                    };
            } else if (led==0) { // Currently OFF
                document.getElementById("ledToggle").
                    innerHTML = "ON";
                document.getElementById("ledToggle").
                    className = "btn btn-info";
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
