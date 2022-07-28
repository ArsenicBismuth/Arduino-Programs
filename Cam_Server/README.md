[Ez] Reference:
- Wiring with UNO: https://willmakes.tv/2021/programming-esp32cam-arduino/
- Change SSID & stuff in `myconfig.h`.
- OTA password = `esp32admin`.
- No support for 5GHz network.
- Brownout error: Need external 5V, read below (it's as simple as **abc**).

What to do:

 0. Upload as usual (no external power). Flash mode: ESP32 IO-0 grounded. Remove it on run.
 1. Unplug Nano 5V -> ESP 5V.
 2. External 5V & GND -> ESP 5V & GND.
 3. External 5V -> Nano 5V.
 4. Will be a bit finnicky to let the serial start.
 5. Disconnect ext power (now it's like old with Nano 5V -> ESP 5V).
 6. Start as usual until error appear.
 7. Plug external power.
 8. In case you can't get serial, use the static IP address to connect.

Uploading with OTA works, make sure you follow this:
- https://github.com/easytarget/esp32-cam-webserver#programming
- The upload setting in Arduino IDE, not the "ESP32-CAM".
- Upload as usual first, to make sure OTA feature is added in firmware.