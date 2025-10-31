# Smart Door ESPHome

Using ESPHome for low-code development of the Servo-controlled key rotator, and magnetic sensor for sensing.

## Setup

1. Rename or copy `secrets.example.yaml` to `secrets.yaml` and fill in the necessary information.
2. Install ESPHome CLI: https://esphome.io/guides/getting-started-command-line.html
3. Run `esphome run smart-door.yaml` to compile and upload the firmware to the ESP32.
4. Access http://smart-door.local:80 to control.