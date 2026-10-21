# Smart Door ESPHome

Using ESPHome for low-code development of the Servo-controlled key rotator, and magnetic sensor for sensing.

## Setup

1. Rename or copy `secrets.example.yaml` to `secrets.yaml` and fill in the necessary information.
2. Install ESPHome CLI: https://esphome.io/guides/getting-started-command-line.html
3. Run `esphome run smart-door.yaml` to compile and upload the firmware to the ESP32.
4. Access http://smart-door.local:80 to control.

## Text API (used by Telegram_Bridge)

This device does **not** talk to Telegram: no bot token, no chat id, no
`http_request` component. It only exposes text, and
[`Telegram_Bridge`](../Telegram_Bridge) relays it.

The **Status** button formats the current state and publishes it to the
`Message` text sensor, and the `Open` action publishes an "OPENING" message:

```
POST http://smart-door.local/button/Status/press   # fill Message
POST http://smart-door.local/button/Open/press     # open door + fill Message
GET  http://smart-door.local/text_sensor/Message   # {"id":"message","value":"..."}
```

The text is Telegram-Markdown (`*Status: ...*`), so the bridge sends it with
`parse_mode: Markdown`. Anything that can do an HTTP GET can read it — the
telemetry is not tied to Telegram.
