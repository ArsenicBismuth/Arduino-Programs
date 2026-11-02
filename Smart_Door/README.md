# Smart Door ESPHome

Using ESPHome for low-code development of the Servo-controlled key rotator, and magnetic sensor for sensing.

## Setup

1. Rename or copy `secrets.example.yaml` to `secrets.yaml` and fill in the necessary information.
2. Install ESPHome CLI: https://esphome.io/guides/getting-started-command-line.html
3. Run `esphome run smart-door.yaml` to compile and upload the firmware to the ESP32.
4. Access http://smart-door.local:80 to control.

## Physical button (child safety)

The wired button on GPIO14 does **not** open the door on a single press. It must
be pressed twice in quick succession: each press can last at most
`press_max_length` (1s) and the gap between them at most
`double_press_window` (500ms), both tunable in `substitutions:` at the top of
`smart-door.yaml`. A single, long, or slow press is ignored.

## Text API and close announcements

This device does **not** talk to the Telegram API: no bot token and no chat id.
It exposes text over its REST API, and [`Telegram_Bridge`](../Telegram_Bridge)
relays it. The **Status** button formats the current state and publishes it to
the `Message` text sensor, and the `Open` action publishes an "OPENING" message:

```
POST http://smart-door.local/button/Status/press   # fill Message
POST http://smart-door.local/button/Open/press     # open door + fill Message
GET  http://smart-door.local/text_sensor/Message   # {"id":"message","value":"..."}
```

When a close sequence finishes (the servo has stopped), the door calls its
`send_status_message` script with `announce: true`. That publishes the text to
`Message` and POSTs it to the bridge's announcement API:

```
POST http://telegram-bridge.local/text/Announce/set?value=<url-encoded Message>
```

The **Status** button calls the same script with `announce: false`, so a manual
status check only updates the sensor and sends nothing. The bridge forwards an
announcement to the last Telegram user that interacted with it. Change
`telegram_bridge_url` in `substitutions:` if the bridge is not reachable as
`telegram-bridge.local`. This is the only outbound HTTP request the door makes.

The text is Telegram-Markdown (`*Status: ...*`), so the bridge sends it with
`parse_mode: Markdown`. Anything that can do an HTTP GET can read it — the
telemetry is not tied to Telegram.
