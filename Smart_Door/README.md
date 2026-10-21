# Smart Door ESPHome

Using ESPHome for low-code development of the Servo-controlled key rotator, and magnetic sensor for sensing.

## Setup

1. Rename or copy `secrets.example.yaml` to `secrets.yaml` and fill in the necessary information.
2. Install ESPHome CLI: https://esphome.io/guides/getting-started-command-line.html
3. Run `esphome run smart-door.yaml` to compile and upload the firmware to the ESP32.
4. Access http://smart-door.local:80 to control.

## Telegram

- `use_telegram` — send door status messages to Telegram.
- `use_telegram_polling` — read `/open` and `/status` commands from Telegram on
  this device.

Polling Telegram on the door competes with the servo loop, so keep
`use_telegram_polling: false` and let [`Telegram_Bridge`](../Telegram_Bridge)
poll instead:

```yaml
substitutions:
  use_telegram: true           # keep status messages
  use_telegram_polling: false  # commands are polled by Telegram_Bridge
```

The **Status** button sends the current door state to Telegram and is also
available over the REST API:

```
POST http://smart-door.local/button/Status/press
```

`Telegram_Bridge` calls exactly this endpoint for `/status`. It requires
`use_telegram: true`; otherwise the button just logs a warning.
