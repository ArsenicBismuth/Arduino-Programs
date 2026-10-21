# Telegram Bridge

Small ESPHome program that does **one** thing: poll a Telegram bot and turn each
command into an HTTP request to another device.

It exists because `Smart_Door` used to poll Telegram itself, which competed with
the servo control, web server and status messages for the ESP32's CPU and
memory. The door no longer needs to speak to Telegram for commands; this device
does it and simply calls the door's REST API.

```
Telegram  ──getUpdates──▶  telegram-bridge (ESP32)  ──POST──▶  http://smart-door.local/button/Open/press
```

## Files

| File | Purpose |
| --- | --- |
| `telegram-bridge.yaml` | ESPHome configuration (this is the whole program) |
| `secrets.example.yaml` | Copy to `secrets.yaml` and fill in |
| `.gitignore` | Keeps `.esphome/` and `secrets.yaml` out of git |

## Setup

1. Copy `secrets.example.yaml` to `secrets.yaml` and fill in the WiFi, AP and
   Telegram values.
2. Install the ESPHome CLI: https://esphome.io/guides/getting-started-command-line.html
3. Flash it: `esphome run telegram-bridge.yaml`
4. Open http://telegram-bridge.local — the *Last Command* and *Last Result*
   sensors show what is happening, and *Reset Telegram Offset* re-syncs the
   bot cursor.

Getting the `telegram_chat_id`: message the bot once, then run
`python ../Smart_Door/macro/telegram.py --get-id` (uses the Smart_Door
`secrets.yaml`).

## The command map

All mappings live in `substitutions.command_map` at the top of
`telegram-bridge.yaml`:

```yaml
command_map: |-
  {"/open", "http://smart-door.local/button/Open/press"},
  {"/close", "http://smart-door.local/button/Close/press"},
  {"/zero", "http://smart-door.local/button/Zero/press"},
  {"/status", "http://smart-door.local/button/Status/press", true}
```

Each entry is `{"/command", "http://host/path"}`. Every command is forwarded as
an **HTTP POST with an empty body**, so any ESPHome REST endpoint works
(`/button/<name>/press`, `/switch/<name>/turn_on`, `/select/<name>/set?option=...`
needs a body, so use a script endpoint or a query string instead).

The optional third value `true` marks a **quiet** command: the bridge still calls
the URL and updates *Last Result*, but sends no `OK: ...` / `FAILED: ...` reply.
Use it when the target posts its own Telegram message — `/status` makes the door
send a formatted status message, so a second "OK" line would just be noise.

Adding an action is one line — no other changes needed:

```yaml
command_map: |-
  {"/open", "http://smart-door.local/button/Open/press"},
  {"/close", "http://smart-door.local/button/Close/press"},
  {"/zero", "http://smart-door.local/button/Zero/press"},
  {"/status", "http://smart-door.local/button/Status/press", true},
  {"/lights", "http://192.168.1.50/button/Desk_Lights/press"}
```

Notes:

- Command names are matched case-insensitively. `/open@MyBot` (group chat form)
  works too, and arguments are ignored (`/status now` → `/status`).
- `/help` (also `/commands`, `/start`) replies with the list of mapped commands
  and does not call anything.
- Entity names in the URL are **case sensitive** and use the ESPHome entity
  `name:`, e.g. `name: Open` → `/button/Open/press`.
- `.local` names are resolved over mDNS. If a target does not answer, use its
  IP address instead (and give it a static DHCP lease).

## Behaviour

- **One request at a time.** Polling, forwarded commands and replies are
  serialised through a single `http_busy` flag, so the bridge never opens
  overlapping TLS connections.
- **Single chat only.** Messages from any chat other than `telegram_chat_id` are
  ignored and logged. Do not remove this check — the bot would otherwise be an
  open doorbell.
- **No replay after reboot.** The last handled `update_id` is persisted, so a
  reboot continues where it left off.
- **No replay after flashing.** On the first poll after a fresh flash the
  current cursor is stored and the backlog is dropped.
- **Stale commands are dropped.** Anything older than `max_command_age`
  seconds (default 120) is ignored, so a `/open` sent before an outage cannot
  fire hours later. Set it to `0` to accept any age.
- **Optional result replies.** With `send_replies: true` you get
  `OK: /open -> HTTP 200` back in Telegram. Set it to `false` to halve the
  number of requests. Commands marked quiet (third map value `true`) never get
  this confirmation.

## Smart Door side

In `Smart_Door/smart-door.yaml` the door should stop polling Telegram itself,
while it can keep sending status messages:

```yaml
substitutions:
  use_telegram: true           # keep status messages
  use_telegram_polling: false  # commands are polled by telegram-bridge
```

Then add (or keep) mappings in this bridge:

| Telegram | HTTP request | Notes |
| --- | --- | --- |
| `/open` | `http://smart-door.local/button/Open/press` | replies `OK: /open -> HTTP 200` |
| `/close` | `http://smart-door.local/button/Close/press` | replies `OK: /close -> HTTP 200` |
| `/zero` | `http://smart-door.local/button/Zero/press` | replies `OK: /zero -> HTTP 200` |
| `/status` | `http://smart-door.local/button/Status/press` | quiet: the door sends the status message itself |

`/status` works because `send_status_message` in Smart_Door is exposed as a
normal button named **Status**, so the API endpoint is
`POST http://smart-door.local/button/Status/press`. Pressing it sends the
formatted `Status: Door is OPEN/CLOSED ...` message to `telegram_chat_id`.
This needs `use_telegram: true` on the door; with it `false`, the button only
logs a warning.

## Tuning

| Substitution | Default | Meaning |
| --- | --- | --- |
| `poll_interval` | `1s` | How often `getUpdates` is called. `2s`–`5s` is easier on the ESP32. |
| `max_command_age` | `120` | Seconds before a command is considered stale (`0` = never). |
| `send_replies` | `true` | Reply in Telegram with the HTTP status. |
| `request_timeout` | `8s` | Per-request timeout. |

Logs are available with `esphome logs telegram-bridge.yaml`.
