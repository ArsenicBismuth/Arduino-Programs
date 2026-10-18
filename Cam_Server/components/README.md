# SD card components

> Build/flash from the `esphome` conda environment: `conda activate esphome`, then
> `esphome run cam.yaml`. (`python`/`pip` are not on PATH outside that env.)

`latest_snapshot/` is the local component described below; the rest of this directory is
the SD card driver and file server that `cam.yaml` includes.

## latest_snapshot (local)

Serves the newest snapshot straight off the card, no UI:

```
GET http://<host>/latest  ->  newest *.jpg in /sdcard, as image/jpeg
```

```yaml
latest_snapshot:
  id: latest_snapshot_srv
  url: /latest
  root: /sdcard
  extension: .jpg
```

`latest.jpg` (i.e. `latest` + `extension`) is preferred when present, because `cam.yaml`
overwrites it on every save, so it is the newest by construction. Only if it is missing
does the lookup fall back to the lexicographically largest matching name (correct for the
old timestamped `snap_YYYYMMDD_HHMMSS.jpg` files, whose names sort by time).

## Troubleshooting log

### 2026-10-17 — SD file downloads returned an empty response

**Symptom.** `GET /file/<name>.jpg` (and a first cut of `GET /latest`) returned no headers
and no body: browser `net::ERR_EMPTY_RESPONSE`, `curl` exit 52. Meanwhile the `/file`
directory index worked, and the MJPEG/snapshot servers on `:8080`/`:8081` delivered larger
images fine.

**Root cause.** Every failing path buffered the whole ~21 KB image in one heap allocation
(`std::vector<uint8_t>::resize()`, or a growing `std::string`). This is a **non-PSRAM**
ESP32-CAM whose camera already holds DRAM frame buffers, so there is no free *contiguous*
block that large. `operator new` fails, and ESPHome's `throw_stubs.cpp` turns a failed
allocation into `esp_system_abort("std::bad_alloc")` — an immediate reset. The connection
dies before a single byte is sent.

The paths that *worked* never allocated a frame-sized buffer: the index only builds a
small string, and `:8080`/`:8081` stream the camera's existing buffer. This also explains
why the device felt slow and flaky while it was happening.

**Fix.** Stream the file to the socket in 512-byte chunks (`httpd_resp_send_chunk`) from a
static buffer — see `latest_snapshot.cpp`. Peak extra memory is ~0.5 KB instead of
~21 KB.

`sd_file_server` had the same problem: `handle_download()` read the file with
`SdMmc::read_file()`, which resizes a vector to the whole file. It is now patched to
stream the same way, so `GET /file/<name>.jpg` works.

Update: the `/file` index no longer buffers the whole page in RAM. It is streamed in small
chunks and paged (100 rows per request, `?start=N`) — see the 2026-10-18 entry below.

### 2026-10-18 — `/file/<dir>` hung completely, root index was sluggish

**Symptom.** `GET /file` (card root) usually loaded, but `GET /file/snap` and
`GET /file/snap/<dir>/` hung every time and never returned a byte. It happened with a
small card (fewer than a hundred files), so it was not a "too many files" problem.

**Root cause.** The index served a *directory listing*, and built the whole page in RAM
first: the full `std::vector<FileInfo>` plus the complete HTML as a `std::string` in
`AsyncResponseStream`. Even ~90 rows is ~26 KB of HTML, and on this non-PSRAM board an
allocation that size fails (see the 2026-10-17 entry) — the failed allocation aborts the
device, so the request dies before a single byte is sent. The root listing stays small
(a couple of folders plus `latest.jpg`) and so usually survived; a day folder full of
snapshots did not.

**Fix.** `sd_file_server` now streams the page in small chunks
(`httpd_resp_send_chunk`, no full-page `std::string`) and renders at most 100 rows per
request, chosen with `?start=N`, stopping the directory walk as soon as the page is full.
The SD driver gained a callback-based listing walk so the listing vector is never built.
`/file` and `/file/snap/<dir>/` now serve a bounded page regardless of card contents.

**Related fixes in the same pass.**

- `sd_mmc_card::update_sensors()` called `f_getfree()` — a full-FAT scan — on every
  write/delete/mkdir even when no used/total/free sensor was configured, and outside the
  FatFs volume lock. It now runs only when a space sensor exists. Together with the
  per-write cost, this was why the device also felt slow and flaky.
- Unsupported HTTP methods (anything but GET/DELETE) fell through `handleRequest()`
  without a response, so the client hung until its own timeout. They now get a 405.
- Streamed downloads and `/latest` used 512-byte chunks with Nagle enabled; they now use
  2 KB chunks with `TCP_NODELAY` set on the session.
- `/latest` preferred a stale `snap_*.jpg` over `latest.jpg` (see above).

**Card housekeeping.** Older firmware wrote flat `snap_YYYYMMDD_HHMMSS.jpg` files into the
card root, which `cam_store::prune()` (scoped to `/snap`) never removes. They enlarge the
root directory and are what `/latest` used to pick up, so it is worth emptying the card
root by hand once — the current firmware keeps everything under `/snap/`.

### 2026-10-18 (b) — links from a directory opened with a trailing slash contained `//`

**Symptom.** Opening a file from a folder browsed as `/file/snap/20261018/` loaded
`/file/snap/20261018//001744.jpg` — a doubled slash.

**Root cause.** `list_directory_file_info_rec()` copied the listing path and then always
appended a separator before each entry name. When the path already ended in `/` (exactly what
a browser sends for a directory URL), every entry came out as `/snap/20261018//x.jpg`, and
the row link is built straight from it. FatFs tolerated the doubled slash, so the file still
opened, but the URL was wrong.

**Fix.** Only insert the separator when the path does not already end in one.

### 2026-10-18 (c) — the board went unresponsive shortly after browsing a few files

**Symptom.** After opening a couple of files the device stopped answering — the whole host
went quiet, including the :8080/:8081 camera servers — and recovered after tens of seconds.
The serial log sometimes showed `[E][json:111] JSON document overflow`.

**What is going on.** Two independent limits are in play:

- The esp-idf web server is built with `max_open_sockets = 7` and reserves three of those for
  itself, so only about **four client connections** can be open at once. Neither `web_server`
  nor the YAML exposes this, so it cannot be raised from the config. When the server is full
  and a new connection arrives, `lru_purge_enable` (set by ESPHome) makes httpd close the
  *least recently used* session — it drops a live request, which is exactly what "takes a
  long time or does not respond" looks like from the browser. The :8080/:8081 camera servers
  are tighter still: ESPHome configures them with a single socket.
- esp-idf's web server sends no `Connection` header, so a browser applies HTTP/1.1
  keep-alive and parks up to six connections per host — enough on its own to fill the server.
  Measured on the device: holding keep-alive connections open to `/file` while issuing
  another request pushed that request from ~0.1 s to ~8 s, and holding connections whose
  responses were *not* read wedged the device entirely. That last part is because there is
  only **one** httpd task and esp-idf sends with a 5 s timeout, so a client that stops
  reading blocks every other request on the port.

`JSON document overflow` is ArduinoJson 7 (7.4.2) failing to allocate its pool — a **heap
exhaustion** message, not a "payload too big" one, and the step just before the failed
allocation abort described in the 2026-10-17 entry.

**Fix.** The file server and `/latest` now answer with `Connection: close`, so the browser
drops its connection once the body is read instead of parking it, and the index page declares
an inline favicon so loading it no longer costs an extra request. Browsing files therefore
does not compete with the web UI (and its long-lived SSE connection) for the same few session
slots.

**If it still happens**, check, in order: whether the ESPHome web UI is open at the same time
(its SSE connection plus its parked asset connections use most of the four slots), whether
the camera stream on :8080 is open (that server has a single socket), and the serial log
around the event — a reboot banner (`rst:0x…`, `Guru Meditation`, `abort()`) points at heap
exhaustion rather than plain connection contention.

### 2026-10-18 (d) — the chunked index page looked like it hung on a day folder

**Symptom.** After the index was rewritten to stream chunks (2026-10-18 entry), `/file/snap`
(two folders) still opened instantly, `/file` felt flaky, and `/file/snap/<date>` appeared to
hang: the request stayed open for tens of seconds and often never rendered.

**Root cause.** `handle_index()` writes the page as one `httpd_resp_send_chunk()` call per
table row — dozens to ~100 tiny writes. It was the only streamed handler that did **not**
call `set_tcp_nodelay()`, so Nagle was still on. Nagle sends one small segment and then waits
for its ACK before sending the next, while the client deliberately delays that ACK
(delayed-ACK, up to ~200 ms). The page therefore advanced roughly one row per round-trip:
a 100-row day folder takes tens of seconds, the 2-row `/file/snap` stays instant, and any
other request queued behind the single httpd task looks dropped. Downloads and `/latest`
had already been given `TCP_NODELAY` in an earlier pass; the (newer) index was missed.

**Fix.** `handle_index()` now sets `TCP_NODELAY` on the session before writing, the same way
`handle_download()` and `/latest` do. `ChunkWriter` also stops after the first failed chunk
instead of retrying every remaining one: a retry can block for the full 5 s send timeout per
chunk, and with one httpd task that would wedge the whole web server for a closed client.

## Periodic snapshots + change gate (`cam_snapshot.h`)

`cam.yaml` saves snapshots on a schedule and filters near-duplicates, using the helpers in
`cam_snapshot.h` (pulled in via `esphome: includes:` next to `cam_ctl.h`).

- **Scheduled capture is free.** `esp32_camera`'s `on_image` fires for idle frames too
  (~every 10 s at the default `idle_framerate: 0.1`), so scheduling only needs a clock check
  in the trigger - no extra `request_image()` polling and no second capture.
- **Change gate** (`cam_gate::analyze`): decodes the frame at 1/8 scale with the ESP32 ROM
  TJpgDec and folds the result straight into a 16x12 luminance thumbnail, then returns the
  mean absolute difference against the last kept frame. Cost is a fixed ~4 KB decoder pool
  plus a couple of hundred bytes of grid state - **independent of resolution**, so it is safe
  on this non-PSRAM board (a full `esp_jpeg_decode` buffer would be 9-60 KB). On any decode
  failure it returns `-1` and the frame is kept (fail-open).
- **Layout:** `/snap/YYYYMMDD/HHMMSS.jpg`, plus `/latest.jpg` overwritten on every save so
  the `/latest` endpoint and `latest_snapshot` keep working unchanged. Day folders keep each
  FAT directory small - writing to one flat directory at a 10 s cadence would pass tens of
  thousands of entries in 7 days.
- **Retention** (`cam_store::prune`): once a day, drain the files older than `Retention Days`
  (age from the FAT mtime) in bounded 100-file batches (up to 2000 per pass) and drop empty
  day folders. It streams directory entries one at a time, so no listing is ever held in RAM,
  and the batch cap keeps the single daily pass from stalling the main loop for long. Runs
  regardless of the `Auto Snapshot` switch.

Tuning: the `Change Score` sensor reports the current difference (0..255); watch it while
something moves and set `Change Threshold` just below that. Turn off `Change Gate` to store
every period.

