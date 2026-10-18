# SD card components

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
- **Retention** (`cam_store::prune`): once a minute, delete a bounded batch (100) of files
  older than `Retention Days` (age from the FAT mtime) and drop empty day folders. It streams
  directory entries one at a time, so no listing is ever held in RAM.

Tuning: the `Change Score` sensor reports the current difference (0..255); watch it while
something moves and set `Change Threshold` just below that. Turn off `Change Gate` to store
every period.

