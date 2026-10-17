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

"Newest" is the lexicographically largest matching name, which is correct here because
files are timestamped (`snap_YYYYMMDD_HHMMSS.jpg`).

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

Known limitation: the `/file` index still builds its whole page in RAM first
(`beginResponseStream`), so it will hit the same ceiling once the card holds a few dozen
snapshots. Streaming the index would be a small follow-up patch.
