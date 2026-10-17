# Vendored external components

These components are a local snapshot of
[`n-serrette/esphome_sd_card`](https://github.com/n-serrette/esphome_sd_card)
(`main` branch), vendored so we can carry one local patch. `cam.yaml` loads them via:

```yaml
external_components:
  - source:
      type: local
      path: components
    components: [sd_mmc_card, sd_file_server]
```

## Local patch

`sd_file_server/sd_file_server.cpp` → `SDFileServer::handle_download()`

Upstream passed a **temporary** MIME string straight into the response:

```cpp
auto *response = request->beginResponse(200, Path::mime_type(path).c_str(), file.data(), file.size());
```

Under the esp-idf framework `httpd_resp_set_type()` stores that pointer **without
copying** it, and `httpd_resp_send()` dereferences it later when it builds the response
header. By then `Path::mime_type(path)` (a short string) has been destroyed, so the
header is assembled from freed stack memory; `httpd_resp_send()` then bails out with
`ESP_ERR_HTTPD_RESP_HDR` **before sending a single byte**, and the browser reports an
empty response (`net::ERR_EMPTY_RESPONSE`). The folder index still worked because it
uses a string literal (`"text/html"`), which has static lifetime.

Fix (keep the string alive until after `send()` returns):

```cpp
std::string mime = Path::mime_type(path);
auto *response = request->beginResponse(200, mime.c_str(), file.data(), file.size());
```

## Re-vendoring an upstream update

Copy `components/sd_mmc_card` and `components/sd_file_server` from a newer upstream
ref, then re-apply the patch above to `sd_file_server.cpp`.
