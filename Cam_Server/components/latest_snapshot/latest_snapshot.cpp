#include "latest_snapshot.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <esp_heap_caps.h>
#include <esp_system.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>

#include "esphome/core/hal.h"
#include "esphome/core/log.h"

namespace esphome {
namespace latest_snapshot {

static const char *TAG = "latest_snapshot";

// 1 KB, not 2: with ~4 KB free heap the largest block is often only ~1.5 KB, so a 2 KB
// pbuf allocation fails and httpd_send_chunk() blocks until the socket timeout.
static char STREAM_BUF[1024];

// Files are streamed in small chunks; with Nagle enabled the client can wait an extra
// round-trip between them. Turn it off for this response.
static void set_tcp_nodelay(httpd_req_t *req) {
  int fd = httpd_req_to_sockfd(req);
  int one = 1;
  if (fd >= 0 && setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) < 0) {
    ESP_LOGW(TAG, "setsockopt(TCP_NODELAY) failed: %s", strerror(errno));
  }
}

// httpd keeps a session (and its socket buffers, ~10 KB of heap) after the response unless
// it is told to close it; seven of those leave the heap too small to send the next chunk.
static void close_session(httpd_req_t *req) {
  int fd = httpd_req_to_sockfd(req);
  if (fd >= 0 && req->handle != nullptr)
    httpd_sess_trigger_close(req->handle, fd);
}

void LatestSnapshot::setup() { this->base_->add_handler(this); }

void LatestSnapshot::dump_config() {
  ESP_LOGCONFIG(TAG, "Latest Snapshot Server:");
  ESP_LOGCONFIG(TAG, "  Url: %s", this->url_.c_str());
  ESP_LOGCONFIG(TAG, "  Root: %s", this->root_.c_str());
  ESP_LOGCONFIG(TAG, "  Extension: %s", this->extension_.c_str());
}

bool LatestSnapshot::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET)
    return false;
  char buffer[AsyncWebServerRequest::URL_BUF_SIZE];
  std::string url(request->url_to(buffer));
  std::string with_slash = this->url_ + "/";
  return url == this->url_ || url == with_slash;
}

std::string LatestSnapshot::find_latest_() const {
  DIR *dir = opendir(this->root_.c_str());
  if (dir == nullptr) {
    ESP_LOGE(TAG, "opendir(%s) failed: %s", this->root_.c_str(), strerror(errno));
    return "";
  }

  // cam.yaml overwrites <root>/latest.jpg on every snapshot, so that file is the newest
  // by construction. Prefer it explicitly: a lexicographic scan would otherwise pick a
  // leftover snap_YYYYMMDD_HHMMSS.jpg from the old flat layout ('s' > 'l').
  const std::string latest_name = "latest" + this->extension_;
  bool has_latest = false;

  // Fallback for cards without latest.jpg: names are timestamped
  // (snap_YYYYMMDD_HHMMSS.jpg), so the lexicographically largest one is the newest.
  std::string best;
  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    std::string name = entry->d_name;
    if (name.size() < this->extension_.size())
      continue;
    if (name.compare(name.size() - this->extension_.size(), this->extension_.size(), this->extension_) != 0)
      continue;
    if (name == latest_name) {
      has_latest = true;
      continue;
    }
    if (name > best)
      best = name;
  }
  closedir(dir);

  if (has_latest)
    return this->root_ + "/" + latest_name;
  if (best.empty()) {
    ESP_LOGW(TAG, "No *%s file found in %s", this->extension_.c_str(), this->root_.c_str());
    return "";
  }
  return this->root_ + "/" + best;
}

void LatestSnapshot::handleRequest(AsyncWebServerRequest *request) {
  std::string path = this->find_latest_();
  if (path.empty()) {
    request->send(404, "text/plain", "no snapshot found");
    return;
  }

  FILE *fp = fopen(path.c_str(), "rb");
  if (fp == nullptr) {
    ESP_LOGE(TAG, "fopen(%s) failed: %s", path.c_str(), strerror(errno));
    request->send(500, "text/plain", "failed to open file");
    return;
  }

  // Stream straight from the file to the socket. No whole-file buffer is allocated, so
  // an image of any size can be served without risking the heap (and an abort).
  httpd_req_t *req = *request;
  httpd_resp_set_type(req, "image/jpeg");
  httpd_resp_set_hdr(req, "Content-Disposition", "inline; filename=latest.jpg");
  httpd_resp_set_hdr(req, "Accept-Ranges", "none");
  // Close after the response: the web server only has a few client slots, and a browser that
  // keeps the connection alive would hold one of them (see sd_file_server.cpp).
  httpd_resp_set_hdr(req, "Connection", "close");
  set_tcp_nodelay(req);

  size_t total = 0;
  size_t n;
  uint32_t t = millis();
  while ((n = fread(STREAM_BUF, 1, sizeof(STREAM_BUF), fp)) > 0) {
    uint32_t rms = millis() - t;
    if (rms > 250)
      ESP_LOGW(TAG, "fread %u B took %u ms (at %u)", (unsigned) n, (unsigned) rms, (unsigned) total);
    t = millis();
    esp_err_t err = httpd_resp_send_chunk(req, STREAM_BUF, n);
    uint32_t sms = millis() - t;
    if (sms > 250)
      ESP_LOGW(TAG, "send %u B took %u ms (at %u) heap %u min %u largest %u", (unsigned) n, (unsigned) sms,
               (unsigned) total, (unsigned) esp_get_free_heap_size(), (unsigned) esp_get_minimum_free_heap_size(),
               (unsigned) heap_caps_get_largest_free_block(MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT));
    t = millis();
    if (err != ESP_OK)
      break;
    total += n;
  }
  fclose(fp);
  if (httpd_resp_send_chunk(req, nullptr, 0) != ESP_OK)
    ESP_LOGW(TAG, "terminator failed, heap %u", (unsigned) esp_get_free_heap_size());
  close_session(req);

  ESP_LOGD(TAG, "Served %s (%u bytes)", path.c_str(), (unsigned) total);
}

}  // namespace latest_snapshot
}  // namespace esphome
