#include "latest_snapshot.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>

#include "esphome/core/log.h"

namespace esphome {
namespace latest_snapshot {

static const char *TAG = "latest_snapshot";

// Chunk used to copy the file to the socket. Deliberately a small static buffer:
// the handler must never allocate a whole-file buffer (see the header / README).
static char STREAM_BUF[512];

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

  // Snapshot names are timestamped (snap_YYYYMMDD_HHMMSS.jpg), so the lexicographically
  // largest matching name is also the newest.
  std::string best;
  struct dirent *entry;
  while ((entry = readdir(dir)) != nullptr) {
    std::string name = entry->d_name;
    if (name.size() < this->extension_.size())
      continue;
    if (name.compare(name.size() - this->extension_.size(), this->extension_.size(), this->extension_) != 0)
      continue;
    if (name > best)
      best = name;
  }
  closedir(dir);

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

  size_t total = 0;
  size_t n;
  while ((n = fread(STREAM_BUF, 1, sizeof(STREAM_BUF), fp)) > 0) {
    if (httpd_resp_send_chunk(req, STREAM_BUF, n) != ESP_OK)
      break;
    total += n;
  }
  fclose(fp);
  httpd_resp_send_chunk(req, nullptr, 0);

  ESP_LOGD(TAG, "Served %s (%u bytes)", path.c_str(), (unsigned) total);
}

}  // namespace latest_snapshot
}  // namespace esphome
