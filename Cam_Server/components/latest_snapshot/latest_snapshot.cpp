#include "latest_snapshot.h"

#include <cerrno>
#include <cstdio>
#include <cstring>
#include <dirent.h>
#include <sys/stat.h>

#include "esphome/core/log.h"

namespace esphome {
namespace latest_snapshot {

static const char *TAG = "latest_snapshot";

void LatestSnapshot::setup() {
  ESP_LOGCONFIG(TAG, "Latest Snapshot endpoint '%s' -> newest '%s' in %s", this->url_.c_str(),
                this->extension_.c_str(), this->root_.c_str());
  this->base_->add_handler(this);
}

void LatestSnapshot::dump_config() {
  ESP_LOGCONFIG(TAG, "Latest Snapshot Server:");
  ESP_LOGCONFIG(TAG, "  Url: %s", this->url_.c_str());
  ESP_LOGCONFIG(TAG, "  Root: %s", this->root_.c_str());
  ESP_LOGCONFIG(TAG, "  Extension: %s", this->extension_.c_str());
}

bool LatestSnapshot::canHandle(AsyncWebServerRequest *request) const {
  if (request->method() != HTTP_GET)
    return false;
  // URL is decoded by url_to(); compare against the configured url (and its trailing-slash form).
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
  ESP_LOGI(TAG, "Request for %s", request->url().c_str());

  std::string path = this->find_latest_();
  if (path.empty()) {
    request->send(404, "text/plain", "no snapshot found");
    return;
  }

  // Report the size via stat() but do NOT use it to size a buffer - we stream.
  struct stat info;
  if (stat(path.c_str(), &info) != 0) {
    ESP_LOGE(TAG, "stat(%s) failed: %s", path.c_str(), strerror(errno));
  } else {
    ESP_LOGI(TAG, "Serving %s (%ld bytes)", path.c_str(), (long) info.st_size);
  }

  FILE *fp = fopen(path.c_str(), "rb");
  if (fp == nullptr) {
    ESP_LOGE(TAG, "fopen(%s) failed: %s", path.c_str(), strerror(errno));
    request->send(500, "text/plain", "failed to open file");
    return;
  }

  // Accumulate into an AsyncResponseStream - the exact code path the working /file
  // index page uses (beginResponseStream + send), just with an image/jpeg content type.
  AsyncResponseStream *response = request->beginResponseStream("image/jpeg");
  char chunk[1024];
  size_t total = 0;
  size_t read_len;
  while ((read_len = fread(chunk, 1, sizeof(chunk), fp)) > 0) {
    response->print(std::string(chunk, read_len));
    total += read_len;
  }
  bool read_error = ferror(fp);
  fclose(fp);

  ESP_LOGI(TAG, "Read %u bytes (error=%d); sending response", (unsigned) total, (int) read_error);
  request->send(response);
  ESP_LOGI(TAG, "Response sent for %s", path.c_str());
}

}  // namespace latest_snapshot
}  // namespace esphome
