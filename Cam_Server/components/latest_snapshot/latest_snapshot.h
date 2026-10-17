#pragma once

#include <string>

#include "esphome/core/component.h"
#include "esphome/components/web_server_base/web_server_base.h"

namespace esphome {
namespace latest_snapshot {

/// Serves the newest matching file from the SD card at a fixed URL, e.g.
/// `GET /latest` -> the newest `*.jpg` in `/sdcard`, as `image/jpeg`.
///
/// The file is streamed to the socket in small chunks, so the whole image is never
/// buffered in RAM. That matters on this non-PSRAM board: a ~21 KB allocation fails
/// and ESPHome turns a failed allocation into an abort/reboot (empty HTTP response).
/// See components/README.md -> "Troubleshooting log".
class LatestSnapshot : public Component, public AsyncWebHandler {
 public:
  explicit LatestSnapshot(web_server_base::WebServerBase *base) : base_(base) {}

  void setup() override;
  void dump_config() override;

  bool canHandle(AsyncWebServerRequest *request) const override;
  void handleRequest(AsyncWebServerRequest *request) override;
  bool isRequestHandlerTrivial() const override { return false; }

  void set_url(const std::string &url) { this->url_ = url; }
  void set_root(const std::string &root) { this->root_ = root; }
  void set_extension(const std::string &ext) { this->extension_ = ext; }

 protected:
  /// Full path of the newest file in root_ ending with extension_, or "" if none.
  std::string find_latest_() const;

  web_server_base::WebServerBase *base_;
  std::string url_;
  std::string root_;
  std::string extension_;
};

}  // namespace latest_snapshot
}  // namespace esphome
