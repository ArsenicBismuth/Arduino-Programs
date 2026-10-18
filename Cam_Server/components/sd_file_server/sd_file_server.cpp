#include "sd_file_server.h"
#include <cerrno>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <map>
#include <string>
#include <vector>
#ifdef USE_ESP_IDF
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/socket.h>
#endif
#include "esphome/core/log.h"
#include "esphome/components/network/util.h"
#include "esphome/core/helpers.h"

namespace esphome {
namespace sd_file_server {

static const char *TAG = "sd_file_server";

#ifdef USE_ESP_IDF
// Files are streamed in small chunks; with Nagle enabled the client can wait an extra
// round-trip between them. Turn it off for this response.
static void set_tcp_nodelay(httpd_req_t *req) {
  int fd = httpd_req_to_sockfd(req);
  int one = 1;
  if (fd >= 0 && setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one)) < 0) {
    ESP_LOGW(TAG, "setsockopt(TCP_NODELAY) failed: %s", strerror(errno));
  }
}

// The esp-idf web server is configured for 7 sockets, 3 of which it reserves for itself, so
// only ~4 client connections exist at a time. It sends no "Connection" header and esp-idf does
// not add one, so a browser defaults to keep-alive and parks up to six connections per host -
// which alone can fill the server. Declaring "Connection: close" makes the client drop the
// connection once the body is read, so browsing files never holds a slot a later request needs
// (a full server makes httpd evict the least recently used session, i.e. drop a live request).
static void set_connection_close(httpd_req_t *req) { httpd_resp_set_hdr(req, "Connection", "close"); }
#endif

// Sends the index page in small pieces instead of accumulating the whole page in a
// std::string first. A day folder can hold thousands of snapshots; buffering the full
// page exhausts the heap on this non-PSRAM board and aborts the device (see
// components/README.md). httpd copies each chunk straight to the socket.
class ChunkWriter {
 public:
#ifdef USE_ESP_IDF
  explicit ChunkWriter(httpd_req_t *req) : req_(req) {}
  bool write(const char *data, size_t len) { return httpd_resp_send_chunk(this->req_, data, len) == ESP_OK; }
  bool finish() { return httpd_resp_send_chunk(this->req_, nullptr, 0) == ESP_OK; }

 protected:
  httpd_req_t *req_;
#else
  explicit ChunkWriter(AsyncResponseStream *response) : response_(response) {}
  bool write(const char *data, size_t len) {
    this->response_->print(std::string(data, len).c_str());
    return true;
  }
  bool finish() { return true; }

 protected:
  AsyncResponseStream *response_;
#endif
 public:
  // Convenience overload so string literals can be written without a manual length.
  template<size_t N> bool write(const char (&data)[N]) { return this->write(data, N - 1); }
};

static const char INDEX_HEAD[] = R"(
  <!DOCTYPE html>
  <html lang="en">
  <head>
    <meta charset=UTF-8>
    <meta name=viewport content="width=device-width, initial-scale=1,user-scalable=no">
    <meta name="color-scheme" content="dark">
    <title>SD Card Files</title>
    <link rel="icon" href="data:,">
    <style>
    body {
      font-family: 'Segoe UI', system-ui, sans-serif;
      margin: 0;
      padding: 2rem;
      background: #121212;
      color: #e6e6e6;
      color-scheme: dark;
    }
    h1 {
      color: #6cb0ff;
      margin-bottom: 1.5rem;
      display: flex;
      align-items: center;
      gap: 1rem;
    }
    .container {
      max-width: 1200px;
      margin: 0 auto;
      background: #1e1e1e;
      border: 1px solid #2e2e2e;
      border-radius: 12px;
      box-shadow: 0 4px 12px rgba(0, 0, 0, 0.5);
      padding: 2rem;
    }
    table {
      width: 100%;
      border-collapse: collapse;
      margin-top: 1.5rem;
    }
    th, td {
      padding: 12px;
      text-align: left;
      border-bottom: 1px solid #2e2e2e;
    }
    th {
      background: #262626;
      font-weight: 500;
    }
    .file-actions {
      display: flex;
      gap: 8px;
    }
    button, .action-link {
      padding: 6px 12px;
      border: none;
      border-radius: 6px;
      background: #1f6feb;
      color: white;
      text-decoration: none;
      cursor: pointer;
      transition: background 0.2s;
    }
    button:hover, .action-link:hover {
      background: #388bfd;
    }
    .upload-form {
      margin-bottom: 2rem;
      padding: 1rem;
      background: #262626;
      border-radius: 8px;
    }
    .upload-form input[type="file"] {
      margin-right: 1rem;
    }
    .breadcrumb {
      margin-bottom: 1.5rem;
      font-size: 0.9rem;
      color: #9e9e9e;
    }
    .breadcrumb a {
      color: #6cb0ff;
      text-decoration: none;
    }
    .breadcrumb a:hover {
      text-decoration: underline;
    }
    .breadcrumb a:not(:last-child)::after {
      display: inline-block;
      margin: 0 .25rem;
      content: ">";
    }
    .folder {
      color: #6cb0ff;
      font-weight: 500;
    }
    .file-type {
      color: #9e9e9e;
      font-size: 0.9rem;
    }
    .folder-icon {
      width: 20px;
      height: 20px;
      margin-right: 8px;
      vertical-align: middle;
    }
    .header-actions {
      display: flex;
      align-items: center;
      gap: 1rem;
    }
    .header-actions button {
      background: #2ea043;
    }
    .header-actions button:hover {
      background: #3fb950;
    }
    .pager {
      display: flex;
      justify-content: space-between;
      gap: 1rem;
      margin-top: 1.5rem;
    }
  </style>
  </head>
  <body>
  <div class="container">
    <div class="header-actions">
      <h1>SD Card Files</h1>
      <button onclick="window.location.href='/'">Go to web server</button>
    </div>
    <div class="breadcrumb">
      <a href="/">Home</a>)";

static const char INDEX_TABLE_HEAD[] =
    "<table><thead><tr>"
    "<th>Name</th>"
    "<th>Type</th>"
    "<th>Size</th>"
    "<th>Actions</th>"
    "</tr></thead><tbody>";

static const char INDEX_FOOT[] =
    "</tbody></table>"
    "<script>"
    "function delete_file(path) {fetch(path, {method: \"DELETE\"});}"
    "</script>";

static const char UPLOAD_FORM[] =
    "<div class=\"upload-form\"><form method=\"POST\" enctype=\"multipart/form-data\">"
    "<input type=\"file\" name=\"file\"><input type=\"submit\" value=\"upload\"></form></div>";

SDFileServer::SDFileServer(web_server_base::WebServerBase *base) : base_(base) {}

void SDFileServer::setup() { this->base_->add_handler(this); }

void SDFileServer::dump_config() {
  ESP_LOGCONFIG(TAG, "SD File Server:");
  ESP_LOGCONFIG(TAG, "  Address: %s:%u", network::get_use_address(), this->base_->get_port());
  ESP_LOGCONFIG(TAG, "  Url Prefix: %s", this->url_prefix_.c_str());
  ESP_LOGCONFIG(TAG, "  Root Path: %s", this->root_path_.c_str());
  ESP_LOGCONFIG(TAG, "  Deletation Enabled: %s", TRUEFALSE(this->deletion_enabled_));
  ESP_LOGCONFIG(TAG, "  Download Enabled : %s", TRUEFALSE(this->download_enabled_));
  ESP_LOGCONFIG(TAG, "  Upload Enabled : %s", TRUEFALSE(this->upload_enabled_));
}

bool SDFileServer::canHandle(AsyncWebServerRequest *request) const {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  std::string url(request->url_to(url_buf));
  bool matches = str_startswith(url, this->build_prefix());
  ESP_LOGD(TAG, "can handle %s %u", url.c_str(), matches);
  return matches;
}

void SDFileServer::handleRequest(AsyncWebServerRequest *request) {
  char url_buf[AsyncWebServerRequest::URL_BUF_SIZE];
  std::string url(request->url_to(url_buf));
  ESP_LOGD(TAG, "%s", url.c_str());
  if (!str_startswith(url, this->build_prefix()))
    return;

  if (request->method() == HTTP_GET) {
    this->handle_get(request);
    return;
  }
  if (request->method() == HTTP_DELETE) {
    this->handle_delete(request);
    return;
  }

  // GET and DELETE are the only methods this handler implements. Anything else used to
  // fall through without sending a response at all, leaving the client to hang until its
  // own timeout. Reply explicitly instead.
#ifdef USE_ESP_IDF
  // The ESPHome shim maps any status it doesn't know to 500, so set 405 directly.
  httpd_req_t *req = *request;
  httpd_resp_set_status(req, "405 Method Not Allowed");
  httpd_resp_set_type(req, "application/json");
  set_connection_close(req);
  httpd_resp_send(req, "{ \"error\": \"method not allowed\" }", HTTPD_RESP_USE_STRLEN);
#else
  request->send(405, "application/json", "{ \"error\": \"method not allowed\" }");
#endif
}

void SDFileServer::handleUpload(AsyncWebServerRequest *request, const std::string &filename, size_t index, uint8_t *data,
                                size_t len, bool final) {
  if (!this->upload_enabled_) {
    request->send(401, "application/json", "{ \"error\": \"file upload is disabled\" }");
    return;
  }
  std::string extracted = this->extract_path_from_url(std::string(request->url().c_str()));
  std::string path = this->build_absolute_path(extracted);

  if (index == 0 && !this->sd_mmc_card_->is_directory(path)) {
    auto response = request->beginResponse(401, "application/json", "{ \"error\": \"invalid upload folder\" }");
    response->addHeader("Connection", "close");
    request->send(response);
    return;
  }
  std::string file_name(filename.c_str());
  if (index == 0) {
    ESP_LOGD(TAG, "uploading file %s to %s", file_name.c_str(), path.c_str());
    this->sd_mmc_card_->write_file(Path::join(path, file_name).c_str(), data, len);
    return;
  }
  this->sd_mmc_card_->append_file(Path::join(path, file_name).c_str(), data, len);
  if (final) {
    auto response = request->beginResponse(201, "text/html", "upload success");
    response->addHeader("Connection", "close");
    request->send(response);
    return;
  }
}

void SDFileServer::set_url_prefix(std::string const &prefix) { this->url_prefix_ = prefix; }

void SDFileServer::set_root_path(std::string const &path) { this->root_path_ = path; }

void SDFileServer::set_sd_mmc_card(sd_mmc_card::SdMmc *card) { this->sd_mmc_card_ = card; }

void SDFileServer::set_deletion_enabled(bool allow) { this->deletion_enabled_ = allow; }

void SDFileServer::set_download_enabled(bool allow) { this->download_enabled_ = allow; }

void SDFileServer::set_upload_enabled(bool allow) { this->upload_enabled_ = allow; }

void SDFileServer::handle_get(AsyncWebServerRequest *request) const {
  std::string extracted = this->extract_path_from_url(std::string(request->url().c_str()));
  std::string path = this->build_absolute_path(extracted);

  if (!this->sd_mmc_card_->is_directory(path)) {
    handle_download(request, path);
    return;
  }

  handle_index(request, path);
}

void SDFileServer::write_row(std::string &out, sd_mmc_card::FileInfo const &info) const {
  std::string uri = "/" + Path::join(this->url_prefix_, Path::remove_root_path(info.path, this->root_path_));
  std::string file_name = Path::file_name(info.path);
  out += "<tr><td>";
  if (info.is_directory) {
    out += "<a href=\"";
    out += uri;
    out += "\">";
    out += file_name;
    out += "</a>";
  } else {
    out += file_name;
  }
  out += "</td><td>";

  if (info.is_directory) {
    out += "Folder";
  } else {
    out += "<span class=\"file-type\">";
    out += Path::file_type(file_name);
    out += "</span>";
  }
  out += "</td><td>";
  if (!info.is_directory) {
    out += sd_mmc_card::format_size(info.size);
  }
  out += "</td><td><div class=\"file-actions\">";
  if (!info.is_directory) {
    if (this->download_enabled_) {
      out += "<a class=\"action-link\" href=\"";
      out += uri;
      out += "\" target=\"_blank\" rel=\"noopener\">Open</a>";
    }
    if (this->deletion_enabled_) {
      out += "<button onClick=\"delete_file('";
      out += uri;
      out += "')\">Delete</button>";
    }
  }
  out += "<div></td></tr>";
}

void SDFileServer::handle_index(AsyncWebServerRequest *request, std::string const &path) const {
  // A day folder can hold thousands of snapshots. Buffering the whole listing (or the
  // whole HTML page) in RAM first exhausts the heap on this non-PSRAM board and aborts
  // the device. Instead, stream the page in small chunks and render at most PAGE_SIZE
  // rows per request (?start=N selects the page), stopping the directory walk as soon as
  // the page is full.
  static constexpr size_t PAGE_SIZE = 100;

  size_t start = 0;
  std::string start_arg = request->arg("start");
  if (!start_arg.empty()) {
    char *end = nullptr;
    unsigned long parsed = strtoul(start_arg.c_str(), &end, 10);
    if (end != start_arg.c_str())
      start = parsed;
  }

#ifdef USE_ESP_IDF
  httpd_req_t *req = *request;
  httpd_resp_set_type(req, "text/html");
  httpd_resp_set_hdr(req, "Accept-Ranges", "none");
  set_connection_close(req);
  ChunkWriter out(req);
#else
  AsyncResponseStream *response = request->beginResponseStream("text/html");
  ChunkWriter out(response);
#endif

  out.write(INDEX_HEAD);

  std::string current_path = "/";
  std::string relative_path = Path::join(this->url_prefix_, Path::remove_root_path(path, this->root_path_));
  std::vector<std::string> parts = Path::split_path(relative_path);
  for (std::string const &part : parts) {
    if (!part.empty()) {
      current_path = Path::join(current_path, part);
      out.write("<a href=\"");
      out.write(current_path.data(), current_path.size());
      out.write("\">");
      out.write(part.data(), part.size());
      out.write("</a>");
    }
  }
  out.write("</div>");

  if (this->upload_enabled_)
    out.write(UPLOAD_FORM);

  out.write(INDEX_TABLE_HEAD);

  size_t index = 0;
  size_t shown = 0;
  bool has_more = false;
  std::string row;
  this->sd_mmc_card_->list_directory_file_info(path, 0, [&](sd_mmc_card::FileInfo const &info) {
    if (index++ < start)
      return true;  // skip the rows of earlier pages
    if (shown >= PAGE_SIZE) {
      has_more = true;
      return false;  // page is full: stop the directory walk
    }
    row.clear();
    write_row(row, info);
    out.write(row.data(), row.size());
    shown++;
    return true;
  });

  out.write(INDEX_FOOT);

  if (start > 0 || has_more) {
    std::string base = "/" + Path::join(this->url_prefix_, Path::remove_root_path(path, this->root_path_));
    out.write("<div class=\"pager\">");
    if (start > 0) {
      std::string prev = "<a class=\"action-link\" href=\"" + base + "?start=" +
                         std::to_string(start > PAGE_SIZE ? start - PAGE_SIZE : 0) + "\">&laquo; Previous</a>";
      out.write(prev.data(), prev.size());
    }
    if (has_more) {
      std::string next = "<a class=\"action-link\" href=\"" + base + "?start=" + std::to_string(start + PAGE_SIZE) +
                         "\">Next &raquo;</a>";
      out.write(next.data(), next.size());
    }
    out.write("</div>");
  }

  out.write("</body></html>");
  out.finish();

#ifndef USE_ESP_IDF
  request->send(response);
#endif
}

void SDFileServer::handle_download(AsyncWebServerRequest *request, std::string const &path) const {
  if (!this->download_enabled_) {
    request->send(401, "application/json", "{ \"error\": \"file download is disabled\" }");
    return;
  }

  // Keep the content-type string alive until the response has been sent: httpd stores the
  // pointer as-is.
  std::string mime = Path::mime_type(path);
#ifdef USE_ESP_IDF
  // Stream the file in small chunks instead of reading it into memory first: a whole-file
  // buffer (~20 KB per snapshot) does not fit in this board's heap, and a failed
  // allocation aborts the device (see components/README.md).
  std::string absolute_path = sd_mmc_card::build_path(path.c_str());
  FILE *file = fopen(absolute_path.c_str(), "rb");
  if (file == nullptr) {
    ESP_LOGE(TAG, "Failed to open file for reading: %s (%s)", absolute_path.c_str(), strerror(errno));
    request->send(401, "application/json", "{ \"error\": \"failed to read file\" }");
    return;
  }

  httpd_req_t *req = *request;
  httpd_resp_set_type(req, mime.c_str());
  httpd_resp_set_hdr(req, "Accept-Ranges", "none");
  set_connection_close(req);
  set_tcp_nodelay(req);

  // 2 KB chunks keep the per-chunk framing and syscall overhead down while staying well
  // within the heap budget.
  static char buffer[2048];
  size_t read = 0;
  while ((read = fread(buffer, 1, sizeof(buffer), file)) > 0) {
    if (httpd_resp_send_chunk(req, buffer, read) != ESP_OK)
      break;
  }
  fclose(file);
  httpd_resp_send_chunk(req, nullptr, 0);
#else
  auto file = this->sd_mmc_card_->read_file(path);
  if (file.size() == 0) {
    request->send(401, "application/json", "{ \"error\": \"failed to read file\" }");
    return;
  }
  auto *response = request->beginResponseStream(mime.c_str());
  response->print(std::string(file.begin(), file.end()).c_str());
  request->send(response);
#endif
}

void SDFileServer::handle_delete(AsyncWebServerRequest *request) {
  if (!this->deletion_enabled_) {
    request->send(401, "application/json", "{ \"error\": \"file deletion is disabled\" }");
    return;
  }
  std::string extracted = this->extract_path_from_url(std::string(request->url().c_str()));
  std::string path = this->build_absolute_path(extracted);
  if (this->sd_mmc_card_->is_directory(path)) {
    request->send(401, "application/json", "{ \"error\": \"cannot delete a directory\" }");
    return;
  }
  if (this->sd_mmc_card_->delete_file(path)) {
    request->send(204, "application/json", "{}");
    return;
  }
  request->send(401, "application/json", "{ \"error\": \"failed to delete file\" }");
}

std::string SDFileServer::build_prefix() const {
  if (this->url_prefix_.length() == 0 || this->url_prefix_.at(0) != '/')
    return "/" + this->url_prefix_;
  return this->url_prefix_;
}

std::string SDFileServer::extract_path_from_url(std::string const &url) const {
  std::string prefix = this->build_prefix();
  return url.substr(prefix.size(), url.size() - prefix.size());
}

std::string SDFileServer::build_absolute_path(std::string relative_path) const {
  if (relative_path.size() == 0)
    return this->root_path_;

  std::string absolute = Path::join(this->root_path_, relative_path);
  return absolute;
}

std::string Path::file_name(std::string const &path) {
  size_t pos = path.rfind(Path::separator);
  if (pos != std::string::npos) {
    return path.substr(pos + 1);
  }
  return "";
}

bool Path::is_absolute(std::string const &path) { return path.size() && path[0] == separator; }

bool Path::trailing_slash(std::string const &path) { return path.size() && path[path.length() - 1] == separator; }

std::string Path::join(std::string const &first, std::string const &second) {
  std::string result = first;
  if (!trailing_slash(first) && !is_absolute(second)) {
    result.push_back(separator);
  }
  if (trailing_slash(first) && is_absolute(second)) {
    result.pop_back();
  }
  result.append(second);
  return result;
}

std::string Path::remove_root_path(std::string path, std::string const &root) {
  if (!str_startswith(path, root))
    return path;
  if (path.size() == root.size() || path.size() < 2)
    return "/";
  return path.erase(0, root.size());
}

std::vector<std::string> Path::split_path(std::string path) {
  std::vector<std::string> parts;
  size_t pos = 0;
  while ((pos = path.find('/')) != std::string::npos) {
    std::string part = path.substr(0, pos);
    if (!part.empty()) {
      parts.push_back(part);
    }
    path.erase(0, pos + 1);
  }
  parts.push_back(path);
  return parts;
}

std::string Path::extension(std::string const &file) {
  size_t pos = file.find_last_of('.');
  if (pos == std::string::npos)
    return "";
  return file.substr(pos + 1);
}

std::string Path::file_type(std::string const &file) {
  static const std::map<std::string, std::string> file_types = {
      {"mp3", "Audio (MP3)"},   {"wav", "Audio (WAV)"}, {"png", "Image (PNG)"},   {"jpg", "Image (JPG)"},
      {"jpeg", "Image (JPEG)"}, {"bmp", "Image (BMP)"}, {"txt", "Text (TXT)"},    {"log", "Text (LOG)"},
      {"csv", "Text (CSV)"},    {"html", "Web (HTML)"}, {"css", "Web (CSS)"},     {"js", "Web (JS)"},
      {"json", "Data (JSON)"},  {"xml", "Data (XML)"},  {"zip", "Archive (ZIP)"}, {"gz", "Archive (GZ)"},
      {"tar", "Archive (TAR)"}, {"mp4", "Video (MP4)"}, {"avi", "Video (AVI)"},   {"webm", "Video (WEBM)"}};

  std::string ext = Path::extension(file);
  if (ext.empty())
    return "File";

  std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
  auto it = file_types.find(ext);
  if (it != file_types.end())
    return it->second;
  return "File (" + ext + ")";
}

std::string Path::mime_type(std::string const &file) {
  static const std::map<std::string, std::string> file_types = {
      {"mp3", "audio/mpeg"},        {"wav", "audio/vnd.wav"},   {"png", "image/png"},       {"jpg", "image/jpeg"},
      {"jpeg", "image/jpeg"},       {"bmp", "image/bmp"},       {"txt", "text/plain"},      {"log", "text/plain"},
      {"csv", "text/csv"},          {"html", "text/html"},      {"css", "text/css"},        {"js", "text/javascript"},
      {"json", "application/json"}, {"xml", "application/xml"}, {"zip", "application/zip"}, {"gz", "application/gzip"},
      {"tar", "application/x-tar"}, {"mp4", "video/mp4"},       {"avi", "video/x-msvideo"}, {"webm", "video/webm"}};

  std::string ext = Path::extension(file);
  ESP_LOGD(TAG, "ext : %s", ext.c_str());
  if (!ext.empty()) {
    std::transform(ext.begin(), ext.end(), ext.begin(), [](unsigned char c) { return std::tolower(c); });
    auto it = file_types.find(ext);
    if (it != file_types.end())
      return it->second;
  }
  return "application/octet-stream";
}

}  // namespace sd_file_server
}  // namespace esphome