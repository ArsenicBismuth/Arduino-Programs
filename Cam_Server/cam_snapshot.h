#pragma once

// ---------------------------------------------------------------------------
// Periodic-snapshot helpers for the non-PSRAM ESP32-CAM (used from cam.yaml).
//
// Two parts:
//
//   1. cam_gate::analyze()  - change detection. Decodes the camera JPEG at 1/8
//      scale with the ESP32 ROM Tiny JPEG Decoder (TJpgDec) and folds the
//      result straight into a 16x12 luminance thumbnail. Only a fixed ~4 KB
//      decoder pool plus a few hundred bytes of grid state are used - nothing
//      scales with image resolution, so this is safe on a board with no PSRAM
//      (where any frame-sized heap allocation aborts; see components/README.md).
//
//   2. cam_store::prune()   - age-based retention. Walks the snapshot tree one
//      directory entry at a time and deletes a bounded batch of old files per
//      call, so it never builds a directory listing in RAM. The daily caller
//      (cam.yaml) loops it until the tree is drained.
//
// Path convention: the sd_mmc_card component's own methods take card-relative
// paths ("/snap", it prepends "/sdcard"); cam_store works with absolute VFS
// paths ("/sdcard/...") because it calls fopen()/remove() directly.
// ---------------------------------------------------------------------------

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <ctime>

#include <dirent.h>
#include <sys/stat.h>
#include <unistd.h>

extern "C" {
#include "rom/tjpgd.h"
}

namespace cam_gate {

static constexpr int GRID_W = 16;
static constexpr int GRID_H = 12;
static constexpr int GRID_N = GRID_W * GRID_H;

// Scratchpool for the ROM TJpgDec. The decoder wants ~3100 bytes regardless of
// image size; 4096 leaves headroom.
static constexpr size_t POOL_SIZE = 4096;

struct State {
  uint8_t pool[POOL_SIZE];
  uint32_t acc[GRID_N];  // per-cell luma accumulator
  uint16_t cnt[GRID_N];  // per-cell pixel count
  uint8_t cur[GRID_N];   // thumbnail of the frame just analysed
  uint8_t prev[GRID_N];  // thumbnail of the last frame we kept
  bool have_prev{false};
  int last_score{0};
};

inline State &state() {
  static State s;
  return s;
}

struct InputCtx {
  const uint8_t *data;
  size_t len;
  size_t pos;
};

inline unsigned int input_cb(JDEC *jd, BYTE *buff, unsigned int nbyte) {
  auto *ctx = static_cast<InputCtx *>(jd->device);
  size_t avail = ctx->len - ctx->pos;
  size_t n = nbyte;
  if (n > avail)
    n = avail;
  if (buff != nullptr && n > 0)
    memcpy(buff, ctx->data + ctx->pos, n);
  ctx->pos += n;
  return static_cast<unsigned int>(n);
}

inline unsigned int output_cb(JDEC *jd, void *bitmap, JRECT *rect) {
  State &st = state();
  const uint32_t out_w = jd->width >> jd->scale;
  const uint32_t out_h = jd->height >> jd->scale;
  if (out_w == 0 || out_h == 0)
    return 0;

  uint8_t *p = static_cast<uint8_t *>(bitmap);  // RGB888, contiguous MCU block
  const uint32_t rw = rect->right - rect->left + 1;
  const uint32_t rh = rect->bottom - rect->top + 1;
  for (uint32_t j = 0; j < rh; j++) {
    uint32_t gy = (rect->top + j) * GRID_H / out_h;
    if (gy >= GRID_H)
      gy = GRID_H - 1;
    const uint32_t row = gy * GRID_W;
    for (uint32_t i = 0; i < rw; i++) {
      const uint8_t r = p[0], g = p[1], b = p[2];
      p += 3;
      uint32_t gx = (rect->left + i) * GRID_W / out_w;
      if (gx >= GRID_W)
        gx = GRID_W - 1;
      const uint8_t luma = static_cast<uint8_t>((r * 77u + g * 150u + b * 29u) >> 8);
      st.acc[row + gx] += luma;
      st.cnt[row + gx]++;
    }
  }
  return 1;  // keep going
}

// Decode `jpeg` and return the mean absolute luma difference (0..255) against
// the last committed frame, storing the new thumbnail in `cur`.
//   -1  -> could not decode (callers should treat this as "changed")
//   255 -> first frame after boot (nothing to compare against)
inline int analyze(const uint8_t *jpeg, size_t len) {
  State &st = state();
  memset(st.acc, 0, sizeof(st.acc));
  memset(st.cnt, 0, sizeof(st.cnt));

  InputCtx ctx{jpeg, len, 0};
  JDEC jd;
  if (jd_prepare(&jd, input_cb, st.pool, POOL_SIZE, &ctx) != JDR_OK)
    return -1;
  if (jd_decomp(&jd, output_cb, 3) != JDR_OK)  // 3 == 1/8 scale
    return -1;

  int total = 0;
  int cells = 0;
  for (int i = 0; i < GRID_N; i++) {
    if (st.cnt[i] == 0)
      continue;
    const int avg = static_cast<int>(st.acc[i] / st.cnt[i]);
    st.cur[i] = static_cast<uint8_t>(avg);
    if (st.have_prev) {
      int d = avg - static_cast<int>(st.prev[i]);
      total += d < 0 ? -d : d;
      cells++;
    }
  }

  if (!st.have_prev) {
    memcpy(st.prev, st.cur, GRID_N);
    st.have_prev = true;
    st.last_score = 255;
    return 255;
  }
  st.last_score = cells > 0 ? total / cells : 0;
  return st.last_score;
}

// Adopt the frame just analysed as the new reference.
inline void commit() { memcpy(state().prev, state().cur, GRID_N); }
inline void reset() { state().have_prev = false; }
inline int last_score() { return state().last_score; }

}  // namespace cam_gate

namespace cam_store {

inline bool write_file(const char *abs_path, const uint8_t *data, size_t len) {
  FILE *f = fopen(abs_path, "wb");
  if (f == nullptr)
    return false;
  const size_t written = fwrite(data, 1, len, f);
  fclose(f);
  return written == len;
}

// Delete up to `budget` files older than `retention_days` under `root`, then
// drop any day folder that is now empty. Streams the tree entry by entry, so
// the memory cost is a couple of path buffers, not a listing. Returns how many
// files were removed. A return count equal to `budget` means the caller should
// call it again to keep draining (cam.yaml loops it once a day).
inline int prune(const char *root, int retention_days, int budget) {
  // NB: qualified as std::time because a `using namespace esphome;` in the
  // generated main.cpp makes the unqualified name ambiguous with esphome::time.
  const time_t now = std::time(nullptr);
  if (now < 1600000000)  // RTC not synced yet - leave files alone
    return 0;
  const time_t cutoff = now - static_cast<time_t>(retention_days) * 86400;

  DIR *dir = opendir(root);
  if (dir == nullptr)
    return 0;

  int deleted = 0;
  char sub[320];
  char file[384];
  struct dirent *entry;
  while (deleted < budget && (entry = readdir(dir)) != nullptr) {
    if (entry->d_name[0] == '.')
      continue;
    snprintf(sub, sizeof(sub), "%s/%s", root, entry->d_name);

    struct stat info;
    if (stat(sub, &info) != 0 || !S_ISDIR(info.st_mode))
      continue;

    DIR *day = opendir(sub);
    if (day != nullptr) {
      struct dirent *f;
      while (deleted < budget && (f = readdir(day)) != nullptr) {
        if (f->d_name[0] == '.')
          continue;
        snprintf(file, sizeof(file), "%s/%s", sub, f->d_name);
        struct stat fi;
        if (stat(file, &fi) != 0 || S_ISDIR(fi.st_mode))
          continue;
        if (fi.st_mtime != 0 && fi.st_mtime < cutoff && remove(file) == 0)
          deleted++;
      }
      closedir(day);
    }
    rmdir(sub);  // only succeeds once the folder is empty
  }
  closedir(dir);
  return deleted;
}

}  // namespace cam_store
