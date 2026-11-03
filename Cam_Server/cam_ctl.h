#pragma once

// Helpers shared by the camera runtime controls in cam.yaml.
// Pulled into every lambda via the `esphome: includes:` option.

#include <string>

#include "esp_camera.h"        // sensor_t, esp_camera_sensor_get()
#include "esphome/core/log.h"  // ESP_LOGW()

// Returns the active camera sensor. If the camera failed to initialise (e.g. a
// frame-buffer allocation failure) esp_camera_sensor_get() is null, and a
// warning is logged instead.
inline sensor_t *cam_sensor() {
  sensor_t *s = esp_camera_sensor_get();
  if (s == nullptr) {
    ESP_LOGW("cam_ctl", "Camera sensor unavailable");
  }
  return s;
}

// Fetch the sensor and bail out of the enclosing lambda if it's unavailable:
//   CAM_SENSOR_OR_RETURN();
//   s->set_brightness(s, (int) x);
#define CAM_SENSOR_OR_RETURN() \
  sensor_t *s = cam_sensor();  \
  if (s == nullptr)            \
    return

// --- AGC gain ceiling --------------------------------------------------------
// The 2X..128X steps of gainceiling_t are NOT a register value on every sensor.
// On the OV2640 the enum goes into COM9 (bits 5..7), so 2X really is a 2x
// ceiling. On the OV3660/OV5640 the driver's set_gainceiling() writes the enum
// number straight into 0x3A18/0x3A19, which is a Q4.4 ceiling there:
// 0x10 = 1x, 0x40 = 4x, 0xF8 = 15.5x (the driver's own power-on value). Step 0
// ("2X") therefore lands as 0x0000 - a ceiling below unity - which pins AGC at
// minimum gain and underexposes every frame.
//
// esp32_camera calls set_gainceiling() unconditionally in its init, using the
// YAML `agc_gain_ceiling:` value (default "2X"), so the OV3660 needs this write
// after the camera has been set up.
//
// `ceiling` is in Q4.4 units: 16 = 1x.
inline void cam_gain_ceiling_q44(sensor_t *s, uint16_t ceiling) {
  if (s == nullptr || s->set_reg == nullptr)
    return;
  if (ceiling > 0x3FF)
    ceiling = 0x3FF;  // 0x3A18 is 2 bits wide: 63.9x is the representable max
  s->set_reg(s, 0x3A18, 0x03, (ceiling >> 8) & 0x03);
  s->set_reg(s, 0x3A19, 0xFF, ceiling & 0xFF);
}

// Picks the ceiling encoding from the board file's `cam_sensor` substitution.
// Only the OV2640 uses enum-as-COM9; everything else is assumed to use the
// 0x3A18/0x3A19 Q4.4 form, so an unset/unrecognised value stays on the safer
// OV3660 path.
inline bool cam_sensor_uses_q44_ceiling(const char *name) {
  return std::string(name) != "ov2640";
}
