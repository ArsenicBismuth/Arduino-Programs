#pragma once

// Helpers shared by the camera runtime controls in cam.yaml.
// Pulled into every lambda via the `esphome: includes:` option.

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
