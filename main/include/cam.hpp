#pragma once

#include "esp_camera.h"
#include "esp_log.h"
#include <esp_system.h>
#include "include/camera_pins.h"
#include "esp_heap_caps.h"

namespace cam {

esp_err_t init_camera(void);

camera_fb_t* combine_grayscale_to_rgb(const camera_fb_t *r,
                                      const camera_fb_t *g,
                                      const camera_fb_t *b);

void free_fb(camera_fb_t *fb);

camera_fb_t* capture_rgb_sequence();

}