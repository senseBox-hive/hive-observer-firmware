#pragma once

#include <cstdint>
#include "esp_camera.h"

namespace bee_vision {
    // potentially use a more complex struct if you need to convey other info such as coordinates too
    std::vector<uint8_t> classify_frame(const camera_fb_t* frame);

    std::vector<CropView> candidate_crops(const camera_fb_t* frame);

    uint8_t classify_crop(const CropView candidate);

    float bee_activity_index(std::vector<uint8_t> classification_results);

    struct CropView { //a cropject
        uint8_t origin;
        uint16_t stride; //in bytes
        uint16_t size;
        uint8_t bytes_per_px; // grayscale:1, RGB565:2, RGB888:3

        inline uint8_t* pixel(uint16_t col, uint16_t row) const {
            return origin + (row * stride_bytes) + (col * bytes_per_px);
        }
    }

    CropView make_crop(const camera_fb_t* frame, uint16_t x,  uint16_t y, uint16_t n);

}