#include "bee_vision.hpp"
#include <cstdint>
#include "esp_camera.h"
#include "esp_camera.h"

/** 
THE PLAN

1. motion-saturation detection
    check for colored dots. The parameters for that one are alrady in 
2. Pass 30x30 candidates to CNN for classification
3. generate activity index from this.
    moving average of moving bees, slow bees,
    slow bees can indicate entrance activity, moving bees indicate in and outgoing bees


this requires PIXFORMAT_RGB565
*/ 

classify_frame(const camera_fb_t* frame){
    std::vector<uint8_t> results = {};

    std::vector<CropView> candidates = candidate_crops(frame);

    if(candidates.size == 0) {
        return results;
    }

    for (auto i = 0; i < candidates.size(); i++){
        uint8_t class = classify_crop(candidates[i])
        // TODO: error when classification fails
        results.push_back(class);
    }

    return results;
}

// This is where the CNN is used
classify_crop(const camera_fb_t* candidate){
    //TODO
}

bee_activity_index(std::vector<uint8_t> classification_results) {
    //perform maths on the resulting classes to estimate how busy the hive is
}

CropView make_crop(const camera_fb_t* frame, uint16_t x,  uint16_t y, uint16_t n){
    //TODO: add black pixels at frame edges, as that is in the training data too

    //determine stride from color type
    uint16_t stride;
    uint8_t bytes_per_px;
    switch (frame->format) {
        case PIXFORMAT_RGB888:      bytes_per_px = 3;
        case PIXFORMAT_RGB565:      bytes_per_px = 2;
        case PIXFORMAT_GRAYSCALE:   bytes_per_px = 1;
        default:                    bytes_per_px = 0;
    }
    stride = frame->width * bytes_per_px

    CropView crop;
    view.origin = frame->buf + (y * stride) + (x * bytes_per_px);
    view.stride = stride;
    view.size = size;
    view.bytes_per_px = bytes_per_px;
    return view;
}

candidate_crops(const camera_fb_t* frame){
    // This is where the operation previously implemented in python is performed

    // read image

    // compute saturation
    // blur saturation

    // determine saturation hotspots

    // create 30x30 crops around saturation hotspots and return
}