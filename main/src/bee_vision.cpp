#include "bee_vision.hpp"
#include <cstdint>
#include <vector>
#include "esp_camera.h"
#include "esp_heap_caps.h"

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
namespace bee_vision {

std::vector<uint8_t> classify_frame(const camera_fb_t* frame){
    std::vector<uint8_t> results = {};

    std::vector<CropView> candidates = candidate_crops(frame);

    if(candidates.size() == 0) {
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
    //determine stride from color type
    uint16_t stride;
    uint8_t bytes_per_px;
    switch (frame->format) {
        case PIXFORMAT_RGB888:      bytes_per_px = 3; break;
        case PIXFORMAT_RGB565:      bytes_per_px = 2; break;
        case PIXFORMAT_GRAYSCALE:   bytes_per_px = 1; break;
        default:                    bytes_per_px = 0; break;
    }
    stride = frame->width * bytes_per_px;

    CropView crop {
        frame->buf + (y * stride) + (x * bytes_per_px),
        stride,
        n, //size
        static_cast<uint16_t>(frame->width),
        static_cast<uint16_t>(frame->height),
        bytes_per_px
    };
    return crop;
}

candidate_crops(const camera_fb_t* frame, uint8_t saturation_threshold){
    // This is where the operation previously implemented in python is performed
    // read image
    size_t amount_px = frame->width * frame->height;
    uint8_t* labelmask = (uint8_t*) heaps_caps_malloc(amount_px, MALLOC_CAP_SPIRAM);
    UntionFind unions = UnionFind(255)
    uint8_t current_label_id = 1;
    // for now we just assume 255 possible labels. if more are found, assume something is wrong

    if(!labelmask){
        ESP_LOGE("CNN", "Failed to allocate mask memory");
    }

    // original script used blur k size 0, so the gaussian blur step is skipped for now.
    // if new training data is created that makes use of the blur step, it needs to be added here with the same params.
    for (int i = 0; i<amount_px; i++) {
        //*unswaps your bytes*
        uint8_t first   = frame->buf[i*2 + 0];
        uint8_t second  = frame->buf[i*2 + 1];
        uint16_t px = (second << 8) | first;

        // RRRRRGGG GGGBBBBB -> GGGGGGBB BBBRRRRR & 00011111mask = 000RRRRR (what happens to the first byte?) 
        uint8_t r = (px >> 11) & 0x1F;  
        uint8_t g = (px >> 5)  & 0x3F;  // 6    00111111 mask
        uint8_t b =  px        & 0x1F;  // 5    00011111 mask
        // convert to 8bit
        r = (r << 3) | (r >> 2); // 000rrrrr: (rrrrr000) | (00000rrr) -> rrrrrrrr
        g = (g << 2) | (g >> 4);
        b = (b << 3) | (b >> 2);

        // compute saturation and apply threshold
        uint8_t mx = std::max({r, g, b});
        uint8_t mn = std::min({r, g, b});
        uint8_t sat = mx - mn;

        // do the first pass of the 2 pass CCL algorithm
        if (sat > saturation_threshold) {
            // TODO: prevent out of bounds
            bool isMostWest = (i % frame->width == 0);
            bool isMostNorth = (i < frame->width);
            labelmask[i] = current_label_id;
            current_label_id++;
            // check west pixel
            if (!isMostWest && (labelmask[i-1] != 0)){
                //compare north and west
                if (!isMostNorth && (labelmask[i-frame->width] != 0)) {
                    if (labelmask[i-1] != labelmask[i-frame->width]){
                        unions.unite(labelmask[i-frame->width], labelmask[i-1]);
                    }
                }
                labelmask[i] = labelmask[i-1];
                current_label_id--; //no new label created after all
            // if no west label is found check once more for north label
            } else if(!isMostNorth && (labelmask[i-frame->width] != 0)) {
                labelmask[i] = labelmask[i-frame->width];
                current_label_id--; //no new label created after all
            }
        } else {
            labelmask[i] = 0;
        }
    }

    // determine saturation hotspots
    // flood fill is how the python prototype was implemented
    // consider using a more efficient algo at a later point
    for (int i; i<amount_px; i++) {
        if (mask[i] == 0xFF ) { //unvisited
            uint8_t area = 0;
            uint8_t sum_x = 0;
            uint8_t sum_y = 0;

            uint8_t x = i % frame->width;
            uint8_t y = i / frame->width;


        }
    }

    // create 30x30 crops around saturation hotspots and return
    heap_caps_free(mask)
}

}