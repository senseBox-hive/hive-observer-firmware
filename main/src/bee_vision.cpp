#include "bee_vision.hpp"

#include <cstdint>
#include <vector>
#include <algorithm>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

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

    std::vector<CropView> candidates = candidate_crops(
        frame, 
        40, 
        16, 
        32, 
        800
    );

    if(candidates.size() == 0) {
        return results;
    }

    for (auto i = 0; i < candidates.size(); i++){
        uint8_t classifieds = classify_crop(candidates[i]);
        // TODO: error when classification fails
        results.push_back(classifieds);
    }

    return results;
}

// This is where the CNN is used
//classify_crop(const camera_fb_t* candidate){
//    //TODO
//}
//
//bee_activity_index(std::vector<uint8_t> classification_results) {
//    //perform maths on the resulting classes to estimate how busy the hive is
//}

CropView make_crop(const camera_fb_t* frame, int16_t x,  int16_t y, uint16_t n){
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
        x, //origin_x
        y, //origin_y
        stride,
        n, //size
        static_cast<uint16_t>(frame->width),
        static_cast<uint16_t>(frame->height),
        bytes_per_px
    };
    return crop;
}

std::vector<CropView> candidate_crops(
    const camera_fb_t* frame, 
    uint8_t saturation_threshold, 
    uint8_t min_area, 
    uint8_t cropsize,
    uint16_t max_area
){
    // This is where the operation previously implemented in python is performed
    size_t amount_px = frame->width * frame->height;
    uint16_t* labelmask = (uint16_t*) heap_caps_malloc(amount_px, MALLOC_CAP_SPIRAM);
    UnionFind unions = UnionFind(256);
    uint16_t current_label_id = 1;
    std::vector<CropView> crops;
    uint16_t MAX_LABELS = 2048;
    // for now we just assume 255 possible labels. if more are found, assume something is wrong

    if(!labelmask){
        ESP_LOGE("CNN", "Failed to allocate mask memory");
        heap_caps_free(labelmask);
        return crops;
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

        if (current_label_id > MAX_LABELS) {
            ESP_LOGE("CNN", "Label ID rolled over max labels, frame considered invalid");
            heap_caps_free(labelmask);
            return crops;
        }

        // guard current_label_id rolling over to 0, which would be interpreted as background
        if (current_label_id == 0) {
            ESP_LOGE("CNN", "Label ID rolled over to 0, frame considered invalid");
            heap_caps_free(labelmask);
            return crops;
        }
    }

    //uint32t is a bit wasteful. if it ends up too large for internal memory, switch to uint16_t and just check for overflow when adding to the sums.
    uint32_t* area  = (uint32_t*) heap_caps_calloc(MAX_LABELS + 1, sizeof(uint32_t), MALLOC_CAP_INTERNAL);
    uint32_t* sum_x = (uint32_t*) heap_caps_calloc(MAX_LABELS + 1, sizeof(uint32_t), MALLOC_CAP_INTERNAL);
    uint32_t* sum_y = (uint32_t*) heap_caps_calloc(MAX_LABELS + 1, sizeof(uint32_t), MALLOC_CAP_INTERNAL);
    if (!area || !sum_x || !sum_y) {
        ESP_LOGE("CNN", "failed to allocate memory for area/sum arrays");
        heap_caps_free(labelmask);
        heap_caps_free(area);
        heap_caps_free(sum_x);
        heap_caps_free(sum_y);
        return crops;
     }

    // pass 2
    for(int i = 0; i<amount_px; i++){
        if (labelmask[i] != 0){
            //correct labels for all pixels
            uint16_t root = unions.find(labelmask[i]);
            labelmask[i] = root;
            uint16_t x = i % frame->width;
            uint16_t y = i / frame->width;

            area[root]  += 1;
            sum_x[root] += x;
            sum_y[root] += y;
        }
    }

    // create NxN crops for each root label
    for(uint16_t label = 1; label<unions.size(); label++){
        if (area[label] == 0) continue; // label is non-root
        if (area[label] < min_area) continue; // label is too small
        int cx = sum_x[label] / area[label];
        int cy = sum_y[label] / area[label];

        // get necessary values for make crop
        int16_t topleft_x = cx - (cropsize/2);
        int16_t topleft_y = cy - (cropsize/2);

        // CROP THE BLOB
        crops.push_back(
            make_crop(frame, topleft_x, topleft_y, cropsize)
        );
    }
    heap_caps_free(labelmask);
    heap_caps_free(area);
    heap_caps_free(sum_x);
    heap_caps_free(sum_y);

    return crops;
}

}