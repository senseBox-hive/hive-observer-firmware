#include "bee_vision.hpp"

#include <cstdint>
#include <vector>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <limits>
#include <map>

#include "esp_camera.h"
#include "esp_heap_caps.h"
#include "esp_log.h"

#include "dl_model_base.hpp"

/** 
THE PLAN

1. motion-saturation detection
    check for colored dots. The parameters for that one are alrady in 
2. Pass 32x32 candidates to CNN for classification

this requires PIXFORMAT_RGB565
*/ 
namespace bee_vision {

const uint16_t CropView::black_565 = 0x0000;

uint16_t CropView::pixel(int16_t col, int16_t row) const {
    if (col < 0 || row < 0 || col >= size || row >= size) {
        return black_565;
    }

    const int16_t fx = origin_x + col;
    const int16_t fy = origin_y + row;
    if (fx < 0 || fy < 0 || fx >= frame_width || fy >= frame_height) {
        return black_565;
    }

    const int16_t frame_x = origin_x < 0 ? 0 : origin_x;
    const int16_t frame_y = origin_y < 0 ? 0 : origin_y;
    const uint8_t* p = origin
        + (fy - frame_y) * stride
        + (fx - frame_x) * bytes_per_px;
    return (uint16_t(p[1]) << 8) | p[0];
}

UnionFind::UnionFind(int size) {
    parent.resize(size);
    for (int i = 0; i < size; i++) {
        parent[i] = i;
    }
}

int UnionFind::find(int i) {
    int root = i;
    while (parent[root] != root) {
        root = parent[root];
    }
    while (parent[i] != root) {
        int next = parent[i];
        parent[i] = root;
        i = next;
    }
    return root;
}

void UnionFind::unite(int i, int j) {
    int irep = find(i);
    int jrep = find(j);
    parent[irep] = jrep;
}

int UnionFind::size() {
    return parent.size();
}


extern const uint8_t espdl_bee_model[] asm("_binary_model_beeactivity_espdl_start");
dl::Model *bee_model = nullptr;
int8_t model_input[32*32*3]; //input buffer for the model, RGB888, 32x32

std::vector<uint8_t> classify_frame(const camera_fb_t* frame){
    std::vector<uint8_t> results = {};

    //ESP_LOGI("CNN", "getting candidate crops from frame");
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
        uint8_t classified = classify_crop(candidates[i]);
        // TODO: error when classification fails

        if (classified > 3) {
            ESP_LOGE("CNN", "Failed to classify frame");
            return results;
        }

        results.push_back(classified);
    }

    return results;
}

// This is where the CNN is used
uint8_t classify_crop(const CropView& candidate){
    if (!bee_model) {
        ESP_LOGE("CNN", "Failed to classify crop: no model initialized");
        return 9;
    }

    //ESP_LOGI("CNN", "Classifying crop at (%d, %d) with size %dx%d", candidate.origin_x, candidate.origin_y, candidate.size, candidate.size);
    //measure time to convert to tensor and run inference
    for (int row = 0; row < 32; row++) {
      for (int col = 0; col < 32; col++) {
        uint16_t px = candidate.pixel(col, row);
        uint8_t r, g, b;
        unpack_565(px, r, g, b);                    // byte order + bit replication

        // ToTensors /255, exponent -7 -> ×2^7 = ×128, clamp
        int qr = std::clamp((int)lroundf(r / 255.0f * 128.0f), -127, 127);
        int qg = std::clamp((int)lroundf(g / 255.0f * 128.0f), -127, 127);
        int qb = std::clamp((int)lroundf(b / 255.0f * 128.0f), -127, 127);

        int base = (row * 32 + col) * 3;            // HWC interleaved (from .info: 1x32x32x3)
        model_input[base + 0] = (int8_t)qr;
        model_input[base + 1] = (int8_t)qg;
        model_input[base + 2] = (int8_t)qb;
      }
    }
    //ESP_LOGI("CNN", "Converted crop to tensor");

    // run inference here
    std::map<std::string, dl::TensorBase *> &model_inputs = bee_model->get_inputs();
    std::map<std::string, dl::TensorBase *> &model_outputs = bee_model->get_outputs();
    dl::TensorBase *input_tensor = model_inputs.begin()->second;
    dl::TensorBase *output_tensor = model_outputs.begin()->second;
    std::memcpy(input_tensor->get_element_ptr<int8_t>(), model_input, sizeof(model_input));
    //bee_model->run(dl::RUNTIME_MODE_SINGLE_CORE);
    bee_model->run(dl::RUNTIME_MODE_MULTI_CORE); 
        //using a model with 48 hidden layers instead of 100,
        // same configs otherwise, compared to the model in last commit
        // gets the job done in about 20-30ms and is only 1/3rd the size
    //ESP_LOGI("CNN", "Ran inference on crop");

    int class_id = 0;
    float best_score = -std::numeric_limits<float>::infinity();
    for (int i = 0; i < 3; ++i) {
        float score;
        switch (output_tensor->dtype) {
            case dl::DATA_TYPE_INT8:
                score = output_tensor->get_element_ptr<int8_t>()[i];
                break;
            default:
                ESP_LOGE("CNN", "Unsupported output type: %s",
                         output_tensor->get_dtype_string());
                return 9;
        }

        if (score > best_score) {
            best_score = score;
            class_id = i;
        }
    }

    return static_cast<uint8_t>(class_id);
}

bool initialize_bee_model() {
    bee_model = new dl::Model(
        (const char *)espdl_bee_model
    );
    if (!bee_model) {
        ESP_LOGE("CNN", "Failed to create model");
        return false;
    }
    
    esp_err_t test_result = bee_model->test();
    if ( test_result != ESP_OK ) {
        ESP_LOGE("CNN", "Model info check failed");
        delete bee_model;
        bee_model = nullptr;
        return false;
    }

    bee_model->profile(true);

    return true;
}

bool model_initialized() {
    return (bee_model != NULL);
}

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
    const size_t frame_x = x < 0 ? 0 : std::min<size_t>(x, frame->width);
    const size_t frame_y = y < 0 ? 0 : std::min<size_t>(y, frame->height);

    CropView crop {
        frame->buf + frame_y * stride + frame_x * bytes_per_px,
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

// helper function to convert a 16-bit RGB565 pixel to 8-bit RGB values
// considers the byte swap that I was told exists 
void unpack_565(uint16_t px, uint8_t& r, uint8_t& g, uint8_t& b) {
    // RRRRRGGG GGGBBBBB -> GGGGGGBB BBBRRRRR & 00011111
    uint8_t r5 = (px >> 11) & 0x1F;
    uint8_t g6 = (px >> 5)  & 0x3F; // 6    00111111 mask
    uint8_t b5 =  px        & 0x1F; // 5    00011111 mask
    r = (r5 << 3) | (r5 >> 2); // 000rrrrr: (rrrrr000) | (00000rrr) -> rrrrrrrr
    g = (g6 << 2) | (g6 >> 4);
    b = (b5 << 3) | (b5 >> 2);
}

std::vector<CropView> candidate_crops(
    const camera_fb_t* frame, 
    uint8_t saturation_threshold, 
    uint8_t min_area, 
    uint8_t cropsize,
    uint16_t max_area
){

    // This is where the operation previously implemented in python is performed
    const size_t amount_px = frame->width * frame->height;
    const size_t w = frame->width;
    const size_t h = frame->height;
    constexpr uint16_t MAX_LABELS = 1024;
    uint16_t* labelmask = (uint16_t*) heap_caps_malloc(amount_px * sizeof(uint16_t), MALLOC_CAP_SPIRAM);
    uint8_t* satmask = (uint8_t*) heap_caps_malloc(amount_px * sizeof(uint8_t), MALLOC_CAP_SPIRAM);
    UnionFind unions = UnionFind(38400); //38400 would be absolute worst case. unlikely to happen
    uint16_t current_label_id = 1;
    std::vector<CropView> crops;
    // experimentally, training data never yielded more than 808 blobs a frame.
    // assume cap at 1024

    if(!labelmask){
        ESP_LOGE("CNN", "Failed to allocate mask memory");
        heap_caps_free(labelmask);
        heap_caps_free(satmask);
        return crops;
    }
    if(!satmask){
        ESP_LOGE("CNN", "Failed to allocate saturation mask memory");
        heap_caps_free(satmask);
        return crops;
    }
    // since both this preprocessing, as well as the model
    // convert RGB565 to RGB888, consider using RGB888 framebuffers
    // in the first place.
    // TODO

    // original script used blur k size 0, so the gaussian blur step is skipped for now.
    // consider adding gaussian blur again if it increases performance over scanning excessive crops
    
    //compute saturation mask
    //ESP_LOGI("CNN", "Computing saturation mask");
    bool threshold_reached = false;
    for (int i = 0; i<amount_px; i++) {
        //*unswaps your bytes*
        uint8_t first   = frame->buf[i*2 + 0];
        uint8_t second  = frame->buf[i*2 + 1];
        uint16_t px = (second << 8) | first;

        uint8_t r, g, b;

        unpack_565(px, r, g, b);

        // compute saturation
        uint8_t mx = std::max({r, g, b});
        uint8_t mn = std::min({r, g, b});
        uint8_t sat = mx - mn;

        satmask[i] = sat;

        if (sat > saturation_threshold) {
            threshold_reached = true;
        } 
    }
    if (!threshold_reached) {
        ESP_LOGI("CNN", "No pixels exceeded saturation threshold");
        heap_caps_free(labelmask);
        heap_caps_free(satmask);
        return crops;
    }
    
    
    for (size_t y = 0; y < h; y++) {
        // rolling row-buffer
        const int previous_y = border_reflect(static_cast<int>(y) - 1, static_cast<int>(h));
        const int next_y = border_reflect(static_cast<int>(y) + 1, static_cast<int>(h));
        const uint8_t* rn = satmask + previous_y * w; //row-1
        const uint8_t* rc = satmask + y * w; //row
        const uint8_t* rs = satmask + next_y * w; //row+1

        for (size_t x = 0; x < w; x++) {
            // blur
            const int west_x = border_reflect(static_cast<int>(x) - 1, static_cast<int>(w));
            const int east_x = border_reflect(static_cast<int>(x) + 1, static_cast<int>(w));

            uint16_t acc =     rn[west_x] + 2 * rn[x] +     rn[east_x]
                         + 2 * rc[west_x] + 4 * rc[x] + 2 * rc[east_x]
                         +     rs[west_x] + 2 * rs[x] +     rs[east_x];
            uint8_t blurred = (acc + 8) >> 4;

            const size_t i = y * w + x;
            
            // 1st pass of the 2 pass CCL algorithm
            if (blurred > saturation_threshold) {
                bool isMostWest = (x == 0);
                bool isMostNorth = (y == 0);

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

        if (current_label_id > MAX_LABELS) {
            ESP_LOGE("CNN", "Label ID rolled over max labels, frame considered invalid");
            heap_caps_free(labelmask);
            heap_caps_free(satmask);
            return crops;
        }

    }
    heap_caps_free(satmask);

    // technically max uint16 65535 can be smaller than qvga area 76800. very unlikely.
    // safeguard with max area
    uint16_t* area  = (uint16_t*) heap_caps_calloc(MAX_LABELS + 1, sizeof(uint16_t), MALLOC_CAP_INTERNAL);
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
    //ESP_LOGI("CNN", "2nd pass ccl");
    for (size_t i = 0; i < amount_px; i++) {
        if (labelmask[i] != 0){
            //correct labels for all pixels
            uint16_t root = unions.find(labelmask[i]);
            labelmask[i] = root;
            size_t x = i % w;
            size_t y = i / w;

            //check max_area
            if (area[root] >= max_area ) continue;

            area[root]  += 1;
            sum_x[root] += x;
            sum_y[root] += y;
        }
    }

    // create NxN crops for each root label
    //ESP_LOGI("CNN", "NxN crops for each root of the %i root-labels", current_label_id);
    for (uint16_t label = 1; label <= MAX_LABELS; label++) {
        if (area[label] == 0) continue; // label is non-root
        if (area[label] < min_area) continue; // label is too small
        if (area[label] >= max_area) continue;
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