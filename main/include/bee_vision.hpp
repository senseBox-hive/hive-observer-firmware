#pragma once

#include <cstdint>
#include "esp_camera.h"
#include <vector>
#include <tuple>

namespace bee_vision {
    struct CropView { //a cropject
        const uint8_t* origin; //pointer to RGB565 px buffer
        int16_t origin_x; //x coordinate of the top left corner of the crop in the original frame
        int16_t origin_y; //y coordinate of the top left corner of the crop in
        uint16_t stride; //in bytes
        uint16_t size;
        uint16_t frame_width;
        uint16_t frame_height;
        uint8_t bytes_per_px; // grayscale:1, RGB565:2, RGB888:3
        // black pixel
        static const uint16_t black_565;

        uint16_t pixel(int16_t col, int16_t row) const;
    };

    class UnionFind {
        std::vector<uint16_t> parent;
    public:
        UnionFind(int size);
        int find(int i);
        void unite(int i, int j);
        int size();
    };

    // ring buffer of activity for bee activity rolling average
    class ActivityBuffer {
    private:
        std::vector<std::tuple<uint32_t ,std::vector<uint8_t>>> buffer; //tuple: timestamp and inferences
        int front;
        int back;
        int capacity; // in this case, 60000 milliseconds. 1 Minute

    public:
        ActivityBuffer(int _capacity);
        void push_back(std::vector<uint8_t> element);
        std::tuple<uint32_t ,std::vector<uint8_t>> getFront();

        // get the last 1 minute of the buffer

        // Function to check if the buffer is empty
        bool empty() const;

        // Function to check if the buffer is full
        bool full() const;

        // Function to get the size of the buffer
        int size() const;
    };


    static inline int border_reflect(int p, int len) {
        if (p < 0)     return -p;
        if (p >= len)  return 2 * len - p - 2;
        return p;
    };
    
    // potentially use a more complex struct if you need to convey other info such as coordinates too
    std::vector<uint8_t> classify_frame(const camera_fb_t* frame);

    std::vector<CropView> candidate_crops(
        const camera_fb_t* frame, 
        uint8_t saturation_threshold, 
        uint8_t min_area = 16, 
        uint8_t cropsize = 32, 
        uint16_t max_area = 800
    );

    uint8_t classify_crop(const CropView& candidate);

    bool initialize_bee_model();

    bool model_initialized();

    float frame_bee_activity_index(std::vector<uint8_t> classification_results);

    CropView make_crop(const camera_fb_t* frame, int16_t x,  int16_t y, uint16_t n);

    void unpack_565(uint16_t px, uint8_t& r, uint8_t& g, uint8_t& b);
}