#pragma once

#include <cstdint>
#include "esp_camera.h"
#include <vector>

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
        inline static const uint16_t black_565 = 0x0000;
        
        inline uint16_t pixel(int16_t col, int16_t row) const {
            //guard against negative coordinates
            const int16_t fx = origin_x + col;
            const int16_t fy = origin_y + row;
            // out of frame-bounds? also return black pixel
            if (fx < 0 || fy < 0 || fx >= frame_width || fy >= frame_height) {
                return black_565;
            }

            const uint8_t* p = origin + fy * stride + fx * bytes_per_px;
            return (uint16_t(p[1]) << 8) | p[0];
        }
    };

    class UnionFind {
        // copied from https://www.geeksforgeeks.org/dsa/introduction-to-disjoint-set-data-structure-or-union-find-algorithm/
        std::vector<uint16_t> parent;
        //TODO: use of psram might be necessary.
    public:
        UnionFind(int size) {
            parent.resize(size);
            // Initialize the parent array with each element as its own representative
            for (int i = 0; i < size; i++) {
                parent[i] = i;
            }
        }

        // Find the representative (root) of the set that includes element i
        int find(int i) {
            int root = i;
            while (parent[root] != root) {
                root = parent[root];
            }
            // path compression
            while (parent[i] != root) {
                int next = parent[i];
                parent[i] = root;
                i = next;
            }
            return root;
        }

        // Unite (merge) the set that includes element i and the set that includes element j
        void unite(int i, int j) {
            // Representative of set containing i
            int irep = find(i);
            // Representative of set containing j
            int jrep = find(j);
            // Make the representative of i's set
            // be the representative of j's set
            parent[irep] = jrep;
        }

        int size(){
            return parent.size();
        }
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

    float bee_activity_index(std::vector<uint8_t> classification_results);

    CropView make_crop(const camera_fb_t* frame, int16_t x,  int16_t y, uint16_t n);

    void unpack_565(uint16_t px, uint8_t& r, uint8_t& g, uint8_t& b);
}