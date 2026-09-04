#pragma once

#include <cstdint>
#include "esp_camera.h"
#include <vector>

namespace bee_vision {
    struct CropView { //a cropject
        const uint8_t* origin; //pointer to RGB565 px buffer
        uint16_t stride; //in bytes
        uint16_t size;
        uint16_t frame_width;
        uint16_t frame_height;
        uint8_t bytes_per_px; // grayscale:1, RGB565:2, RGB888:3
        // black pixel
        inline static const uint16_t black_565 = 0x0000;
        
        inline const uint16_t* pixel(int16_t col, int16_t row) const {
            // out of bounds? return black pixel
            if (row >= size || col >= size || row < 0 || col < 0) {
                return &black_565; 
            }

            // offset the px buffer by the given values
            return reinterpret_cast<const uint16_t*>(
                origin + (row * stride) + (col * bytes_per_px)
            );
        }
    };

    class UnionFind {
        // copied from https://www.geeksforgeeks.org/dsa/introduction-to-disjoint-set-data-structure-or-union-find-algorithm/
        vector<int> parent;
    public:
        UnionFind(int size) {
            parent.resize(size);
            // Initialize the parent array with each 
            // element as its own representative
            for (int i = 0; i < size; i++) {
                parent[i] = i;
            }
        }

        // Find the representative (root) of the
        // set that includes element i
        int find(int i) {
            // If i itself is root or representative
            if (parent[i] == i) {
                return i;
            }
            // Else recursively find the representative 
            // of the parent
            return find(parent[i]);
        }

        // Unite (merge) the set that includes element 
        // i and the set that includes element j
        void unite(int i, int j) {
            // Representative of set containing i
            int irep = find(i);
            // Representative of set containing j
            int jrep = find(j);
            // Make the representative of i's set
            // be the representative of j's set
            parent[irep] = jrep;
        }
    };
    
    // potentially use a more complex struct if you need to convey other info such as coordinates too
    std::vector<uint8_t> classify_frame(const camera_fb_t* frame);

    std::vector<CropView> candidate_crops(const camera_fb_t* frame);

    uint8_t classify_crop(const CropView candidate);

    float bee_activity_index(std::vector<uint8_t> classification_results);

    CropView make_crop(const camera_fb_t* frame, uint16_t x,  uint16_t y, uint16_t n);

}