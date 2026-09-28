#include <stddef.h>
#include "esp_heap_caps.h"
#include <cstdint>
#include "sensor.h"
#include <stdio.h>
#include <algorithm>
#include "esp_log.h"
#include <esp_system.h>
#include <string.h>
#include <vector>
#include "bsp/esp-bsp.h"
#include "freertos/FreeRTOS.h"
#include <esp_system.h>
#include "freertos/task.h"
#include "include/camera_pins.h"

#include "ds18b20.h"
#include "ds18b20_types.h"
#include "onewire_bus.h"
#include "temp_pins.h"
#include "temp_sensors.hpp"
#include "sd_card.hpp"
#include "bee_vision.hpp"
#include "classification_category_name.hpp"

extern "C" void app_main(void)
{
    //example image to test the model
    extern const uint8_t example_crop[] asm("_binary_example_rgb565_start");

    ESP_LOGI("SD", "Mounting SD card...");
    gpio_set_direction(GPIO_NUM_43, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_43, 1);
    bool mounted = sdcard::init();
    if (!mounted) {
        ESP_LOGE("SD", "SD card init/mount failed");
        //return;
    }

    int device_nums = temp_sensors::init();

    //ESP_LOGI("MEM", "Capturing base frame for stacking...");

    ESP_LOGI("MEM", "Begin Main loop...");
    while (true) {
        ESP_LOGI("MEM", "Free heap at start of loop: %lu bytes", esp_get_free_heap_size());

        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGI("CNN", "initializing bee model...");
        if (!bee_vision::model_initialized()) { //this check is necessary to prevent memory from filling up
            if (!bee_vision::initialize_bee_model()) {
                ESP_LOGE("CNN", "Failed to initialize bee model");
                continue;
            }
        } else {
            ESP_LOGI("CNN", "Model already exists");
        }
        
        ESP_LOGI("CNN", "Model initialized");
        ESP_LOGI("CNN", "Model test run");
        bee_vision::CropView crop { 
            example_crop,
            0,0,
            32 * 2,
            32,
            32,32,
            2 
        };
        uint8_t cls = bee_vision::classify_crop(crop);
        ESP_LOGI("CNN", "test crop classified as: %s", classification_cat_names[cls]);

        // print sensors found
        std::vector<float> v = temp_sensors::read_temperatures(device_nums);
        for (auto i : v){
            ESP_LOGI("TEMP", "Measured: %.2f", i);
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
