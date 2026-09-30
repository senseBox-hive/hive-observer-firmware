#include <stddef.h>
#include <string.h>
#include <vector>
#include "sensor.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"  // IWYU pragma: keep
#include "freertos/task.h"
#include "esp_timer.h"

#include "temp_pins.h"
#include "temp_sensors.hpp"
#include "sd_card.hpp"
#include "bee_vision.hpp"
#include "classification_category_name.hpp"
#include "cam.hpp"
#include "wifi.hpp"

static const char POST_URL[] = "http://example:8080/";

std::string create_json(
    uint32_t timestamp, 
    std::vector<uint8_t> frame_inferences,
    std::vector<float> temperatures
){
    std::string output = "{\"timestamp\":";
    //timestamp
    output.append(std::to_string(timestamp));
    output.append(",");

    //inferences
    output.append("\"inferences\":[");
    for (int i = 0; i<frame_inferences.size(); i++){
        std::string inference = "";
        if(i>0){
            inference.append(",");
        }
        inference.append("\"");
        inference.append(classification_cat_names[frame_inferences[i]]);
        inference.append("\"");
        output.append(inference);
    }
    output.append("],");

    //temperature
    output.append("\"temperatures\":[");
    for (int i = 0; i<temperatures.size(); i++){
        std::string temp = "";
        if(i>0){
            temp.append(",");
        }
        temp.append(std::to_string(temperatures[i]));
        output.append(temp);
    }
    output.append("]");

    output.append("}");
    return output;
}

extern "C" void app_main(void)
{
    //example images to test the model
    extern const uint8_t example_crop[] asm("_binary_example_rgb565_start");
    extern const uint8_t example_frame[] asm("_binary_example_frame_rgb565_start");
    camera_fb_t example_frame_fb = {};
    example_frame_fb.buf = const_cast<uint8_t*>(example_frame);
    example_frame_fb.len = 320 * 240 * 2;
    example_frame_fb.width = 320;
    example_frame_fb.height = 240;
    example_frame_fb.format = PIXFORMAT_RGB565;

    //initialize components
    ESP_LOGI("SD", "Mounting SD card...");
    gpio_set_direction(GPIO_NUM_43, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_43, 1);
    bool mounted = sdcard::init();
    if (!mounted) {
        ESP_LOGE("SD", "SD card init/mount failed");
        //return;
    }

    if (ESP_OK != cam::init_camera()) {
        ESP_LOGE("APP", "Camera initialization failed");
        return;
    }
    ESP_LOGI("CNN", "initializing bee model...");
    if (!bee_vision::model_initialized()) { //this check is necessary to prevent memory from filling up
        if (!bee_vision::initialize_bee_model()) {
            ESP_LOGE("CNN", "Failed to initialize bee model");
            return;
        }
    } else {
        ESP_LOGI("CNN", "Model already exists");
    }
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
    std::vector<uint8_t> testframe_inferences = bee_vision::classify_frame(&example_frame_fb);
    for (auto i : testframe_inferences){
        ESP_LOGI("CNN", "testframe crop classified as: %s", classification_cat_names[i]);
    }
    ESP_LOGI("CNN", "Model test run finished");

    int device_nums = temp_sensors::init();

    ESP_LOGI("NET", "Initializing network connection");
    if (ESP_OK != wifi::connect()) {
        ESP_LOGE("NET", "Network interface initialization failed. continuing offline");
    }

    ESP_LOGI("MEM", "Begin Main loop...");
    while (true) {
        ESP_LOGI("MEM", "Free heap at start of loop: %lu bytes", esp_get_free_heap_size());

        //capture frame
        uint32_t timestamp = esp_timer_get_time() / 1000;
        camera_fb_t *rgb_sequence = cam::capture_rgb_sequence();
        if (!rgb_sequence) {
            ESP_LOGE("APP", "Failed to capture RGB sequence");
            continue;
        }
        //run inference on frame
        std::vector<uint8_t> frame_inferences = bee_vision::classify_frame(rgb_sequence);

        // fetch sensor outputs
        std::vector<float> temperatures = temp_sensors::read_temperatures(device_nums);

        //build json
        std::string measurement_results = create_json(
            timestamp,
            frame_inferences,
            temperatures
        );

        //send json/save json
        wifi::HttpResponse resp;
        esp_err_t err = wifi::http_post(
            POST_URL,
            measurement_results,
            resp
        );
        if (err == ESP_OK) {
            ESP_LOGI("NET", "Status: %d", resp.status_code);
        }

        cam::free_fb(rgb_sequence);

        vTaskDelay(pdMS_TO_TICKS(5));
    }

}
