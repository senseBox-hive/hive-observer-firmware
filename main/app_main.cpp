#include <stddef.h>
#include "esp_heap_caps.h"
#include <cstdint>
#include "sensor.h"
#include <stdio.h>
#include <algorithm>
#include "esp_log.h"
#include "sd_card.hpp"
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
#include "onewire_types.h"

#define EXAMPLE_ONEWIRE_BUS_GPIO GPIO_NUM_2 // IO2 or IO14
#define EXAMPLE_ONEWIRE_MAX_DS18B20 10

extern "C" void app_main(void)
{
    ESP_LOGI("SD", "Mounting SD card...");
    gpio_set_direction(GPIO_NUM_43, GPIO_MODE_OUTPUT);
    gpio_set_level(GPIO_NUM_43, 1);
    bool mounted = sdcard::init();
    if (!mounted) {
        ESP_LOGE("SD", "SD card init/mount failed");
        //return;
    }

    // install 1-wire bus
    onewire_bus_handle_t bus = NULL;
    onewire_bus_config_t bus_config = {
        .bus_gpio_num = EXAMPLE_ONEWIRE_BUS_GPIO,
        .flags = {
            .en_pull_up = true, // enable the internal pull-up resistor in case the external device didn't have one
        }
    };
    onewire_bus_rmt_config_t rmt_config = {
        .max_rx_bytes = 10, // 1byte ROM command + 8byte ROM number + 1byte device command
    };
    ESP_ERROR_CHECK(onewire_new_bus_rmt(&bus_config, &rmt_config, &bus));

    int ds18b20_device_num = 0;
    ds18b20_device_handle_t ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];
    onewire_device_iter_handle_t iter = NULL;
    onewire_device_t next_onewire_device;
    esp_err_t search_result = ESP_OK;

    // create 1-wire device iterator, which is used for device search
    ESP_ERROR_CHECK(onewire_new_device_iter(bus, &iter));
    ESP_LOGI("MEM", "Device iterator created, start searching...");
    do {
        search_result = onewire_device_iter_get_next(iter, &next_onewire_device);
        if (search_result == ESP_OK) { // found a new device, let's check if we can upgrade it to a DS18B20
            ds18b20_config_t ds_cfg = {};
            onewire_device_address_t address;
            // check if the device is a DS18B20, if so, return the ds18b20 handle
            if (ds18b20_new_device_from_enumeration(&next_onewire_device, &ds_cfg, &ds18b20s[ds18b20_device_num]) == ESP_OK) {
                ds18b20_get_device_address(ds18b20s[ds18b20_device_num], &address);
                ESP_LOGI("MEM", "Found a DS18B20[%d], address: %016llX", ds18b20_device_num, address);
                ds18b20_device_num++;
            } else {
                ESP_LOGI("MEM", "Found an unknown device, address: %016llX", next_onewire_device.address);
            }
        }
    } while (search_result != ESP_ERR_NOT_FOUND);
    ESP_ERROR_CHECK(onewire_del_device_iter(iter));
    ESP_LOGI("MEM", "Searching done, %d DS18B20 device(s) found", ds18b20_device_num);

    // Now you have the DS18B20 sensor handle, you can use it to read the temperature


    //ESP_LOGI("MEM", "Capturing base frame for stacking...");

    ESP_LOGI("MEM", "Begin Main loop...");
    while (true) {
        ESP_LOGI("MEM", "Free heap at start of loop: %lu bytes", esp_get_free_heap_size());
        
        // print sensors found
        for (int i = 0; i < ds18b20_device_num; i++) {
            ESP_LOGI("MEM", "DS18B20[%d]", i);
            float temperature;
            //convert, then get. otherwise you end with constant 85.0 reading
            ds18b20_trigger_temperature_conversion(ds18b20s[i]);
            if (ds18b20_get_temperature(ds18b20s[i], &temperature) == ESP_OK) {
                ESP_LOGI("MEM", "DS18B20[%d] Measured: %.2f", i, temperature);
            } else {
                ESP_LOGE("MEM", "Failed to read temperature from DS18B20[%d]", i);
            }
        }

        vTaskDelay(pdMS_TO_TICKS(5));
    }
}
