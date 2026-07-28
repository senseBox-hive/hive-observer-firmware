#include "temp_sensors.hpp"

#include <soc/gpio_struct.h>
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_err.h"

//#include "temp_pins.h"
#include "ds18b20.h"
#include "ds18b20_types.h"
#include "onewire_bus.h"
#include "onewire_types.h"

#include <vector>

#define EXAMPLE_ONEWIRE_BUS_GPIO GPIO_NUM_2 // IO2 or IO14
#define EXAMPLE_ONEWIRE_MAX_DS18B20 10

namespace temp_sensors {

ds18b20_device_handle_t ds18b20s[EXAMPLE_ONEWIRE_MAX_DS18B20];

//init function
int init(){
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
    
    // set device resolution
    for (int i = 0; i < ds18b20_device_num; i++) {
        ds18b20_set_resolution(ds18b20s[i], DS18B20_RESOLUTION_11B);
    }
    
    return ds18b20_device_num;
}

float read_temperature(int device_id){
    float temperature;
    //convert, then get. otherwise you end with constant 85.0 reading
    ds18b20_trigger_temperature_conversion(ds18b20s[device_id]);
    if (ds18b20_get_temperature(ds18b20s[device_id], &temperature) == ESP_OK) {
        return temperature;
    } else {
        ESP_LOGE("MEM", "Failed to read temperature from DS18B20[%d]", device_id);
    }
    return .0f; //TODO:fix
}

std::vector<float> read_temperatures(int ds18b20_device_num){
    // create vector
    std::vector<float> v;
    for (int i = 0; i < ds18b20_device_num; i++) {
        v.push_back(read_temperature(i));
    }
    return v;
}

}