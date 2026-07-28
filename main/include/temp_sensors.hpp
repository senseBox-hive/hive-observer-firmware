#pragma once
#include <vector>

namespace temp_sensors {

    int init();

    float read_temperature(int device_id);

    std::vector<float> read_temperatures(int device_num);
}
