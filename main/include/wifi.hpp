#pragma once

#include <string>
#include "esp_err.h"

namespace wifi {

struct HttpResponse {
    int status_code = 0;
    std::string body;
};

esp_err_t connect();

esp_err_t disconnect();

bool is_connected();

esp_err_t http_post(const std::string &url,
                    const std::string &body,
                    HttpResponse &response,
                    const std::string &content_type = "application/json",
                    int timeout_ms = 5000);

esp_err_t http_get(const std::string &url,
                   HttpResponse &response,
                   int timeout_ms = 5000);

}