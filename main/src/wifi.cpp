#include "wifi.hpp"

#include "sdkconfig.h"
#include "esp_log.h"
#include "esp_event.h"
#include "esp_netif.h"
#include "esp_http_client.h"
#include "nvs_flash.h"
#include "protocol_examples_common.h"

namespace wifi 
{

namespace {

bool s_connected = false;

// Appends incoming response data to the std::string passed via user_data
esp_err_t http_event_handler(esp_http_client_event_t *evt)
{
    auto *body = static_cast<std::string *>(evt->user_data);

    switch (evt->event_id) {
    case HTTP_EVENT_ERROR:
        ESP_LOGE("NET", "HTTP_EVENT_ERROR");
        break;
    case HTTP_EVENT_ON_CONNECTED:
        ESP_LOGD("NET", "HTTP connected");
        break;
    case HTTP_EVENT_ON_DATA:
        if (body && evt->data_len > 0) {
            body->append(static_cast<const char *>(evt->data), evt->data_len);
        }
        break;
    case HTTP_EVENT_ON_FINISH:
        ESP_LOGD("NET", "HTTP finished");
        break;
    case HTTP_EVENT_DISCONNECTED:
        ESP_LOGD("NET", "HTTP disconnected");
        break;
    default:
        break;
    }
    return ESP_OK;
}

esp_err_t perform_request(const std::string &url,
                          esp_http_client_method_t method,
                          const std::string *body,
                          const std::string &content_type,
                          HttpResponse &response,
                          int timeout_ms)
{
    if (!s_connected) {
        ESP_LOGE("NET", "Not connected, call wifi::connect() first");
        return ESP_ERR_INVALID_STATE;
    }

    response.status_code = 0;
    response.body.clear();

    // Zero-initialize, then assign (avoids C++ designated-initializer quirks)
    esp_http_client_config_t config = {};
    config.url = url.c_str();
    config.method = method;
    config.event_handler = http_event_handler;
    config.user_data = &response.body;
    config.timeout_ms = timeout_ms;

    esp_http_client_handle_t client = esp_http_client_init(&config);
    if (client == nullptr) {
        ESP_LOGE("NET", "Failed to init HTTP client");
        return ESP_FAIL;
    }

    if (body != nullptr) {
        esp_http_client_set_header(client, "Content-Type", content_type.c_str());
        esp_http_client_set_post_field(client, body->c_str(),
                                       static_cast<int>(body->size()));
    }

    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        response.status_code = esp_http_client_get_status_code(client);
        ESP_LOGI("NET", "%s %s -> %d (%u bytes)",
                 method == HTTP_METHOD_POST ? "POST" : "GET",
                 url.c_str(), response.status_code,
                 static_cast<unsigned>(response.body.size()));
    } else {
        ESP_LOGE("NET", "Request to %s failed: %s", url.c_str(), esp_err_to_name(err));
    }

    esp_http_client_cleanup(client);
    return err;
}

}

esp_err_t connect()
{
    if (s_connected) {
        return ESP_OK;
    }

    // NVS (required by the Wi-Fi driver)
    esp_err_t err = nvs_flash_init();
    if (err == ESP_ERR_NVS_NO_FREE_PAGES || err == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        err = nvs_flash_init();
    }
    if (err != ESP_OK) {
        ESP_LOGE("NET", "nvs_flash_init failed: %s", esp_err_to_name(err));
        return err;
    }

    err = esp_netif_init();
    if (err != ESP_OK) {
        ESP_LOGE("NET", "esp_netif_init failed: %s", esp_err_to_name(err));
        return err;
    }

    // Tolerate the event loop already existing (e.g. created elsewhere)
    err = esp_event_loop_create_default();
    if (err != ESP_OK && err != ESP_ERR_INVALID_STATE) {
        ESP_LOGE("NET", "esp_event_loop_create_default failed: %s", esp_err_to_name(err));
        return err;
    }

    // Blocks until connected and an IP address is obtained
    err = example_connect();
    if (err != ESP_OK) {
        ESP_LOGE("NET", "example_connect failed: %s", esp_err_to_name(err));
        return err;
    }

    s_connected = true;
    ESP_LOGI("NET", "Connected");
    return ESP_OK;
}

esp_err_t disconnect()
{
    if (!s_connected) {
        return ESP_OK;
    }
    esp_err_t err = example_disconnect();
    if (err == ESP_OK) {
        s_connected = false;
    }
    return err;
}

bool is_connected()
{
    return s_connected;
}

esp_err_t http_post(const std::string &url,
                    const std::string &body,
                    HttpResponse &response,
                    const std::string &content_type,
                    int timeout_ms)
{
    return perform_request(url, HTTP_METHOD_POST, &body, content_type, response, timeout_ms);
}

esp_err_t http_get(const std::string &url, HttpResponse &response, int timeout_ms)
{
    return perform_request(url, HTTP_METHOD_GET, nullptr, "", response, timeout_ms);
}

}