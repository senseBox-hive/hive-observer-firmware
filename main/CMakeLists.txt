set(src_dirs        ./
                    ${CMAKE_CURRENT_SOURCE_DIR}/src)

set(include_dirs    ./
                    ${CMAKE_CURRENT_SOURCE_DIR}/include
                    ${CMAKE_CURRENT_SOURCE_DIR}/src)

if (IDF_TARGET STREQUAL "esp32s3")
    list(APPEND requires esp32_s3_eye_noglib
                         esp_lcd
                         esp_wifi)
elseif (IDF_TARGET STREQUAL "esp32p4")
    list(APPEND requires esp32_p4_function_ev_board_noglib
                         esp_lcd
                         esp_wifi)
endif()


file(GLOB embed_imgs "${CMAKE_CURRENT_SOURCE_DIR}/data/*")
file(GLOB embed_models "${CMAKE_CURRENT_SOURCE_DIR}/models/*")

set(embed_files ${embed_imgs} ${embed_models})

idf_component_register(
    SRC_DIRS ${src_dirs}
    INCLUDE_DIRS ${include_dirs}
    REQUIRES ${requires}
    PRIV_REQUIRES fatfs esp_http_client nvs_flash esp_netif esp_event
    EMBED_FILES ${embed_files}
)