#include "cam.hpp"

namespace cam {

// Camera Module pin mapping
static camera_config_t camera_config = {
    .pin_pwdn = PWDN_GPIO_NUM,
    .pin_reset = RESET_GPIO_NUM,
    .pin_xclk = XCLK_GPIO_NUM,
    .pin_sccb_sda = SIOD_GPIO_NUM,
    .pin_sccb_scl = SIOC_GPIO_NUM,
    .pin_d7 = Y9_GPIO_NUM,
    .pin_d6 = Y8_GPIO_NUM,
    .pin_d5 = Y7_GPIO_NUM,
    .pin_d4 = Y6_GPIO_NUM,
    .pin_d3 = Y5_GPIO_NUM,
    .pin_d2 = Y4_GPIO_NUM,
    .pin_d1 = Y3_GPIO_NUM,
    .pin_d0 = Y2_GPIO_NUM,
    .pin_vsync = VSYNC_GPIO_NUM,
    .pin_href = HREF_GPIO_NUM,
    .pin_pclk = PCLK_GPIO_NUM,

    .xclk_freq_hz = 20000000,          // The clock frequency of the image sensor
    .pixel_format = PIXFORMAT_GRAYSCALE,    // The pixel format of the image: PIXFORMAT_ + YUV422|GRAYSCALE|RGB565|JPEG
    .frame_size = FRAMESIZE_QVGA,      // The resolution size of the image: FRAMESIZE_ + QVGA|CIF|VGA|SVGA|XGA|SXGA|UXGA
    .jpeg_quality = 5,                // The quality of the JPEG image, ranging from 0 to 63.
    .fb_count = 4,                     // The number of frame buffers to use.
    .fb_location = CAMERA_FB_IN_PSRAM, // Set the frame buffer storage location
    .grab_mode = CAMERA_GRAB_LATEST    //  The image capture mode.
};

esp_err_t init_camera(void)
{
    // Initialize the camera
    esp_err_t err = esp_camera_init(&camera_config);
    if (err != ESP_OK) {
        ESP_LOGE("CAM", "Camera Init Failed");
    }
    // camera settings
    sensor_t * s = esp_camera_sensor_get();
    s->set_ae_level(s, -1);      // Slightly underexpose (-2 to 2) to force a faster shutter

    return err;
}

camera_fb_t* combine_grayscale_to_rgb565(const camera_fb_t *r,
                                         const camera_fb_t *g,
                                         const camera_fb_t *b)
{
    if (!r || !g || !b) {
        ESP_LOGE("CAM", "Null input buffer");
        return NULL;
    }
    if (r->width != g->width || r->width != b->width ||
        r->height != g->height || r->height != b->height) {
        ESP_LOGE("CAM", "Dimension mismatch");
        return NULL;
    }
    if (r->format != PIXFORMAT_GRAYSCALE ||
        g->format != PIXFORMAT_GRAYSCALE ||
        b->format != PIXFORMAT_GRAYSCALE) {
        ESP_LOGE("CAM", "Inputs must be GRAYSCALE");
        return NULL;
    }

    size_t pixels = static_cast<size_t>(r->width) * r->height;
    size_t rgb565_len = pixels * 2;

    // --- Allocate output fb ---
    camera_fb_t *out = (camera_fb_t*)heap_caps_malloc(sizeof(camera_fb_t),
                                                      MALLOC_CAP_DEFAULT);
    if (!out) return NULL;

    // Prefer PSRAM for the large pixel buffer
    out->buf = (uint8_t*)heap_caps_malloc(rgb565_len, MALLOC_CAP_SPIRAM);
    if (!out->buf) {
        out->buf = (uint8_t*)heap_caps_malloc(rgb565_len, MALLOC_CAP_DEFAULT);
    }
    if (!out->buf) {
        free(out);
        ESP_LOGE("CAM", "Failed to allocate %u bytes", static_cast<unsigned>(rgb565_len));
        return NULL;
    }

    out->len    = rgb565_len;
    out->width  = r->width;
    out->height = r->height;
    out->format = PIXFORMAT_RGB565;
    out->timestamp = r->timestamp;

    // Pack each pixel as RGB565, stored low byte first to match CropView::pixel().
    const uint8_t *rp = r->buf;
    const uint8_t *gp = g->buf;
    const uint8_t *bp = b->buf;
    uint8_t *dst = out->buf;

    for (size_t i = 0; i < pixels; i++) {
        const uint16_t px =
            (static_cast<uint16_t>(rp[i] & 0xF8) << 8) |
            (static_cast<uint16_t>(gp[i] & 0xFC) << 3) |
            (static_cast<uint16_t>(bp[i]) >> 3);
        *dst++ = static_cast<uint8_t>(px & 0xFF);
        *dst++ = static_cast<uint8_t>(px >> 8);
    }

    return out;
}

void free_fb(camera_fb_t *fb) {
    if (fb) {
        if (fb->buf) free(fb->buf);
        free(fb);
    }
}

camera_fb_t* capture_rgb_sequence()
{
    // capture 3 consecutive frames and combine them from the frame buffer into a single image
    camera_fb_t *red = esp_camera_fb_get();
    camera_fb_t *green = esp_camera_fb_get();
    camera_fb_t *blue = esp_camera_fb_get();

    // combine into rgb channels of base pic
    camera_fb_t *combined = combine_grayscale_to_rgb565(red, green, blue);
    if (!combined) {
        ESP_LOGE("APP", "Failed to combine RGB channels");
        esp_camera_fb_return(red);
        esp_camera_fb_return(green);
        esp_camera_fb_return(blue);
        return NULL;
    }

    //return the frame buffers to the camera driver so they can be reused
    esp_camera_fb_return(red);
    esp_camera_fb_return(green);
    esp_camera_fb_return(blue);

    return combined;
}

}