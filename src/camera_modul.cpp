/**
 * @file camera_module.cpp
 * @brief ESP-CAM AI Thinker (OV2640) camera + SD card — implementation.
 *
 * Uses the official Espressif esp32-camera driver (esp_camera.h).
 */

#include "camera_module.h"

#include <SPI.h>
#include <SD.h>

// ---------------------------------------------------------------------------
// Module-private state
// ---------------------------------------------------------------------------
namespace {

struct CamState {
    bool         camera_ready     = false;
    bool         sd_ready         = false;
    framesize_t  framesize        = CAM_DEFAULT_FRAMESIZE;
    int          quality          = CAM_DEFAULT_QUALITY;
    uint32_t     frames_captured  = 0;
    uint32_t     frames_saved     = 0;
    uint32_t     frames_processed = 0;
    size_t       last_frame_bytes = 0;
    CamProcessCb process_cb       = nullptr;
    uint32_t     file_index       = 0;
    SPIClass*    sd_spi           = nullptr;
};

CamState g_cam;

// Populate an esp_camera config from current module state
camera_config_t build_camera_config() {
    camera_config_t cfg = {};

    cfg.pin_pwdn     = CAM_PIN_PWDN;
    cfg.pin_reset    = CAM_PIN_RESET;
    cfg.pin_xclk     = CAM_PIN_XCLK;
    cfg.pin_sscb_sda = CAM_PIN_SIOD;
    cfg.pin_sscb_scl = CAM_PIN_SIOC;
    cfg.pin_d7       = CAM_PIN_Y9;
    cfg.pin_d6       = CAM_PIN_Y8;
    cfg.pin_d5       = CAM_PIN_Y7;
    cfg.pin_d4       = CAM_PIN_Y6;
    cfg.pin_d3       = CAM_PIN_Y5;
    cfg.pin_d2       = CAM_PIN_Y4;
    cfg.pin_d1       = CAM_PIN_Y3;
    cfg.pin_d0       = CAM_PIN_Y2;
    cfg.pin_vsync    = CAM_PIN_VSYNC;
    cfg.pin_href     = CAM_PIN_HREF;
    cfg.pin_pclk     = CAM_PIN_PCLK;

    cfg.xclk_freq_hz = CAM_XCLK_FREQ_HZ;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.ledc_channel = LEDC_CHANNEL_0;

    cfg.pixel_format = PIXFORMAT_JPEG;  // OV2640 compresses in hardware
    cfg.frame_size   = g_cam.framesize;
    cfg.jpeg_quality = g_cam.quality;

    if (psramFound()) {
        cfg.fb_count    = 2;                    // double-buffer: always return freshest frame
        cfg.grab_mode   = CAMERA_GRAB_LATEST;
        cfg.fb_location = CAMERA_FB_IN_PSRAM;
    } else {
        cfg.fb_count    = 1;
        cfg.grab_mode   = CAMERA_GRAB_WHEN_EMPTY;
        cfg.fb_location = CAMERA_FB_IN_DRAM;

        // Protect against OOM — DRAM cannot hold large JPEG frame buffers
        if (cfg.frame_size > FRAMESIZE_CIF) {
            Serial.println("[CAM] No PSRAM — capping resolution to CIF (352x288) to avoid OOM");
            cfg.frame_size   = FRAMESIZE_CIF;
            g_cam.framesize  = FRAMESIZE_CIF;
        }
    }
    return cfg;
}

// Write buf to SD as /img<N>.jpg — returns true on a complete write
bool sd_write_frame(const uint8_t* buf, size_t len) {
    char filename[32];
    int n = snprintf(filename, sizeof(filename), "%s%05lu.jpg",
                     CAM_FILENAME_PREFIX, (unsigned long)g_cam.file_index);
    if (n <= 0 || (size_t)n >= sizeof(filename)) {
        Serial.println("[CAM] ERROR: filename buffer overflow");
        return false;
    }

    File f = SD.open(filename, FILE_WRITE);
    if (!f) {
        Serial.printf("[CAM] ERROR: cannot open %s for writing\n", filename);
        return false;
    }

    size_t written = f.write(buf, len);
    f.close();

    if (written != len) {
        Serial.printf("[CAM] ERROR: partial write — %u/%u bytes in %s\n",
                      (unsigned)written, (unsigned)len, filename);
        return false;
    }

    Serial.printf("[CAM] Saved %s (%u bytes)\n", filename, (unsigned)len);
    g_cam.file_index++;
    return true;
}

} // anonymous namespace

// ---------------------------------------------------------------------------
// Public API — init / deinit
// ---------------------------------------------------------------------------

CamResult cam_init() {
    // Flash LED off by default
    pinMode(SD_PIN_FLASH_LED, OUTPUT);
    cam_flash(false);

    // PWDN pin: active HIGH on AI Thinker (HIGH = sensor powered down)
    if (CAM_PIN_PWDN >= 0) {
        pinMode(CAM_PIN_PWDN, OUTPUT);
        digitalWrite(CAM_PIN_PWDN, LOW);  // power sensor ON
        delay(10);
    }

    // ---- Camera sensor ----
    camera_config_t cfg = build_camera_config();
    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] ERROR: esp_camera_init() failed: 0x%x\n", err);
        return CamResult::ERR_CAMERA_INIT;
    }

    // Initial sensor tuning — adjust to your lighting conditions
    sensor_t* s = esp_camera_sensor_get();
    if (s) {
        s->set_vflip(s, 0);
        s->set_hmirror(s, 0);
        s->set_brightness(s, 0);      // -2 … +2
        s->set_contrast(s, 0);
        s->set_saturation(s, 0);
        s->set_whitebal(s, 1);        // auto white balance on
        s->set_awb_gain(s, 1);
        s->set_exposure_ctrl(s, 1);   // auto exposure on
        s->set_aec2(s, 0);
        s->set_ae_level(s, 0);
        s->set_gain_ctrl(s, 1);       // auto gain on
        s->set_gainceiling(s, (gainceiling_t)6);
    }

    g_cam.camera_ready = true;
    Serial.printf("[CAM] Camera OK — framesize=%d  quality=%d  PSRAM=%s\n",
                  (int)g_cam.framesize, g_cam.quality,
                  psramFound() ? "yes" : "no");

    // ---- SD card on HSPI ----
    // HSPI avoids conflicts with the camera's I2C lines that share VSPI pins.
    g_cam.sd_spi = new SPIClass(HSPI);
    g_cam.sd_spi->begin(SD_PIN_SCK, SD_PIN_MISO, SD_PIN_MOSI, SD_PIN_CS);

    if (!SD.begin(SD_PIN_CS, *g_cam.sd_spi, CAM_SD_SPI_FREQ_HZ)) {
        Serial.println("[CAM] WARNING: SD card not found — saves disabled");
        // Camera still works in callback-only / transmit-only mode
        return CamResult::ERR_SD_INIT;
    }

    uint64_t total_mb = SD.totalBytes() / (1024ULL * 1024ULL);
    uint64_t free_mb  = (SD.totalBytes() - SD.usedBytes()) / (1024ULL * 1024ULL);
    Serial.printf("[CAM] SD OK — %llu MB total, %llu MB free\n", total_mb, free_mb);

    g_cam.sd_ready = true;
    return CamResult::OK;
}

void cam_deinit() {
    if (g_cam.camera_ready) {
        esp_camera_deinit();
        g_cam.camera_ready = false;
    }
    if (g_cam.sd_ready) {
        SD.end();
        g_cam.sd_ready = false;
    }
    if (g_cam.sd_spi) {
        g_cam.sd_spi->end();
        delete g_cam.sd_spi;
        g_cam.sd_spi = nullptr;
    }

    // Power down sensor
    if (CAM_PIN_PWDN >= 0) {
        digitalWrite(CAM_PIN_PWDN, HIGH);
    }
    cam_flash(false);
    Serial.println("[CAM] De-initialised");
}

// ---------------------------------------------------------------------------
// Public API — configuration
// ---------------------------------------------------------------------------

CamResult cam_set_framesize(framesize_t framesize) {
    if (!g_cam.camera_ready) return CamResult::ERR_NOT_INIT;

    sensor_t* s = esp_camera_sensor_get();
    if (!s) return CamResult::ERR_CAMERA_INIT;

    s->set_framesize(s, framesize);
    g_cam.framesize = framesize;
    Serial.printf("[CAM] Framesize set to %d\n", (int)framesize);
    return CamResult::OK;
}

CamResult cam_set_quality(int quality) {
    if (!g_cam.camera_ready) return CamResult::ERR_NOT_INIT;

    sensor_t* s = esp_camera_sensor_get();
    if (!s) return CamResult::ERR_CAMERA_INIT;

    quality = constrain(quality, 0, 63);
    s->set_quality(s, quality);
    g_cam.quality = quality;
    Serial.printf("[CAM] Quality set to %d\n", quality);
    return CamResult::OK;
}

void cam_set_process_cb(CamProcessCb cb) {
    g_cam.process_cb = cb;
    Serial.printf("[CAM] Processing callback %s\n", cb ? "registered" : "cleared");
}

// ---------------------------------------------------------------------------
// Public API — capture
// ---------------------------------------------------------------------------

CamResult cam_capture(bool save_to_sd) {
    if (!g_cam.camera_ready) return CamResult::ERR_NOT_INIT;

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[CAM] ERROR: esp_camera_fb_get() returned null");
        return CamResult::ERR_CAPTURE;
    }

    g_cam.frames_captured++;
    g_cam.last_frame_bytes = fb->len;
    CamResult result = CamResult::OK;

    // --- Processing callback ---
    if (g_cam.process_cb) {
        CamResult cb_res = g_cam.process_cb(fb);
        if (cb_res == CamResult::OK) {
            g_cam.frames_processed++;
        } else {
            Serial.printf("[CAM] WARNING: callback returned error %d\n", (int)cb_res);
            result = cb_res;  // record, but continue to SD step
        }
    }

    // --- Save to SD ---
    if (save_to_sd) {
        if (!g_cam.sd_ready) {
            Serial.println("[CAM] WARNING: SD not ready — frame not saved");
            result = CamResult::ERR_SD_WRITE;
        } else if (sd_write_frame(fb->buf, fb->len)) {
            g_cam.frames_saved++;
        } else {
            result = CamResult::ERR_SD_WRITE;
        }
    }

    esp_camera_fb_return(fb);  // always return the DMA buffer
    return result;
}

CamResult cam_capture_to_buffer(uint8_t* out_buf, size_t buf_size, size_t& out_len) {
    out_len = 0;
    if (!out_buf)            return CamResult::ERR_BUFFER_SMALL;
    if (!g_cam.camera_ready) return CamResult::ERR_NOT_INIT;

    camera_fb_t* fb = esp_camera_fb_get();
    if (!fb) {
        Serial.println("[CAM] ERROR: esp_camera_fb_get() returned null (to_buffer)");
        return CamResult::ERR_CAPTURE;
    }

    g_cam.frames_captured++;
    g_cam.last_frame_bytes = fb->len;

    if (fb->len > buf_size) {
        Serial.printf("[CAM] ERROR: frame %u B > buffer %u B\n",
                      (unsigned)fb->len, (unsigned)buf_size);
        esp_camera_fb_return(fb);
        return CamResult::ERR_BUFFER_SMALL;
    }

    memcpy(out_buf, fb->buf, fb->len);
    out_len = fb->len;

    esp_camera_fb_return(fb);
    return CamResult::OK;
}

// ---------------------------------------------------------------------------
// Public API — misc
// ---------------------------------------------------------------------------

void cam_flash(bool on) {
    digitalWrite(SD_PIN_FLASH_LED, on ? HIGH : LOW);
}

CamStatus cam_get_status() {
    return {
        g_cam.camera_ready,
        g_cam.sd_ready,
        g_cam.framesize,
        g_cam.quality,
        g_cam.frames_captured,
        g_cam.frames_saved,
        g_cam.frames_processed,
        g_cam.last_frame_bytes,
    };
}

void cam_print_status() {
    CamStatus st = cam_get_status();
    Serial.println("=== Camera Module Status ===");
    Serial.printf("  Camera    : %s\n", st.camera_ready ? "READY"     : "NOT INIT");
    Serial.printf("  SD Card   : %s\n", st.sd_ready     ? "READY"     : "NOT READY");
    Serial.printf("  Framesize : %d\n", (int)st.framesize);
    Serial.printf("  Quality   : %d\n", st.quality);
    Serial.printf("  Captured  : %lu\n", (unsigned long)st.frames_captured);
    Serial.printf("  Saved     : %lu\n", (unsigned long)st.frames_saved);
    Serial.printf("  Processed : %lu\n", (unsigned long)st.frames_processed);
    Serial.printf("  Last frame: %u bytes\n", (unsigned)st.last_frame_bytes);
    Serial.println("============================");
}