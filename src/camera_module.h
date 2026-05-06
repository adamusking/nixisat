/**
 * @file camera_module.h
 * @brief ESP-CAM AI Thinker (OV2640) camera + SD card module.
 *
 * Built on the official Espressif esp32-camera library (esp_camera.h).
 *
 * platformio.ini dependency to add:
 *   lib_deps = espressif/esp32-camera @ ^2.0.0
 *
 * Usage:
 *   1. Call cam_init() once in setup().
 *   2. Optionally call cam_set_process_cb() to register a per-frame handler.
 *   3. Call cam_capture() to grab a frame (saves to SD and/or fires callback).
 *   4. Call cam_deinit() before deep-sleep or shutdown.
 */

#pragma once

#include <Arduino.h>
#include "esp_camera.h"

// ---------------------------------------------------------------------------
// AI Thinker pin map
// ---------------------------------------------------------------------------
#define CAM_PIN_PWDN     32
#define CAM_PIN_RESET    -1   // tied to 3V3 on this module
#define CAM_PIN_XCLK      0
#define CAM_PIN_SIOD     26   // SDA
#define CAM_PIN_SIOC     27   // SCL
#define CAM_PIN_Y9       35
#define CAM_PIN_Y8       34
#define CAM_PIN_Y7       39
#define CAM_PIN_Y6       36
#define CAM_PIN_Y5       21
#define CAM_PIN_Y4       19
#define CAM_PIN_Y3       18
#define CAM_PIN_Y2        5
#define CAM_PIN_VSYNC    25
#define CAM_PIN_HREF     23
#define CAM_PIN_PCLK     22

// SD card — HSPI bus
// GPIO2  = MISO — also the onboard status LED and a strapping pin.
//          Add a 10kΩ pull-up to 3V3 so the SD card cannot hold it LOW at boot
//          and accidentally trigger serial-download mode.
// GPIO4  = flash LED (active HIGH). Keep LOW during SD writes — on some AI
//          Thinker revisions the LED shares a circuit with the SD card power.
#define SD_PIN_CS        13
#define SD_PIN_MOSI      15
#define SD_PIN_MISO       2
#define SD_PIN_SCK       14
#define SD_PIN_FLASH_LED  4   // active HIGH

// ---------------------------------------------------------------------------
// Compile-time defaults (override in platformio.ini or before this #include)
// ---------------------------------------------------------------------------
#ifndef CAM_DEFAULT_FRAMESIZE
#define CAM_DEFAULT_FRAMESIZE  FRAMESIZE_VGA   // 640×480
#endif

#ifndef CAM_DEFAULT_QUALITY
#define CAM_DEFAULT_QUALITY    12   // 0 = best, 63 = worst; 10-15 is practical
#endif

#ifndef CAM_XCLK_FREQ_HZ
#define CAM_XCLK_FREQ_HZ       20000000
#endif

#ifndef CAM_SD_SPI_FREQ_HZ
#define CAM_SD_SPI_FREQ_HZ     4000000  // conservative; raise if transfers are stable
#endif

#ifndef CAM_FILENAME_PREFIX
#define CAM_FILENAME_PREFIX    "/img"   // files written as /img00001.jpg, etc.
#endif

// ---------------------------------------------------------------------------
// Result codes
// ---------------------------------------------------------------------------
enum class CamResult : uint8_t {
    OK               = 0,
    ERR_CAMERA_INIT  = 1,
    ERR_SD_INIT      = 2,
    ERR_CAPTURE      = 3,
    ERR_SD_WRITE     = 4,
    ERR_BUFFER_SMALL = 5,
    ERR_NOT_INIT     = 6,
};

// ---------------------------------------------------------------------------
// Processing callback
//
// Register via cam_set_process_cb(). Called on every captured frame before
// the SD write step. The camera_fb_t* is valid only during the callback —
// do NOT store the pointer; esp_camera_fb_return() is called automatically.
//
// Example:
//   CamResult my_handler(const camera_fb_t* fb) {
//       // fb->buf  — JPEG byte buffer
//       // fb->len  — number of bytes
//       // fb->width, fb->height — pixel dimensions
//       // fb->format — should be PIXFORMAT_JPEG
//       radio_send(fb->buf, fb->len);
//       return CamResult::OK;
//   }
//
// Return CamResult::OK on success. Any other value is logged but will NOT
// abort the SD save step.
// ---------------------------------------------------------------------------
using CamProcessCb = CamResult (*)(const camera_fb_t* fb);

// ---------------------------------------------------------------------------
// Read-only status snapshot
// ---------------------------------------------------------------------------
struct CamStatus {
    bool        camera_ready;
    bool        sd_ready;
    framesize_t framesize;
    int         quality;
    uint32_t    frames_captured;
    uint32_t    frames_saved;
    uint32_t    frames_processed;
    size_t      last_frame_bytes;
};

// ---------------------------------------------------------------------------
// Public API
// ---------------------------------------------------------------------------

/**
 * @brief Initialise the OV2640 sensor and SD card.
 *
 * Automatically uses PSRAM-backed double frame buffers when PSRAM is available,
 * otherwise falls back to a single DRAM buffer with a reduced resolution.
 *
 * @return CamResult::OK           — camera and SD both ready.
 *         CamResult::ERR_CAMERA_INIT — sensor failed (check seating/power).
 *         CamResult::ERR_SD_INIT    — SD absent/unformatted; camera still
 *                                     works in callback-only/transmit-only mode.
 */
CamResult cam_init();

/** @brief Release all resources. Safe to call after a partial init failure. */
void cam_deinit();

/**
 * @brief Change the capture resolution at runtime.
 * Valid values: FRAMESIZE_96X96 … FRAMESIZE_UXGA (OV2640 max = 1600×1200).
 */
CamResult cam_set_framesize(framesize_t framesize);

/**
 * @brief Set JPEG compression quality (0 = best quality, 63 = smallest file).
 * Practical range: 10–15.
 */
CamResult cam_set_quality(int quality);

/** @brief Register a per-frame callback, or pass nullptr to clear it. */
void cam_set_process_cb(CamProcessCb cb);

/**
 * @brief Capture one JPEG frame.
 *
 * Execution order:
 *   1. esp_camera_fb_get() — grab DMA frame from sensor
 *   2. Invoke processing callback (if registered)
 *   3. Write JPEG to SD as /img<N>.jpg  (if save_to_sd == true)
 *   4. esp_camera_fb_return() — always, even on error
 *
 * Not safe to call concurrently from multiple tasks without an external mutex.
 *
 * @param save_to_sd  false = callback/transmit-only; skips SD write entirely.
 */
CamResult cam_capture(bool save_to_sd = true);

/**
 * @brief Capture and copy the JPEG into a caller-managed buffer.
 *
 * Use when you need to hold the image data after this call returns,
 * e.g. to enqueue it for a radio transmission task.
 *
 * @param out_buf   Caller-allocated destination buffer.
 * @param buf_size  Capacity of out_buf in bytes.
 * @param out_len   Actual JPEG byte count written (0 on any failure).
 */
CamResult cam_capture_to_buffer(uint8_t* out_buf, size_t buf_size, size_t& out_len);

/** @brief Drive the onboard flash LED (GPIO4, active HIGH). */
void cam_flash(bool on);

/** @brief Return a snapshot of the current module state. */
CamStatus cam_get_status();

/** @brief Print a labelled status block to Serial (useful for bench testing). */
void cam_print_status();    