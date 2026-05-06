/**
 * @file main.cpp
 * @brief Example usage of camera_module for the ESP-CAM AI Thinker.
 *
 * This file demonstrates the camera module API and will later be replaced
 * (or extended) by the full NixiSat flight software.
 *
 * To integrate into your main flight code:
 *   - Copy camera_module.h + camera_module.cpp into src/
 *   - #include "camera_module.h" in your main file
 *   - Call cam_init() during setup, cam_capture() in your main loop or task
 */

#include <Arduino.h>
#include "camera_module.h"
#include "esp_camera.h"

// ---------------------------------------------------------------------------
// Example: Image processing callback
//
// This is called by cam_capture() for EVERY frame before the SD write.
// Swap in your real analysis (edge detection, centroid, target lock, etc.)
// ---------------------------------------------------------------------------
CamResult on_frame_received(const uint8_t* buf, size_t len, framesize_t resolution) {
    // -----------------------------------------------------------------------
    // TODO: Replace with real image processing.
    //
    // The buffer contains a JPEG-encoded frame. On the AI Thinker the OV2640
    // does JPEG compression in hardware, so you get compressed data here.
    //
    // Options:
    //   a) Transmit raw JPEG to ground station and process there (easiest).
    //   b) Decode JPEG on-chip and run lightweight analysis (needs PSRAM).
    //   c) Switch pixel format to PIXFORMAT_RGB565/GRAYSCALE in cam_init()
    //      for raw pixel access — costs more DMA memory but enables on-chip
    //      processing without a JPEG decode step.
    // -----------------------------------------------------------------------

    Serial.printf("[PROCESS] Frame received — %u bytes, res=%d\n",
                  (unsigned)len, (int)resolution);

    // Example: flag if frame is suspiciously small (likely a bad capture)
    if (len < 2000) {
        Serial.println("[PROCESS] WARNING: frame too small — possible sensor glitch");
        return CamResult::ERR_CAPTURE;
    }

    return CamResult::OK;
}

// ---------------------------------------------------------------------------
// Setup
// ---------------------------------------------------------------------------
void setup() {
    Serial.begin(115200);
    delay(500);  // let the serial monitor attach
    Serial.println("\n=== ESP-CAM Module Test ===");

    // Optional: configure before init
    // (resolution and quality can also be changed after init via cam_set_*)

    CamResult res = cam_init();
    if (res != CamResult::OK) {
        Serial.printf("cam_init() failed with code %d\n", (int)res);
        // In flight software: set a FAULT flag and enter safe mode
        // For now, halt and blink to signal error
        while (true) {
            cam_flash(true);  delay(200);
            cam_flash(false); delay(200);
        }
    }

    // Register the processing callback (remove if not needed)
    cam_set_process_cb(on_frame_received);

    // Optional tuning
    cam_set_resolution(FRAMESIZE_VGA);  // 640×480 — good for ground detection
    cam_set_quality(12);

    cam_print_status();
    Serial.println("Setup complete — starting capture loop\n");
}

// ---------------------------------------------------------------------------
// Loop
// ---------------------------------------------------------------------------
void loop() {
    static uint32_t last_capture_ms = 0;
    const uint32_t CAPTURE_INTERVAL_MS = 1000;  // 1 frame/sec — adjust as needed

    uint32_t now = millis();
    if (now - last_capture_ms >= CAPTURE_INTERVAL_MS) {
        last_capture_ms = now;

        // Optional: flash ON briefly for better image (indoor / low-light only)
        // cam_flash(true);
        // delayMicroseconds(100);  // let sensor AEC adjust

        CamResult res = cam_capture(/*save_to_sd=*/true);

        // cam_flash(false);

        if (res != CamResult::OK) {
            Serial.printf("[LOOP] cam_capture() error: %d\n", (int)res);
        }

        // Print status every 10 frames
        CamStatus st = cam_get_status();
        if (st.frames_captured % 10 == 0) {
            cam_print_status();
        }
    }

    // Other flight tasks go here (IMU, PID, comms, etc.)
}