/*
 * ESP32-S3-CAM — HTTP Web Server Test
 * Board  : ESP32-S3-WROOM-1 N16R8 with OV5640 (ribbon cable)
 * Library: esp32-camera by Espressif
 *
 * Endpoints:
 *   GET /          → Control UI  (HTML page)
 *   GET /stream    → MJPEG stream (open in browser or VLC)
 *   GET /capture   → Single JPEG snapshot download
 *   GET /status    → JSON camera + system stats
 *   GET /control?var=<name>&val=<n>  → Adjust camera settings
 *
 * After flashing, open Serial Monitor at 115200 to see the device IP,
 * then navigate to  http://<IP>/  in any browser on the same network.
 */

#include "esp_camera.h"
#include "esp_timer.h"
#include "img_converters.h"
#include "Arduino.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_wifi.h"

// ─── USER CONFIG ────────────────────────────────────────────────────────────
const char *WIFI_SSID     = "sglink20";
const char *WIFI_PASSWORD = "bekapcsolo";
// ────────────────────────────────────────────────────────────────────────────

// ─── Pin map: ESP32-S3-WROOM-1 N16R8 ────────────────────────────────────────
//  Kept identical to your original code — do NOT change unless your PCB differs
#define PWDN_GPIO_NUM    -1
#define RESET_GPIO_NUM   -1
#define XCLK_GPIO_NUM    15
#define SIOD_GPIO_NUM     4
#define SIOC_GPIO_NUM     5
#define Y9_GPIO_NUM      16
#define Y8_GPIO_NUM      17
#define Y7_GPIO_NUM      18
#define Y6_GPIO_NUM      12
#define Y5_GPIO_NUM      10
#define Y4_GPIO_NUM       8
#define Y3_GPIO_NUM       9
#define Y2_GPIO_NUM      11
#define VSYNC_GPIO_NUM    6
#define HREF_GPIO_NUM     7
#define PCLK_GPIO_NUM    13
// ────────────────────────────────────────────────────────────────────────────

// MJPEG multipart boundary
#define PART_BOUNDARY "gc0p4Jq0M2Yt08jU534c0p"
static const char *_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\nX-Timestamp: %ld.%06ld\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// ════════════════════════════════════════════════════════════════════════════
//  Camera init — identical logic to your original, just tidied up
// ════════════════════════════════════════════════════════════════════════════
bool initCamera()
{
    camera_config_t cfg;
    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;
    cfg.pin_d0       = Y2_GPIO_NUM;
    cfg.pin_d1       = Y3_GPIO_NUM;
    cfg.pin_d2       = Y4_GPIO_NUM;
    cfg.pin_d3       = Y5_GPIO_NUM;
    cfg.pin_d4       = Y6_GPIO_NUM;
    cfg.pin_d5       = Y7_GPIO_NUM;
    cfg.pin_d6       = Y8_GPIO_NUM;
    cfg.pin_d7       = Y9_GPIO_NUM;
    cfg.pin_xclk     = XCLK_GPIO_NUM;
    cfg.pin_pclk     = PCLK_GPIO_NUM;
    cfg.pin_vsync    = VSYNC_GPIO_NUM;
    cfg.pin_href     = HREF_GPIO_NUM;
    cfg.pin_sscb_sda = SIOD_GPIO_NUM;
    cfg.pin_sscb_scl = SIOC_GPIO_NUM;
    cfg.pin_pwdn     = PWDN_GPIO_NUM;
    cfg.pin_reset    = RESET_GPIO_NUM;
    cfg.xclk_freq_hz = 20000000;
    cfg.pixel_format = PIXFORMAT_JPEG;

    // OV5640 benefits greatly from PSRAM — use it if available
    if (psramFound()) {
        cfg.frame_size   = FRAMESIZE_VGA;   // 800x600 — good starting point
        cfg.jpeg_quality = 12;               // 0-63, lower = better quality
        cfg.fb_count     = 3;
        cfg.fb_location  = CAMERA_FB_IN_PSRAM;
        cfg.grab_mode    = CAMERA_GRAB_LATEST;
        Serial.println("[CAM] PSRAM found — SVGA, 2 frame buffers");
    } else {
        cfg.frame_size   = FRAMESIZE_QVGA;
        cfg.jpeg_quality = 20;
        cfg.fb_count     = 1;
        cfg.fb_location  = CAMERA_FB_IN_DRAM;
        cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
        Serial.println("[CAM] No PSRAM — QVGA fallback");
    }

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init failed: 0x%x\n", err);
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        Serial.println("[CAM] Sensor null — check pin map");
        return false;
    }

    // OV5640-specific tweaks for better image quality
    s->set_brightness(s, 0);      // -2 to 2
    s->set_contrast(s, 0);        // -2 to 2
    s->set_saturation(s, 0);      // -2 to 2
    s->set_sharpness(s, 0);       // -2 to 2
    s->set_whitebal(s, 1);        // auto white balance on
    s->set_awb_gain(s, 1);        // AWB gain on
    s->set_wb_mode(s, 0);         // 0=auto, 1=sunny, 2=cloudy, 3=office, 4=home
    s->set_exposure_ctrl(s, 1);   // auto exposure on
    s->set_aec2(s, 0);
    s->set_gain_ctrl(s, 1);       // auto gain on
    s->set_gainceiling(s, (gainceiling_t)0);
    s->set_bpc(s, 0);             // black pixel correction
    s->set_wpc(s, 1);             // white pixel correction
    s->set_raw_gma(s, 1);
    s->set_lenc(s, 1);            // lens correction
    s->set_hmirror(s, 0);
    s->set_vflip(s, 0);
    s->set_dcw(s, 1);
    s->set_colorbar(s, 0);        // set to 1 for a color bar test pattern

    Serial.printf("[CAM] OK — Sensor PID: 0x%04X\n", s->id.PID);
    return true;
}

// ════════════════════════════════════════════════════════════════════════════
//  WiFi — same robust approach as your original
// ════════════════════════════════════════════════════════════════════════════
void connectWiFi()
{   
    delay(1000);                    // let radio settle after reset
    WiFi.persistent(false);        // don't write credentials to flash every boot
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);         // clear any stale AP state from previous session
    delay(200);
    Serial.printf("[WIFI] Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    int attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WIFI] Connected — IP: %s\n", WiFi.localIP().toString().c_str());
    } else {
        Serial.println("[WIFI] Failed — restarting in 2s");
        delay(2000);
        ESP.restart();
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  Handler: /stream  — MJPEG live stream
// ════════════════════════════════════════════════════════════════════════════
static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb  = NULL;
    esp_err_t    res = ESP_OK;
    char         part_buf[128];
    uint32_t     last_frame = 0;

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    httpd_resp_set_hdr(req, "X-Framerate", "25");

    while (true)
    {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[STREAM] Capture failed");
            res = ESP_FAIL;
            break;
        }

        // If grab_mode is LATEST and we have 3 bufs, discard and re-grab
        // to ensure we always send the freshest frame
        if (fb->timestamp.tv_sec == 0) {   // stale buffer check
            esp_camera_fb_return(fb);
            fb = esp_camera_fb_get();
            if (!fb) { res = ESP_FAIL; break; }
        }

        size_t fb_len = fb->len;

        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
        if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

        struct timeval tv;
        gettimeofday(&tv, NULL);
        size_t hlen = snprintf(part_buf, sizeof(part_buf), _STREAM_PART,
                               fb_len, tv.tv_sec, tv.tv_usec);

        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb_len);
        esp_camera_fb_return(fb);          // return BEFORE checking res
        if (res != ESP_OK) break;

        // Yield to WiFi/lwIP stack — critical, without this TCP TX queue backs up
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    return res;
}
// ════════════════════════════════════════════════════════════════════════════
//  Handler: /capture  — single JPEG snapshot download
// ════════════════════════════════════════════════════════════════════════════
static esp_err_t capture_handler(httpd_req_t *req)
{
    camera_fb_t *fb = esp_camera_fb_get();
    if (!fb) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    httpd_resp_set_type(req, "image/jpeg");
    httpd_resp_set_hdr(req, "Content-Disposition",
                       "inline; filename=capture.jpg");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    esp_err_t res = httpd_resp_send(req, (const char *)fb->buf, fb->len);
    esp_camera_fb_return(fb);
    return res;
}

// ════════════════════════════════════════════════════════════════════════════
//  Handler: /status  — JSON system + sensor stats
// ════════════════════════════════════════════════════════════════════════════
static esp_err_t status_handler(httpd_req_t *req)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    char json[1024];
    snprintf(json, sizeof(json),
        "{"
        "\"framesize\":%d,"
        "\"quality\":%d,"
        "\"brightness\":%d,"
        "\"contrast\":%d,"
        "\"saturation\":%d,"
        "\"sharpness\":%d,"
        "\"hmirror\":%d,"
        "\"vflip\":%d,"
        "\"whitebal\":%d,"
        "\"awb_gain\":%d,"
        "\"wb_mode\":%d,"
        "\"exposure_ctrl\":%d,"
        "\"gain_ctrl\":%d,"
        "\"colorbar\":%d,"
        "\"rssi\":%d,"
        "\"free_heap\":%u,"
        "\"psram_free\":%u,"
        "\"uptime_s\":%lu"
        "}",
        s->status.framesize,
        s->status.quality,
        s->status.brightness,
        s->status.contrast,
        s->status.saturation,
        s->status.sharpness,
        s->status.hmirror,
        s->status.vflip,
        s->status.awb,
        s->status.awb_gain,
        s->status.wb_mode,
        s->status.aec,
        s->status.agc,
        s->status.colorbar,
        WiFi.RSSI(),
        esp_get_free_heap_size(),
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        millis() / 1000
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, json);
}

// ════════════════════════════════════════════════════════════════════════════
//  Handler: /control?var=<name>&val=<n>
//  Supported vars: framesize, quality, brightness, contrast, saturation,
//                  sharpness, hmirror, vflip, wb_mode, whitebal, awb_gain,
//                  exposure_ctrl, gain_ctrl, colorbar
// ════════════════════════════════════════════════════════════════════════════
static esp_err_t control_handler(httpd_req_t *req)
{
    char  var[32] = {0};
    char  val[8]  = {0};
    char  buf[64] = {0};

    // Parse query string
    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }
    if (httpd_query_key_value(buf, "var", var, sizeof(var)) != ESP_OK ||
        httpd_query_key_value(buf, "val", val, sizeof(val)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        httpd_resp_send_500(req);
        return ESP_FAIL;
    }

    int ival = atoi(val);
    int res  = -1;

    if      (!strcmp(var, "framesize"))     res = s->set_framesize(s, (framesize_t)ival);
    else if (!strcmp(var, "quality"))       res = s->set_quality(s, ival);
    else if (!strcmp(var, "brightness"))    res = s->set_brightness(s, ival);
    else if (!strcmp(var, "contrast"))      res = s->set_contrast(s, ival);
    else if (!strcmp(var, "saturation"))    res = s->set_saturation(s, ival);
    else if (!strcmp(var, "sharpness"))     res = s->set_sharpness(s, ival);
    else if (!strcmp(var, "hmirror"))       res = s->set_hmirror(s, ival);
    else if (!strcmp(var, "vflip"))         res = s->set_vflip(s, ival);
    else if (!strcmp(var, "whitebal"))      res = s->set_whitebal(s, ival);
    else if (!strcmp(var, "awb_gain"))      res = s->set_awb_gain(s, ival);
    else if (!strcmp(var, "wb_mode"))       res = s->set_wb_mode(s, ival);
    else if (!strcmp(var, "exposure_ctrl")) res = s->set_exposure_ctrl(s, ival);
    else if (!strcmp(var, "gain_ctrl"))     res = s->set_gain_ctrl(s, ival);
    else if (!strcmp(var, "colorbar"))      res = s->set_colorbar(s, ival);
    else {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    Serial.printf("[CTRL] %s = %d  (result: %d)\n", var, ival, res);

    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, res == 0 ? "OK" : "FAIL");
}

// ════════════════════════════════════════════════════════════════════════════
//  Handler: /  — control UI (self-contained HTML, no external deps)
// ════════════════════════════════════════════════════════════════════════════
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    // The page fetches the stream from port 81 and controls from port 80
    const char *html = R"rawhtml(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>ESP32-S3 Camera Test</title>
<style>
  body{font-family:monospace;background:#111;color:#eee;margin:0;padding:10px}
  h1{color:#0cf;margin:0 0 10px}
  #stream-wrap{background:#000;display:inline-block;border:2px solid #333}
  #stream{max-width:100%;display:block}
  .controls{display:flex;flex-wrap:wrap;gap:8px;margin-top:10px}
  .ctrl{background:#222;border:1px solid #444;border-radius:6px;padding:8px 12px;min-width:180px}
  .ctrl label{display:block;font-size:11px;color:#aaa;margin-bottom:4px}
  .ctrl select,.ctrl input{width:100%;background:#111;color:#eee;border:1px solid #555;
    border-radius:4px;padding:3px 6px;font-family:monospace}
  .btn{background:#0cf;color:#000;border:none;border-radius:4px;padding:6px 14px;
    cursor:pointer;font-weight:bold;font-family:monospace}
  .btn:hover{background:#09b}
  #status{margin-top:10px;font-size:11px;color:#888;white-space:pre-wrap}
  #fps-disp{color:#0cf;margin-left:12px}
</style>
</head>
<body>
<h1>&#128247; ESP32-S3 Camera Test</h1>

<div>
  <button class="btn" onclick="toggleStream()" id="stream-btn">&#9654; Start Stream</button>
  <button class="btn" onclick="snapshot()">&#128247; Snapshot</button>
  <button class="btn" onclick="refreshStatus()">&#128260; Refresh Status</button>
  <span id="fps-disp"></span>
</div>

<div id="stream-wrap" style="margin-top:8px">
  <img id="stream" src="" width="800" height="600" alt="Stream stopped">
</div>

<div class="controls">
  <div class="ctrl">
    <label>Resolution (framesize)</label>
    <select onchange="ctrl('framesize',this.value)">
      <option value="5">QVGA 320x240</option>
      <option value="6">CIF  400x296</option>
      <option value="7">HVGA 480x320</option>
      <option value="8" selected>VGA  640x480</option>
      <option value="9">SVGA 800x600</option>
      <option value="10">XGA 1024x768</option>
      <option value="11">HD  1280x720</option>
      <option value="12">SXGA 1280x1024</option>
      <option value="13">UXGA 1600x1200</option>
    </select>
  </div>
  <div class="ctrl">
    <label>JPEG Quality (lower = better)</label>
    <input type="range" min="4" max="63" value="10"
      oninput="this.nextElementSibling.textContent=this.value;ctrl('quality',this.value)">
    <span>10</span>
  </div>
  <div class="ctrl">
    <label>Brightness (-2 … 2)</label>
    <input type="range" min="-2" max="2" value="0"
      oninput="this.nextElementSibling.textContent=this.value;ctrl('brightness',this.value)">
    <span>0</span>
  </div>
  <div class="ctrl">
    <label>Contrast (-2 … 2)</label>
    <input type="range" min="-2" max="2" value="0"
      oninput="this.nextElementSibling.textContent=this.value;ctrl('contrast',this.value)">
    <span>0</span>
  </div>
  <div class="ctrl">
    <label>Saturation (-2 … 2)</label>
    <input type="range" min="-2" max="2" value="0"
      oninput="this.nextElementSibling.textContent=this.value;ctrl('saturation',this.value)">
    <span>0</span>
  </div>
  <div class="ctrl">
    <label>H-Mirror</label>
    <select onchange="ctrl('hmirror',this.value)">
      <option value="0" selected>Off</option>
      <option value="1">On</option>
    </select>
  </div>
  <div class="ctrl">
    <label>V-Flip</label>
    <select onchange="ctrl('vflip',this.value)">
      <option value="0" selected>Off</option>
      <option value="1">On</option>
    </select>
  </div>
  <div class="ctrl">
    <label>White Balance</label>
    <select onchange="ctrl('wb_mode',this.value)">
      <option value="0" selected>Auto</option>
      <option value="1">Sunny</option>
      <option value="2">Cloudy</option>
      <option value="3">Office</option>
      <option value="4">Home</option>
    </select>
  </div>
  <div class="ctrl">
    <label>Color Bar Test</label>
    <select onchange="ctrl('colorbar',this.value)">
      <option value="0" selected>Off (live)</option>
      <option value="1">On (test pattern)</option>
    </select>
  </div>
  <div class="ctrl">
    <label>Auto Exposure</label>
    <select onchange="ctrl('exposure_ctrl',this.value)">
      <option value="1" selected>On</option>
      <option value="0">Off</option>
    </select>
  </div>
  <div class="ctrl">
    <label>Auto Gain</label>
    <select onchange="ctrl('gain_ctrl',this.value)">
      <option value="1" selected>On</option>
      <option value="0">Off</option>
    </select>
  </div>
</div>

<pre id="status">Click "Refresh Status" to load camera info...</pre>

<script>
  const HOST = window.location.hostname;
  const STREAM_URL = `http://${HOST}:81/stream`;
  let streaming = false;
  let frameCount = 0;
  let lastFpsTime = Date.now();

  const streamImg = document.getElementById('stream');
  const fpsDisp   = document.getElementById('fps-disp');

  function toggleStream() {
    streaming = !streaming;
    const btn = document.getElementById('stream-btn');
    if (streaming) {
      streamImg.src = STREAM_URL;
      streamImg.onload = () => {};
      btn.textContent = '⏹ Stop Stream';
      trackFps();
    } else {
      streamImg.src = '';
      btn.textContent = '▶ Start Stream';
      fpsDisp.textContent = '';
    }
  }

  function trackFps() {
    if (!streaming) return;
    streamImg.addEventListener('load', function onLoad() {
      frameCount++;
      const now = Date.now();
      if (now - lastFpsTime >= 1000) {
        fpsDisp.textContent = `${frameCount} fps`;
        frameCount = 0;
        lastFpsTime = now;
      }
      if (!streaming) streamImg.removeEventListener('load', onLoad);
    });
  }

  function snapshot() {
    window.open(`http://${HOST}/capture`, '_blank');
  }

  async function ctrl(variable, value) {
    try {
      const r = await fetch(`http://${HOST}/control?var=${variable}&val=${value}`);
      const t = await r.text();
      console.log(`ctrl ${variable}=${value} → ${t}`);
    } catch (e) {
      console.error(e);
    }
  }

  async function refreshStatus() {
    try {
      const r = await fetch(`http://${HOST}/status`);
      const j = await r.json();
      document.getElementById('status').textContent = JSON.stringify(j, null, 2);
    } catch(e) {
      document.getElementById('status').textContent = 'Error: ' + e;
    }
  }
</script>
</body>
</html>
)rawhtml";

    return httpd_resp_sendstr(req, html);
}

// ════════════════════════════════════════════════════════════════════════════
//  Start HTTP servers
//   Port 80  → UI, control, capture, status
//   Port 81  → MJPEG stream (separate server to avoid blocking)
// ════════════════════════════════════════════════════════════════════════════
void startWebServer()
{
    // ── Stream server on port 81 ──────────────────────────────────────────
    httpd_config_t stream_cfg = HTTPD_DEFAULT_CONFIG();
    stream_cfg.server_port      = 81;
    stream_cfg.ctrl_port        = 32769;   // avoid clash with camera_httpd
    stream_cfg.max_uri_handlers = 2;
    // Increase stack and recv timeouts for the blocking stream loop
    stream_cfg.stack_size        = 12288;
    stream_cfg.send_wait_timeout = 10;     // fail fast on stalled clients
    stream_cfg.recv_wait_timeout = 5;
        
    if (httpd_start(&stream_httpd, &stream_cfg) == ESP_OK) {
        httpd_uri_t stream_uri = {
            .uri      = "/stream",
            .method   = HTTP_GET,
            .handler  = stream_handler,
            .user_ctx = NULL
        };
        httpd_register_uri_handler(stream_httpd, &stream_uri);
        Serial.println("[WEB] Stream server  → http://" +
                       WiFi.localIP().toString() + ":81/stream");
    }

    // ── Camera server on port 80 ──────────────────────────────────────────
    httpd_config_t cam_cfg = HTTPD_DEFAULT_CONFIG();
    cam_cfg.server_port      = 80;
    cam_cfg.ctrl_port        = 32768;
    cam_cfg.max_uri_handlers = 8;
    cam_cfg.stack_size       = 8192;

    if (httpd_start(&camera_httpd, &cam_cfg) == ESP_OK) {
        httpd_uri_t index_uri   = { "/",        HTTP_GET, index_handler,   NULL };
        httpd_uri_t capture_uri = { "/capture", HTTP_GET, capture_handler, NULL };
        httpd_uri_t status_uri  = { "/status",  HTTP_GET, status_handler,  NULL };
        httpd_uri_t control_uri = { "/control", HTTP_GET, control_handler, NULL };

        httpd_register_uri_handler(camera_httpd, &index_uri);
        httpd_register_uri_handler(camera_httpd, &capture_uri);
        httpd_register_uri_handler(camera_httpd, &status_uri);
        httpd_register_uri_handler(camera_httpd, &control_uri);

        Serial.println("[WEB] Camera server  → http://" +
                       WiFi.localIP().toString() + "/");
    }
}

// ════════════════════════════════════════════════════════════════════════════
//  setup
// ════════════════════════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.printf("[SYS] Chip: %s  Rev: %d\n",
                  ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("[SYS] Flash: %u MB  PSRAM: %s (%u bytes)\n",
                  ESP.getFlashChipSize() / (1 << 20),
                  psramFound() ? "YES" : "NO",
                  ESP.getPsramSize());
    Serial.printf("[SYS] Free heap: %u bytes\n", ESP.getFreeHeap());
    Serial.printf("[SYS] PSRAM size:  %u bytes\n", ESP.getPsramSize());
    Serial.printf("[SYS] PSRAM free:  %u bytes\n", heap_caps_get_free_size(MALLOC_CAP_SPIRAM));

    if (!initCamera()) {
        Serial.println("[SYS] Camera init failed — halting. Check wiring/pin map.");
        while (true) { delay(1000); }
    }

    connectWiFi();
    esp_wifi_set_ps(WIFI_PS_NONE);          // no modem sleep — needed for smooth streaming
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

   

    startWebServer();

    Serial.println();
    Serial.println("══════════════════════════════════════");
    Serial.println("  Open in browser: http://" + WiFi.localIP().toString() + "/");
    Serial.println("  MJPEG direct:    http://" + WiFi.localIP().toString() + ":81/stream");
    Serial.println("  Snapshot:        http://" + WiFi.localIP().toString() + "/capture");
    Serial.println("  Status JSON:     http://" + WiFi.localIP().toString() + "/status");
    Serial.println("══════════════════════════════════════");
}

// ════════════════════════════════════════════════════════════════════════════
//  loop — nothing to do, httpd runs on its own FreeRTOS tasks
// ════════════════════════════════════════════════════════════════════════════
void loop()
{
    // WiFi watchdog
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WIFI] Lost connection, reconnecting...");
        connectWiFi();
    }
    delay(5000);
}