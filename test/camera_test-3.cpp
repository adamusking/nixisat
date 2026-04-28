/*
 * NixiSat — OV5640 Camera Test
 * Board  : ESP32-S3-WROOM-1 N16R8
 * Camera : OV5640 via ribbon cable
 *
 * Endpoints:
 *   GET /          → Control UI + live stream
 *   GET /stream    → Raw MJPEG (port 81, for VLC / browser img tag)
 *   GET /capture   → Single JPEG snapshot
 *   GET /status    → JSON system + sensor stats
 *   GET /control?var=<name>&val=<n>  → Adjust camera settings at runtime
 *
 * Open Serial Monitor at 115200 to get the IP, then:
 *   http://<IP>/  in any Chromium-based browser (Chrome, Edge, Brave).
 *   Firefox has dropped MJPEG-in-img support — use Chrome for testing.
 */

#include "esp_camera.h"
#include "esp_timer.h"
#include "Arduino.h"
#include <WiFi.h>
#include "esp_http_server.h"
#include "esp_wifi.h"
#include "driver/ledc.h"

// ─── USER CONFIG ─────────────────────────────────────────────────────────────
const char *WIFI_SSID     = "sglink20";
const char *WIFI_PASSWORD = "bekapcsolo";
// ─────────────────────────────────────────────────────────────────────────────

// ─── Pin map ──────────────────────────────────────────────────────────────────
// These match the ESP32-S3-EYE / common ESP32-S3 camera devkit with OV5640.
// If init fails (0x105 error), swap SIOD/SIOC or check your board's schematic.
#define PWDN_GPIO_NUM   -1
#define RESET_GPIO_NUM  -1
#define XCLK_GPIO_NUM   15
#define SIOD_GPIO_NUM    4   // SDA — also called SIO_D
#define SIOC_GPIO_NUM    5   // SCL — also called SIO_C
#define Y9_GPIO_NUM     16
#define Y8_GPIO_NUM     17
#define Y7_GPIO_NUM     18
#define Y6_GPIO_NUM     12
#define Y5_GPIO_NUM     10
#define Y4_GPIO_NUM      8
#define Y3_GPIO_NUM      9
#define Y2_GPIO_NUM     11
#define VSYNC_GPIO_NUM   6
#define HREF_GPIO_NUM    7
#define PCLK_GPIO_NUM   13
// ─────────────────────────────────────────────────────────────────────────────

#define PART_BOUNDARY "gc0p4Jq0M2Yt08jU534c0p"
static const char *_STREAM_CONTENT_TYPE =
    "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char *_STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char *_STREAM_PART     =
    "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

httpd_handle_t stream_httpd = NULL;
httpd_handle_t camera_httpd = NULL;

// ═════════════════════════════════════════════════════════════════════════════
//  Camera init
// ═════════════════════════════════════════════════════════════════════════════
bool initCamera()
{
    camera_config_t cfg = {};

    cfg.ledc_channel = LEDC_CHANNEL_0;
    cfg.ledc_timer   = LEDC_TIMER_0;

    // Data bus — Y2 (LSB) … Y9 (MSB)
    cfg.pin_d0 = Y2_GPIO_NUM;
    cfg.pin_d1 = Y3_GPIO_NUM;
    cfg.pin_d2 = Y4_GPIO_NUM;
    cfg.pin_d3 = Y5_GPIO_NUM;
    cfg.pin_d4 = Y6_GPIO_NUM;
    cfg.pin_d5 = Y7_GPIO_NUM;
    cfg.pin_d6 = Y8_GPIO_NUM;
    cfg.pin_d7 = Y9_GPIO_NUM;

    cfg.pin_xclk  = XCLK_GPIO_NUM;
    cfg.pin_pclk  = PCLK_GPIO_NUM;
    cfg.pin_vsync = VSYNC_GPIO_NUM;
    cfg.pin_href  = HREF_GPIO_NUM;

    // FIX: esp32-camera ≥2.x renamed these from pin_sscb_* to pin_sccb_*
    // Using the old names silently maps to wrong registers → sensor never
    // receives config writes → stuck at default (very low) frame rate.
    cfg.pin_sccb_sda = SIOD_GPIO_NUM;
    cfg.pin_sccb_scl = SIOC_GPIO_NUM;

    cfg.pin_pwdn  = PWDN_GPIO_NUM;
    cfg.pin_reset = RESET_GPIO_NUM;

    cfg.xclk_freq_hz = 20000000;   // 20 MHz — safe for OV5640
    cfg.pixel_format = PIXFORMAT_JPEG;

    if (psramFound()) {
        // Start at SVGA — reliable streaming, can bump up via /control
        cfg.frame_size  = FRAMESIZE_SVGA;   // 800 × 600
        cfg.jpeg_quality = 10;              // 0-63, lower = better
        cfg.fb_count    = 2;               // 2 bufs with GRAB_LATEST is optimal
        cfg.fb_location = CAMERA_FB_IN_PSRAM;
        cfg.grab_mode   = CAMERA_GRAB_LATEST;
        Serial.println("[CAM] PSRAM found — SVGA, 2 frame buffers, GRAB_LATEST");
    } else {
        // Fallback: no PSRAM — stay small
        cfg.frame_size   = FRAMESIZE_QVGA; // 320 × 240
        cfg.jpeg_quality = 20;
        cfg.fb_count     = 1;
        cfg.fb_location  = CAMERA_FB_IN_DRAM;
        cfg.grab_mode    = CAMERA_GRAB_WHEN_EMPTY;
        Serial.println("[CAM] No PSRAM — QVGA fallback");
    }

    esp_err_t err = esp_camera_init(&cfg);
    if (err != ESP_OK) {
        Serial.printf("[CAM] Init FAILED: 0x%x\n", err);
        Serial.println("[CAM]  0x105 = timeout talking to sensor (check SDA/SCL pins)");
        Serial.println("[CAM]  0x103 = camera not found (check power / XCLK)");
        return false;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (!s) {
        Serial.println("[CAM] esp_camera_sensor_get() returned NULL");
        return false;
    }

    Serial.printf("[CAM] OK — Sensor PID: 0x%04X  (OV5640 = 0x5640)\n", s->id.PID);

    // OV5640 defaults — auto everything, no test pattern
    s->set_brightness(s,    0);   // -2..2
    s->set_contrast(s,      0);   // -2..2
    s->set_saturation(s,    0);   // -2..2
    s->set_sharpness(s,     0);   // -2..2  (not all drivers expose this)
    s->set_whitebal(s,      1);   // AWB on
    s->set_awb_gain(s,      1);   // AWB gain on
    s->set_wb_mode(s,       0);   // 0=auto
    s->set_exposure_ctrl(s, 1);   // AEC on
    s->set_aec2(s,          0);
    s->set_gain_ctrl(s,     1);   // AGC on
    s->set_bpc(s,           0);   // black-pixel correction
    s->set_wpc(s,           1);   // white-pixel correction
    s->set_raw_gma(s,       1);
    s->set_lenc(s,          1);   // lens shading correction
    s->set_hmirror(s,       0);
    s->set_vflip(s,         0);
    s->set_dcw(s,           1);
    s->set_colorbar(s,      0);   // set to 1 for a solid color-bar test pattern

    return true;
}

// ═════════════════════════════════════════════════════════════════════════════
//  WiFi
// ═════════════════════════════════════════════════════════════════════════════
void connectWiFi()
{
    WiFi.persistent(false);
    WiFi.mode(WIFI_STA);
    WiFi.disconnect(true);
    delay(100);

    Serial.printf("[WiFi] Connecting to %s", WIFI_SSID);
    WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

    uint8_t attempts = 0;
    while (WiFi.status() != WL_CONNECTED && attempts < 40) {
        delay(500);
        Serial.print(".");
        attempts++;
    }
    Serial.println();

    if (WiFi.status() == WL_CONNECTED) {
        Serial.printf("[WiFi] Connected — IP: %s  RSSI: %d dBm\n",
                      WiFi.localIP().toString().c_str(), WiFi.RSSI());
    } else {
        Serial.println("[WiFi] Failed — rebooting in 3 s");
        delay(3000);
        ESP.restart();
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  /stream  — MJPEG live stream (port 81)
// ═════════════════════════════════════════════════════════════════════════════
static esp_err_t stream_handler(httpd_req_t *req)
{
    camera_fb_t *fb  = NULL;
    esp_err_t    res = ESP_OK;
    char         part_buf[64];

    res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
    if (res != ESP_OK) return res;
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

    while (true)
    {
        fb = esp_camera_fb_get();
        if (!fb) {
            Serial.println("[STREAM] Frame capture failed — retrying");
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;   // don't abort the stream on a single dropped frame
        }

        // Send multipart boundary
        res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY,
                                   strlen(_STREAM_BOUNDARY));
        if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

        // Send part header with content-length
        size_t hlen = snprintf(part_buf, sizeof(part_buf),
                               _STREAM_PART, fb->len);
        res = httpd_resp_send_chunk(req, part_buf, hlen);
        if (res != ESP_OK) { esp_camera_fb_return(fb); break; }

        // Send JPEG payload
        res = httpd_resp_send_chunk(req, (const char *)fb->buf, fb->len);
        esp_camera_fb_return(fb);   // always return before breaking
        if (res != ESP_OK) break;

        // Yield to WiFi / lwIP stack — do not remove; without this the TCP TX
        // queue backs up and the stream freezes after a few frames
        vTaskDelay(pdMS_TO_TICKS(1));
    }

    Serial.println("[STREAM] Client disconnected");
    return res;
}

// ═════════════════════════════════════════════════════════════════════════════
//  /capture  — single JPEG snapshot
// ═════════════════════════════════════════════════════════════════════════════
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

// ═════════════════════════════════════════════════════════════════════════════
//  /status  — JSON stats
// ═════════════════════════════════════════════════════════════════════════════
static esp_err_t status_handler(httpd_req_t *req)
{
    sensor_t *s = esp_camera_sensor_get();
    if (!s) { httpd_resp_send_500(req); return ESP_FAIL; }

    char json[768];
    snprintf(json, sizeof(json),
        "{"
        "\"sensor_pid\":\"0x%04X\","
        "\"framesize\":%d,"
        "\"quality\":%d,"
        "\"brightness\":%d,"
        "\"contrast\":%d,"
        "\"saturation\":%d,"
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
        s->id.PID,
        s->status.framesize, s->status.quality,
        s->status.brightness, s->status.contrast, s->status.saturation,
        s->status.hmirror, s->status.vflip,
        s->status.awb, s->status.awb_gain, s->status.wb_mode,
        s->status.aec, s->status.agc, s->status.colorbar,
        WiFi.RSSI(),
        esp_get_free_heap_size(),
        heap_caps_get_free_size(MALLOC_CAP_SPIRAM),
        millis() / 1000UL
    );

    httpd_resp_set_type(req, "application/json");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, json);
}

// ═════════════════════════════════════════════════════════════════════════════
//  /control?var=<name>&val=<n>
// ═════════════════════════════════════════════════════════════════════════════
static esp_err_t control_handler(httpd_req_t *req)
{
    char var[32] = {}, val[8] = {}, buf[64] = {};

    if (httpd_req_get_url_query_str(req, buf, sizeof(buf)) != ESP_OK ||
        httpd_query_key_value(buf, "var", var, sizeof(var)) != ESP_OK ||
        httpd_query_key_value(buf, "val", val, sizeof(val)) != ESP_OK) {
        httpd_resp_send_404(req);
        return ESP_FAIL;
    }

    sensor_t *s = esp_camera_sensor_get();
    if (!s) { httpd_resp_send_500(req); return ESP_FAIL; }

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
    else { httpd_resp_send_404(req); return ESP_FAIL; }

    Serial.printf("[CTRL] %s = %d → %s\n", var, ival, res == 0 ? "OK" : "FAIL");
    httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
    return httpd_resp_sendstr(req, res == 0 ? "OK" : "FAIL");
}

// ═════════════════════════════════════════════════════════════════════════════
//  /  — Control UI (self-contained HTML, no CDN deps)
//  Note: MJPEG in <img> works in Chrome/Edge/Brave. Firefox dropped it.
// ═════════════════════════════════════════════════════════════════════════════
static esp_err_t index_handler(httpd_req_t *req)
{
    httpd_resp_set_type(req, "text/html");
    const char *html = R"rawhtml(
<!DOCTYPE html><html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<title>NixiSat Camera Test</title>
<style>
  body{font-family:monospace;background:#0d0d0d;color:#e0e0e0;margin:0;padding:12px}
  h1{color:#00ccff;margin:0 0 10px;font-size:1.2em}
  #stream{max-width:100%;display:block;border:2px solid #333;background:#000}
  .bar{display:flex;flex-wrap:wrap;gap:8px;margin:8px 0}
  .btn{background:#00ccff;color:#000;border:none;border-radius:4px;
    padding:6px 14px;cursor:pointer;font-weight:bold;font-family:monospace}
  .btn:hover{background:#009bbf}
  .controls{display:flex;flex-wrap:wrap;gap:8px;margin-top:8px}
  .ctrl{background:#1a1a1a;border:1px solid #333;border-radius:6px;padding:8px 12px;min-width:180px}
  .ctrl label{display:block;font-size:10px;color:#888;margin-bottom:4px}
  .ctrl select,.ctrl input[type=range]{width:100%;background:#0d0d0d;color:#e0e0e0;
    border:1px solid #444;border-radius:3px;padding:2px 4px;font-family:monospace}
  #fps{color:#00ccff;margin-left:10px;font-size:0.9em}
  #status{margin-top:10px;font-size:10px;color:#666;white-space:pre-wrap;
    background:#111;border:1px solid #222;padding:8px;border-radius:4px}
  .note{color:#f90;font-size:10px;margin-top:4px}
</style>
</head>
<body>
<h1>&#128247; NixiSat — OV5640 Camera Test</h1>
<p class="note">&#9888; Use Chrome/Edge/Brave. Firefox does not support MJPEG streams.</p>
<div class="bar">
  <button class="btn" id="stream-btn" onclick="toggleStream()">&#9654; Start Stream</button>
  <button class="btn" onclick="snapshot()">&#128247; Snapshot</button>
  <button class="btn" onclick="refreshStatus()">&#128260; Status</button>
  <span id="fps"></span>
</div>

<img id="stream" src="" width="800" height="600" alt="Stream stopped — click Start Stream">

<div class="controls">
  <div class="ctrl">
    <label>Resolution</label>
    <select onchange="ctrl('framesize',this.value)">
      <option value="5">QVGA 320×240</option>
      <option value="6">CIF  400×296</option>
      <option value="7">HVGA 480×320</option>
      <option value="8">VGA  640×480</option>
      <option value="9" selected>SVGA 800×600</option>
      <option value="10">XGA 1024×768</option>
      <option value="11">HD  1280×720</option>
    </select>
  </div>
  <div class="ctrl">
    <label>JPEG Quality (lower = better image)</label>
    <input type="range" min="4" max="63" value="10" id="q-slider"
      oninput="document.getElementById('q-val').textContent=this.value;ctrl('quality',this.value)">
    <span id="q-val">10</span>
  </div>
  <div class="ctrl">
    <label>Brightness  −2 … 2</label>
    <input type="range" min="-2" max="2" value="0"
      oninput="this.nextElementSibling.textContent=this.value;ctrl('brightness',this.value)">
    <span>0</span>
  </div>
  <div class="ctrl">
    <label>Contrast  −2 … 2</label>
    <input type="range" min="-2" max="2" value="0"
      oninput="this.nextElementSibling.textContent=this.value;ctrl('contrast',this.value)">
    <span>0</span>
  </div>
  <div class="ctrl">
    <label>Saturation  −2 … 2</label>
    <input type="range" min="-2" max="2" value="0"
      oninput="this.nextElementSibling.textContent=this.value;ctrl('saturation',this.value)">
    <span>0</span>
  </div>
  <div class="ctrl">
    <label>H-Mirror</label>
    <select onchange="ctrl('hmirror',this.value)">
      <option value="0" selected>Off</option><option value="1">On</option>
    </select>
  </div>
  <div class="ctrl">
    <label>V-Flip</label>
    <select onchange="ctrl('vflip',this.value)">
      <option value="0" selected>Off</option><option value="1">On</option>
    </select>
  </div>
  <div class="ctrl">
    <label>White Balance</label>
    <select onchange="ctrl('wb_mode',this.value)">
      <option value="0" selected>Auto</option>
      <option value="1">Sunny</option><option value="2">Cloudy</option>
      <option value="3">Office</option><option value="4">Home</option>
    </select>
  </div>
  <div class="ctrl">
    <label>Auto Exposure</label>
    <select onchange="ctrl('exposure_ctrl',this.value)">
      <option value="1" selected>On</option><option value="0">Off</option>
    </select>
  </div>
  <div class="ctrl">
    <label>Auto Gain</label>
    <select onchange="ctrl('gain_ctrl',this.value)">
      <option value="1" selected>On</option><option value="0">Off</option>
    </select>
  </div>
  <div class="ctrl">
    <label>Color Bar (test pattern)</label>
    <select onchange="ctrl('colorbar',this.value)">
      <option value="0" selected>Off (live)</option><option value="1">On</option>
    </select>
  </div>
</div>

<pre id="status">Click "Status" to load sensor info...</pre>

<script>
const HOST       = window.location.hostname;
const STREAM_URL = `http://${HOST}:81/stream`;
let streaming    = false;
let fcount = 0, lastFpsT = Date.now();
const img    = document.getElementById('stream');
const fpsEl  = document.getElementById('fps');
const btn    = document.getElementById('stream-btn');

function toggleStream() {
  streaming = !streaming;
  if (streaming) {
    img.src = STREAM_URL;
    img.addEventListener('load', onFrame);
    btn.textContent = '⏹ Stop Stream';
  } else {
    img.src = '';
    img.removeEventListener('load', onFrame);
    btn.textContent = '▶ Start Stream';
    fpsEl.textContent = '';
  }
}

function onFrame() {
  fcount++;
  const now = Date.now();
  if (now - lastFpsT >= 1000) {
    fpsEl.textContent = `${fcount} fps`;
    fcount = 0; lastFpsT = now;
  }
}

function snapshot() { window.open(`http://${HOST}/capture`, '_blank'); }

async function ctrl(v, val) {
  try { await fetch(`http://${HOST}/control?var=${v}&val=${val}`); }
  catch(e) { console.error(e); }
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
</body></html>
)rawhtml";
    return httpd_resp_sendstr(req, html);
}

// ═════════════════════════════════════════════════════════════════════════════
//  Start HTTP servers
//  Port 80  → UI, control, capture, status
//  Port 81  → MJPEG stream (separate server to avoid blocking the UI)
// ═════════════════════════════════════════════════════════════════════════════
void startWebServers()
{
    // ── Stream server port 81 ─────────────────────────────────────────────
    httpd_config_t scfg = HTTPD_DEFAULT_CONFIG();
    scfg.server_port      = 81;
    scfg.ctrl_port        = 32769;
    scfg.max_uri_handlers = 2;
    scfg.stack_size       = 8192;
    scfg.send_wait_timeout = 10;
    scfg.recv_wait_timeout =  5;

    if (httpd_start(&stream_httpd, &scfg) == ESP_OK) {
        httpd_uri_t u = { "/stream", HTTP_GET, stream_handler, NULL };
        httpd_register_uri_handler(stream_httpd, &u);
    }

    // ── Camera control server port 80 ─────────────────────────────────────
    httpd_config_t ccfg = HTTPD_DEFAULT_CONFIG();
    ccfg.server_port      = 80;
    ccfg.ctrl_port        = 32768;
    ccfg.max_uri_handlers = 8;
    ccfg.stack_size       = 6144;

    if (httpd_start(&camera_httpd, &ccfg) == ESP_OK) {
        httpd_uri_t idx  = { "/",        HTTP_GET, index_handler,   NULL };
        httpd_uri_t cap  = { "/capture", HTTP_GET, capture_handler, NULL };
        httpd_uri_t stat = { "/status",  HTTP_GET, status_handler,  NULL };
        httpd_uri_t ctrl = { "/control", HTTP_GET, control_handler, NULL };
        httpd_register_uri_handler(camera_httpd, &idx);
        httpd_register_uri_handler(camera_httpd, &cap);
        httpd_register_uri_handler(camera_httpd, &stat);
        httpd_register_uri_handler(camera_httpd, &ctrl);
    }
}

// ═════════════════════════════════════════════════════════════════════════════
//  setup
// ═════════════════════════════════════════════════════════════════════════════
void setup()
{
    Serial.begin(115200);
    delay(500);

    Serial.println("\n══════════════════════════════════════");
    Serial.printf("[SYS] Chip   : %s  Rev %d\n",
                  ESP.getChipModel(), ESP.getChipRevision());
    Serial.printf("[SYS] Flash  : %u MB\n", ESP.getFlashChipSize() >> 20);
    Serial.printf("[SYS] PSRAM  : %s  (%u bytes)\n",
                  psramFound() ? "YES" : "NO", ESP.getPsramSize());
    Serial.printf("[SYS] Heap   : %u bytes free\n", ESP.getFreeHeap());

    if (!psramFound()) {
        Serial.println("[WARN] No PSRAM detected — check board_build.arduino.memory_type");
        Serial.println("[WARN] N16R8 needs qio_opi in platformio.ini");
    }

    if (!initCamera()) {
        Serial.println("[SYS] Halting — fix camera before continuing.");
        while (true) delay(1000);
    }

    connectWiFi();

    // Disable modem sleep — essential for smooth MJPEG streaming
    esp_wifi_set_ps(WIFI_PS_NONE);
    WiFi.setTxPower(WIFI_POWER_19_5dBm);

    startWebServers();

    Serial.println("══════════════════════════════════════");
    Serial.println("  UI + control : http://" + WiFi.localIP().toString() + "/");
    Serial.println("  MJPEG stream : http://" + WiFi.localIP().toString() + ":81/stream");
    Serial.println("  Snapshot     : http://" + WiFi.localIP().toString() + "/capture");
    Serial.println("  Status JSON  : http://" + WiFi.localIP().toString() + "/status");
    Serial.println("══════════════════════════════════════");
}

// ═════════════════════════════════════════════════════════════════════════════
//  loop — HTTP servers run on their own FreeRTOS tasks, nothing to do here
// ═════════════════════════════════════════════════════════════════════════════
void loop()
{
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("[WiFi] Connection lost — reconnecting");
        connectWiFi();
    }
    delay(5000);
}