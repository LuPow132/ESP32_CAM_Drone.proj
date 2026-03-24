#include "esp_camera.h"
#include <WiFi.h>
#include <esp_wifi.h> // Needed to read AP client RSSI
#include "esp_http_server.h"

// Select your camera model
#define CAMERA_MODEL_ESP32S3_EYE 
#include "camera_pins.h"

// ===========================
// Access Point Credentials
// ===========================
const char *ssid = "DroneFPV_ESP32";
const char *password = "12345678";

// HTTP Server Handles
httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

// ===========================
// HTML + CSS + JS Frontend
// ===========================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, maximum-scale=1, user-scalable=no">
  <style>
    body { margin: 0; overflow: hidden; background-color: #000; color: #0f0; font-family: 'Courier New', Courier, monospace; font-weight: bold; user-select: none; touch-action: none; }
    #stream { width: 100vw; height: 100vh; object-fit: cover; position: absolute; top: 0; left: 0; z-index: 1; }
    
    .osd { position: absolute; z-index: 2; text-shadow: 2px 2px 2px #000; font-size: 1.2rem; pointer-events: none; }
    .top-left { top: 20px; left: 20px; }
    .top-right { top: 20px; right: 20px; color: #f00; animation: blink 1s step-start infinite; }
    .bottom-left { bottom: 20px; left: 20px; }
    .bottom-right { bottom: 20px; right: 20px; }
    .center-crosshair { top: 50%; left: 50%; transform: translate(-50%, -50%); font-size: 2rem; font-weight: normal; }
    
    @keyframes blink { 50% { opacity: 0; } }

    .interactive { pointer-events: auto; cursor: pointer; background: rgba(0,0,0,0.5); padding: 5px; border: 1px solid #0f0; }
    .interactive:hover { background: #0f0; color: #000; }
    
    #menu { position: absolute; z-index: 3; top: 60px; left: 20px; background: rgba(0, 0, 0, 0.8); border: 1px solid #0f0; padding: 10px; display: none; pointer-events: auto;}
    .menu-btn { display: block; width: 100%; background: transparent; color: #0f0; border: 1px solid #0f0; padding: 15px 8px; margin: 5px 0; font-family: inherit; font-weight: bold; cursor: pointer; text-align: left;}
    .menu-btn:hover { background: #0f0; color: #000; }

    /* Virtual Joysticks */
    .joystick-zone { position: absolute; bottom: 50px; width: 150px; height: 150px; background: rgba(255, 255, 255, 0.05); border: 2px solid rgba(0, 255, 0, 0.3); border-radius: 50%; z-index: 10; pointer-events: auto; }
    .joystick-knob { position: absolute; top: 50px; left: 50px; width: 50px; height: 50px; background: rgba(0, 255, 0, 0.4); border-radius: 50%; pointer-events: none; transform: translate(0px, 0px); }
  </style>
</head>
<body>
  <img id="stream" src="">

  <div class="osd top-left"><span class="interactive" onclick="toggleMenu()">[ MENU ]</span> &nbsp; CH: R8</div>
  <div class="osd top-right">● REC</div>
  <div class="osd center-crosshair">⊹</div>
  <div class="osd bottom-left" style="bottom: 40px;">SIG: <span id="sigText">-- dBm</span></div>
  <div class="osd bottom-left">BATT: 4.2V</div>
  <div class="osd bottom-right" id="statusText">LINK: OK</div>

  <div id="joyL" class="joystick-zone" style="left: 40px;"><div id="knobL" class="joystick-knob"></div></div>
  <div id="joyR" class="joystick-zone" style="right: 40px;"><div id="knobR" class="joystick-knob"></div></div>

  <div id="menu">
    <div>VIDEO RESOLUTION:</div>
    <button class="menu-btn" onclick="setRes(5)">> LOW (QVGA)</button>
    <button class="menu-btn" onclick="setRes(8)">> MID (VGA)</button>
    <button class="menu-btn" onclick="setRes(10)">> HIGH (SVGA)</button>
  </div>

  <script>
    var stream = document.getElementById('stream');
    var menu = document.getElementById('menu');
    var statusEl = document.getElementById('statusText');
    var sigEl = document.getElementById('sigText');
    
    function toggleMenu() {
      menu.style.display = (menu.style.display === 'block') ? 'none' : 'block';
    }

    function startStream() {
      statusEl.innerText = "LINK: CONNECTING...";
      statusEl.style.color = "yellow";
      stream.src = "http://192.168.4.1:81/stream?t=" + new Date().getTime(); 
    }

    function setRes(val) {
      menu.style.display = 'none';
      fetch("http://192.168.4.1/control?var=framesize&val=" + val).then(function(res) {
        if(res.ok) setTimeout(startStream, 1000); 
      });
    }

    stream.onload = function() {
      statusEl.innerText = "LINK: OK";
      statusEl.style.color = "#0f0";
    };

    stream.onerror = function() {
      statusEl.innerText = "LINK: LOST! RECONNECTING...";
      statusEl.style.color = "red";
      setTimeout(startStream, 2000);
    };

    // ==========================================
    // Fetch Raw Signal Strength (dBm) & Colorize
    // ==========================================
    function fetchRSSI() {
      fetch("http://192.168.4.1/rssi")
        .then(function(res) { return res.text(); })
        .then(function(val) { 
          var dbm = parseInt(val);
          if (isNaN(dbm) || dbm === 0) {
             sigEl.innerText = "-- dBm";
             sigEl.style.color = "#0f0";
             return;
          }
          
          sigEl.innerText = dbm + " dBm";
          
          // Color Logic based on raw dBm thresholds
          if (dbm >= -60) {
            sigEl.style.color = "#0f0";     // Green (Excellent)
          } else if (dbm >= -80) {
            sigEl.style.color = "yellow";   // Yellow (Great/Good)
          } else {
            sigEl.style.color = "red";      // Red (Bad)
          }
        })
        .catch(function() { 
          sigEl.innerText = "-- dBm"; 
          sigEl.style.color = "red";
        });
    }
    setInterval(fetchRSSI, 2000); // Poll every 2 seconds

    // ==========================================
    // Joystick Logic
    // ==========================================
    var joyData = { lx: 0, ly: 0, rx: 0, ry: 0 };
    var joyIsSending = false;

    function initJoy(zoneId, knobId, isLeft) {
      var zone = document.getElementById(zoneId);
      var knob = document.getElementById(knobId);
      var active = false;
      var centerOffset = 75; // Center of 150px zone
      var maxRadius = 50;    // Max distance knob can travel

      zone.addEventListener('pointerdown', function(e) { active = true; zone.setPointerCapture(e.pointerId); updateJoy(e); });
      zone.addEventListener('pointermove', function(e) { if(active) updateJoy(e); });
      zone.addEventListener('pointerup', function(e) { active = false; zone.releasePointerCapture(e.pointerId); resetJoy(); });
      zone.addEventListener('pointercancel', function() { active = false; resetJoy(); });

      function updateJoy(e) {
        var rect = zone.getBoundingClientRect();
        var x = e.clientX - rect.left - centerOffset;
        var y = e.clientY - rect.top - centerOffset;

        var dist = Math.sqrt(x*x + y*y);
        if(dist > maxRadius) {
          x = (x / dist) * maxRadius;
          y = (y / dist) * maxRadius;
        }

        knob.style.transform = `translate(${x}px, ${y}px)`;

        var mappedX = Math.round((x / maxRadius) * 100);
        var mappedY = Math.round((y / maxRadius) * -100);

        if(isLeft) { joyData.lx = mappedX; joyData.ly = mappedY; } 
        else { joyData.rx = mappedX; joyData.ry = mappedY; }
        
        sendJoyData();
      }

      function resetJoy() {
        knob.style.transform = `translate(0px, 0px)`;
        if(isLeft) { joyData.lx = 0; joyData.ly = 0; } 
        else { joyData.rx = 0; joyData.ry = 0; }
        sendJoyData();
      }
    }

    initJoy('joyL', 'knobL', true);
    initJoy('joyR', 'knobR', false);

    function sendJoyData() {
      if(joyIsSending) return; 
      joyIsSending = true;

      fetch(`http://192.168.4.1/joy?lx=${joyData.lx}&ly=${joyData.ly}&rx=${joyData.rx}&ry=${joyData.ry}`)
        .then(function() { joyIsSending = false; })
        .catch(function() { joyIsSending = false; });
    }

    // Boot up
    setTimeout(startStream, 500);
    fetchRSSI();
  </script>
</body>
</html>
)rawliteral";

// ===========================
// Stream Boundary Defines
// ===========================
#define PART_BOUNDARY "123456789000000000000987654321"
static const char* _STREAM_CONTENT_TYPE = "multipart/x-mixed-replace;boundary=" PART_BOUNDARY;
static const char* _STREAM_BOUNDARY = "\r\n--" PART_BOUNDARY "\r\n";
static const char* _STREAM_PART = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ===========================
// HTTP Handlers
// ===========================
static esp_err_t index_handler(httpd_req_t *req) {
  httpd_resp_set_type(req, "text/html");
  return httpd_resp_send(req, INDEX_HTML, strlen(INDEX_HTML));
}

// Handler for raw dBm RSSI fetching
static esp_err_t rssi_handler(httpd_req_t *req) {
  wifi_sta_list_t clients;
  int raw_rssi = 0;
  
  if (esp_wifi_ap_get_sta_list(&clients) == ESP_OK && clients.num > 0) {
    raw_rssi = clients.sta[0].rssi;
  }

  char buf[16];
  snprintf(buf, sizeof(buf), "%d", raw_rssi);
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, buf, strlen(buf));
}

static esp_err_t joy_handler(httpd_req_t *req) {
  char* buf;
  size_t buf_len;
  char lx_s[10] = {0}, ly_s[10] = {0}, rx_s[10] = {0}, ry_s[10] = {0};

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      httpd_query_key_value(buf, "lx", lx_s, sizeof(lx_s));
      httpd_query_key_value(buf, "ly", ly_s, sizeof(ly_s));
      httpd_query_key_value(buf, "rx", rx_s, sizeof(rx_s));
      httpd_query_key_value(buf, "ry", ry_s, sizeof(ry_s));
      
      int lx = atoi(lx_s);
      int ly = atoi(ly_s);
      int rx = atoi(rx_s);
      int ry = atoi(ry_s);

      Serial.printf("JOY INPUT -> LX: %4d | LY: %4d | RX: %4d | RY: %4d\n", lx, ly, rx, ry);
    }
    free(buf);
  }
  
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0); 
}

static esp_err_t stream_handler(httpd_req_t *req) {
  camera_fb_t * fb = NULL;
  esp_err_t res = ESP_OK;
  size_t _jpg_buf_len = 0;
  uint8_t * _jpg_buf = NULL;
  char part_buf[64];

  res = httpd_resp_set_type(req, _STREAM_CONTENT_TYPE);
  if(res != ESP_OK) return res;
  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");

  res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));

  while(true) {
    fb = esp_camera_fb_get();
    if (!fb) {
      Serial.println("Camera capture failed");
      res = ESP_FAIL;
    } else {
      _jpg_buf_len = fb->len;
      _jpg_buf = fb->buf;
    }
    
    if(res == ESP_OK){
      size_t hlen = snprintf(part_buf, 64, _STREAM_PART, _jpg_buf_len);
      res = httpd_resp_send_chunk(req, (const char *)part_buf, hlen);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, (const char *)_jpg_buf, _jpg_buf_len);
    }
    if(res == ESP_OK){
      res = httpd_resp_send_chunk(req, _STREAM_BOUNDARY, strlen(_STREAM_BOUNDARY));
    }
    
    if(fb){
      esp_camera_fb_return(fb);
      fb = NULL;
      _jpg_buf = NULL;
    }
    
    if(res != ESP_OK){
      break;
    }
  }
  return res;
}

static esp_err_t cmd_handler(httpd_req_t *req) {
  char* buf;
  size_t buf_len;
  char variable[32] = {0,};
  char value[32] = {0,};

  buf_len = httpd_req_get_url_query_len(req) + 1;
  if (buf_len > 1) {
    buf = (char*)malloc(buf_len);
    if (httpd_req_get_url_query_str(req, buf, buf_len) == ESP_OK) {
      if (httpd_query_key_value(buf, "var", variable, sizeof(variable)) == ESP_OK &&
          httpd_query_key_value(buf, "val", value, sizeof(value)) == ESP_OK) {
      }
    }
    free(buf);
  }

  int val = atoi(value);
  sensor_t * s = esp_camera_sensor_get();
  
  if(!strcmp(variable, "framesize") && s->pixformat == PIXFORMAT_JPEG) {
    s->set_framesize(s, (framesize_t)val);
  }

  httpd_resp_set_hdr(req, "Access-Control-Allow-Origin", "*");
  return httpd_resp_send(req, NULL, 0);
}

// ===========================
// Server Start (Ports 80 & 81)
// ===========================
void startCameraServer() {
  httpd_config_t config = HTTPD_DEFAULT_CONFIG();
  config.server_port = 80;

  httpd_uri_t index_uri = { .uri = "/", .method = HTTP_GET, .handler = index_handler, .user_ctx = NULL };
  httpd_uri_t cmd_uri = { .uri = "/control", .method = HTTP_GET, .handler = cmd_handler, .user_ctx = NULL };
  httpd_uri_t joy_uri = { .uri = "/joy", .method = HTTP_GET, .handler = joy_handler, .user_ctx = NULL };
  httpd_uri_t rssi_uri = { .uri = "/rssi", .method = HTTP_GET, .handler = rssi_handler, .user_ctx = NULL };
  
  if (httpd_start(&camera_httpd, &config) == ESP_OK) {
    httpd_register_uri_handler(camera_httpd, &index_uri);
    httpd_register_uri_handler(camera_httpd, &cmd_uri);
    httpd_register_uri_handler(camera_httpd, &joy_uri);
    httpd_register_uri_handler(camera_httpd, &rssi_uri); 
  }

  httpd_config_t config_stream = HTTPD_DEFAULT_CONFIG();
  config_stream.server_port = 81;
  config_stream.ctrl_port = 81; 
  
  httpd_uri_t stream_uri = { .uri = "/stream", .method = HTTP_GET, .handler = stream_handler, .user_ctx = NULL };
  
  if (httpd_start(&stream_httpd, &config_stream) == ESP_OK) {
    httpd_register_uri_handler(stream_httpd, &stream_uri);
  }
}

void setup() {
  Serial.begin(115200);
  Serial.setDebugOutput(false); 
  Serial.println();

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.frame_size = FRAMESIZE_VGA; 
  config.pixel_format = PIXFORMAT_JPEG;
  config.grab_mode = CAMERA_GRAB_LATEST;
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.jpeg_quality = 12;
  config.fb_count = 2;

  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    Serial.printf("Camera init failed with error 0x%x", err);
    return;
  }

  sensor_t *s = esp_camera_sensor_get();
  s->set_vflip(s, 1); 

  Serial.println("Starting Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid, password);
  
  startCameraServer();

  Serial.println("FPV Server Ready!");
}

void loop() {
  delay(10000);
}
