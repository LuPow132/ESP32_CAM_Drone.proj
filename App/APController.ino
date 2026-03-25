#include "esp_camera.h"
#include <Wire.h>
#include <WiFi.h>
#include <esp_wifi.h>
#include "esp_http_server.h"
#include <math.h>

// Workaround: esp_camera defines sensor_t; Adafruit_Sensor.h also defines it.
// Rename the Adafruit one before it's declared so both can coexist.
#define sensor_t adafruit_sensor_t
#include <Adafruit_Sensor.h>
#include <Adafruit_BMP280.h>
#undef sensor_t

// Select your camera model
#define CAMERA_MODEL_ESP32S3_EYE 
#include "camera_pins.h"

// ===========================
// Access Point Credentials
// ===========================
const char *ssid     = "DroneFPV_ESP32";
const char *password = "12345678";

// HTTP Server Handles
httpd_handle_t camera_httpd = NULL;
httpd_handle_t stream_httpd = NULL;

// ===========================
// I2C / Sensor Setup
// ===========================
#define I2C_SDA 38
#define I2C_SCL 39
#define MPU_ADDR 0x68

Adafruit_BMP280 bmp;

// Sensor state
float baseAltitude   = 0.0f;
float relAltitude    = 0.0f;   // metres above launch
float pitch_deg      = 0.0f;
float roll_deg       = 0.0f;

// Velocity/Speed approximation
float vx = 0.0f, vy = 0.0f, vz = 0.0f;
float speed_kmh = 0.0f;

// BMP280 altitude update (called in loop, ~500ms cadence)
unsigned long lastBmpMs = 0;

// MPU6050 complementary filter
float compPitch = 0.0f;
float compRoll  = 0.0f;
unsigned long lastMpuMs = 0;
bool mpuReady = false;
bool bmpReady = false;

void sensorsInit() {
  Wire.begin(I2C_SDA, I2C_SCL);

  // --- MPU6050 wake-up ---
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  if (Wire.endTransmission(true) == 0) {
    mpuReady = true;
    Serial.println("MPU6050 OK");
  } else {
    Serial.println("MPU6050 NOT FOUND");
  }

  // --- BMP280 init ---
  if (bmp.begin(0x77) || bmp.begin(0x76)) {
    bmp.setSampling(Adafruit_BMP280::MODE_NORMAL,
                    Adafruit_BMP280::SAMPLING_X2,
                    Adafruit_BMP280::SAMPLING_X16,
                    Adafruit_BMP280::FILTER_X16,
                    Adafruit_BMP280::STANDBY_MS_500);
    float sum = 0;
    for (int i = 0; i < 8; i++) { delay(100); sum += bmp.readAltitude(1013.25); }
    baseAltitude = sum / 8.0f;
    bmpReady = true;
    Serial.printf("BMP280 OK  base=%.2f m\n", baseAltitude);
  } else {
    Serial.println("BMP280 NOT FOUND");
  }
}

void sensorsUpdate() {
  unsigned long now = millis();

  // --- MPU6050 @ ~50 Hz ---
  if (mpuReady && (now - lastMpuMs >= 20)) {
    float dt = (now - lastMpuMs) / 1000.0f;
    lastMpuMs = now;

    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B);
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, (uint8_t)14, true);

    int16_t rAx = Wire.read()<<8|Wire.read();
    int16_t rAy = Wire.read()<<8|Wire.read();
    int16_t rAz = Wire.read()<<8|Wire.read();
    Wire.read(); Wire.read(); // temp
    int16_t rGx = Wire.read()<<8|Wire.read();
    int16_t rGy = Wire.read()<<8|Wire.read();
    Wire.read(); Wire.read(); // gz unused

    float ax = rAx / 16384.0f;
    float ay = rAy / 16384.0f;
    float az = rAz / 16384.0f;
    float gx = rGx / 131.0f;   // deg/s
    float gy = rGy / 131.0f;

    // Accelerometer angles
    float aPitch = atan2f(-ax, sqrtf(ay*ay + az*az)) * 57.2958f;
    float aRoll  = atan2f( ay, az)                   * 57.2958f;

    // Complementary filter  (98% gyro, 2% accel)
    compPitch = 0.98f*(compPitch + gx*dt) + 0.02f*aPitch;
    compRoll  = 0.98f*(compRoll  + gy*dt) + 0.02f*aRoll;

    pitch_deg = compPitch;
    roll_deg  = compRoll;

    // --- Approximate Speed Calculation ---
    float pitchRad = pitch_deg * 0.0174533f;
    float rollRad  = roll_deg * 0.0174533f;
    
    float gravX = -sinf(pitchRad);
    float gravY = sinf(rollRad) * cosf(pitchRad);
    float gravZ = cosf(rollRad) * cosf(pitchRad);

    float linAx = ax - gravX;
    float linAy = ay - gravY;
    float linAz = az - gravZ;

    vx += (linAx * 9.81f) * dt;
    vy += (linAy * 9.81f) * dt;
    vz += (linAz * 9.81f) * dt;

    vx *= 0.98f; 
    vy *= 0.98f; 
    vz *= 0.98f;

    speed_kmh = sqrtf(vx*vx + vy*vy + vz*vz) * 3.6f;
  }

  // --- BMP280 @ ~2 Hz ---
  if (bmpReady && (now - lastBmpMs >= 500)) {
    lastBmpMs = now;
    float alt = bmp.readAltitude(1013.25);
    relAltitude = alt - baseAltitude;
  }
}

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
    *{box-sizing:border-box;}
    body{margin:0;overflow:hidden;background:#000;color:#0f0;font-family:'Courier New',monospace;font-weight:bold;user-select:none;touch-action:none;}
    #stream{width:100vw;height:100vh;object-fit:cover;position:absolute;top:0;left:0;z-index:1;}

    /* ---- OSD base ---- */
    .osd{position:absolute;z-index:2;text-shadow:1px 1px 3px #000;font-size:1.1rem;pointer-events:none;}
    .top-left {top:20px;left:20px;}
    .top-right{top:20px;right:20px;color:#f00;animation:blink 1s step-start infinite;}
    .bottom-left {bottom:20px;left:20px;}
    .bottom-right{bottom:20px;right:20px;}
    .center-wrap{top:50%;left:50%;transform:translate(-50%,-50%);pointer-events:none;}
    @keyframes blink{50%{opacity:0;}}

    .interactive{pointer-events:auto;cursor:pointer;background:rgba(0,0,0,.5);padding:5px;border:1px solid #0f0; display:inline-block; margin-bottom:5px;}
    .interactive:hover{background:#0f0;color:#000;}

    #menu{position:absolute;z-index:3;top:100px;left:20px;background:rgba(0,0,0,.85);border:1px solid #0f0;padding:10px;display:none;pointer-events:auto; max-height:75vh; overflow-y:auto;}
    .menu-btn{display:block;width:100%;background:transparent;color:#0f0;border:1px solid #0f0;padding:10px 8px;margin:4px 0;font-family:inherit;font-weight:bold;cursor:pointer;text-align:left; font-size:0.9rem;}
    .menu-btn:hover{background:#0f0;color:#000;}
    .menu-section{color:#ff0; margin-top:10px; font-size:0.9rem;}

    /* ---- Artificial Horizon (SVG canvas) ---- */
    #horizon-svg{display:block;}

    /* ---- Joysticks ---- */
    .joystick-zone{position:absolute;bottom:50px;width:150px;height:150px;background:rgba(255,255,255,.05);border:2px solid rgba(0,255,0,.3);border-radius:50%;z-index:10;pointer-events:auto;}
    .joystick-knob{position:absolute;top:50px;left:50px;width:50px;height:50px;background:rgba(0,255,0,.4);border-radius:50%;pointer-events:none;}
  </style>
</head>
<body>
  <img id="stream" src="">

  <!-- top-left cluster -->
  <div class="osd top-left" style="line-height:1.8">
    <span class="interactive" onclick="toggleMenu()">[ MENU ]</span> 
    <span class="interactive" id="btnArm" onclick="toggleArm()">[ DISARMED ]</span>
    <span class="interactive" id="btnTakeoff">[ LANDED ]</span><br>
    
    <span id="osd-alt">ALT: <span id="altText">0.0</span> m | </span>
    <span id="osd-spd">SPD: <span id="spdText">0.0</span> km/h</span><br id="osd-br">
    
    <span id="osd-timer">ON: <span id="uptimeText">00:00</span> | FLY: <span id="flytimeText">00:00</span></span>
  </div>

  <div class="osd top-right">● REC</div>

  <!-- bottom-left cluster -->
  <div class="osd bottom-left" style="bottom:44px;">SIG: <span id="sigText">-- dBm</span></div>
  <div class="osd bottom-left">BATT: 4.2V</div>
  <div class="osd bottom-right" id="statusText">LINK: OK</div>

  <!-- Artificial Horizon centred -->
  <div class="osd center-wrap" style="position:absolute;">
    <svg id="horizon-svg" width="200" height="200" viewBox="-100 -100 200 200">
      <!-- static reticle -->
      <g stroke="#0f0" stroke-width="1.5" opacity="0.9">
        <line x1="-28" y1="0" x2="-10" y2="0"/>
        <line x1="10"  y1="0" x2="28"  y2="0"/>
        <line x1="0" y1="-6"  x2="0"   y2="6"/>
      </g>
      <!-- Dynamic Water Line -->
      <g id="ahHorizon">
        <line id="ahLineMain" x1="-60" y1="0" x2="60" y2="0" stroke="#0ff" stroke-width="2"/>
        <line id="ahTick1" x1="-20" y1="-20" x2="20" y2="-20" stroke="#0ff" stroke-width="1" opacity="0.5"/>
        <line id="ahTick2" x1="-20" y1="20" x2="20" y2="20" stroke="#0ff" stroke-width="1" opacity="0.5"/>
      </g>
    </svg>
  </div>

  <div id="joyL" class="joystick-zone" style="left:40px;"><div id="knobL" class="joystick-knob"></div></div>
  <div id="joyR" class="joystick-zone" style="right:40px;"><div id="knobR" class="joystick-knob"></div></div>

  <div id="menu">
    <div class="menu-section">VIDEO RES:</div>
    <button class="menu-btn" onclick="setRes(5)">> LOW (QVGA)</button>
    <button class="menu-btn" onclick="setRes(8)">> MID (VGA)</button>
    <button class="menu-btn" onclick="setRes(10)">> HIGH (SVGA)</button>
    
    <div class="menu-section">OSD TOGGLES (SAVE BW):</div>
    <button class="menu-btn" id="btn-tgl-alt" onclick="toggleOSD('alt')">> ALTITUDE [ON]</button>
    <button class="menu-btn" id="btn-tgl-spd" onclick="toggleOSD('spd')">> SPEED [ON]</button>
    <button class="menu-btn" id="btn-tgl-ah" onclick="toggleOSD('ah')">> HORIZON [ON]</button>
    <button class="menu-btn" id="btn-tgl-tmr" onclick="toggleOSD('tmr')">> TIMERS [ON]</button>

    <div class="menu-section">HORIZON SIZE:</div>
    <button class="menu-btn" onclick="setAhSize('sm')">> SMALL</button>
    <button class="menu-btn" onclick="setAhSize('md')">> MEDIUM</button>
    <button class="menu-btn" onclick="setAhSize('lg')">> LARGE</button>
  </div>

  <script>
    var stream    = document.getElementById('stream');
    var menu      = document.getElementById('menu');
    var statusEl  = document.getElementById('statusText');
    var sigEl     = document.getElementById('sigText');
    var altText   = document.getElementById('altText');
    var spdText   = document.getElementById('spdText');
    var ahHorizon = document.getElementById('ahHorizon');

    // ---- OSD State & Menus ----
    var osdState = { alt: true, spd: true, ah: true, tmr: true };
    
    function toggleMenu(){ menu.style.display=(menu.style.display==='block')?'none':'block'; }

    function toggleOSD(key) {
      osdState[key] = !osdState[key];
      document.getElementById('btn-tgl-'+key).innerText = "> " + 
        (key==='alt'?"ALTITUDE":key==='spd'?"SPEED":key==='ah'?"HORIZON":"TIMERS") + 
        " [" + (osdState[key] ? "ON" : "OFF") + "]";
      
      if(key === 'alt') document.getElementById('osd-alt').style.display = osdState.alt ? 'inline' : 'none';
      if(key === 'spd') document.getElementById('osd-spd').style.display = osdState.spd ? 'inline' : 'none';
      if(key === 'tmr') {
        document.getElementById('osd-timer').style.display = osdState.tmr ? 'inline' : 'none';
        document.getElementById('osd-br').style.display = (!osdState.alt && !osdState.spd) ? 'none' : 'inline';
      }
      if(key === 'ah') document.getElementById('horizon-svg').style.display = osdState.ah ? 'block' : 'none';
    }

    function setAhSize(sz) {
      var main = document.getElementById('ahLineMain');
      var t1 = document.getElementById('ahTick1');
      var t2 = document.getElementById('ahTick2');
      var w = sz==='sm' ? 40 : sz==='md' ? 60 : 90;
      var sw = sz==='sm' ? 2 : sz==='md' ? 3 : 4;
      main.setAttribute('x1', -w); main.setAttribute('x2', w);
      main.setAttribute('stroke-width', sw);
      t1.setAttribute('x1', -w/2); t1.setAttribute('x2', w/2);
      t2.setAttribute('x1', -w/2); t2.setAttribute('x2', w/2);
      menu.style.display = 'none';
    }

    function startStream(){
      statusEl.innerText='LINK: CONNECTING...'; statusEl.style.color='yellow';
      stream.src='http://192.168.4.1:81/stream?t='+Date.now();
    }
    function setRes(val){
      menu.style.display='none';
      fetch('http://192.168.4.1/control?var=framesize&val='+val).then(function(r){ if(r.ok) setTimeout(startStream,1000); });
    }
    stream.onload=function(){ statusEl.innerText='LINK: OK'; statusEl.style.color='#0f0'; };
    stream.onerror=function(){ statusEl.innerText='LINK: LOST! RECONNECTING...'; statusEl.style.color='red'; setTimeout(startStream,2000); };

    // ---- Button Logic & Timers ----
    var isArmed = false;
    var isFlying = false;
    var uptime = 0;
    var flytime = 0;

    function formatTime(s) {
      var m = Math.floor(s/60);
      var sec = s % 60;
      return (m<10?'0':'')+m+':'+(sec<10?'0':'')+sec;
    }

    setInterval(function(){
      uptime++;
      if(isArmed) flytime++;
      document.getElementById('uptimeText').innerText = formatTime(uptime);
      document.getElementById('flytimeText').innerText = formatTime(flytime);
    }, 1000);

    function toggleArm() {
      isArmed = !isArmed;
      var btn = document.getElementById('btnArm');
      btn.innerText = isArmed ? "[ ARMED ]" : "[ DISARMED ]";
      btn.style.color = isArmed ? "#f00" : "#0f0";
      btn.style.borderColor = isArmed ? "#f00" : "#0f0";
      fetch('http://192.168.4.1/control?var=arm&val='+(isArmed?1:0));
      if(isArmed) flytime = 0; 
    }

    // Hold-to-Takeoff Logic
    var takeoffTimer = null;
    var btnTakeoff = document.getElementById('btnTakeoff');

    function updateTakeoffUI() {
      btnTakeoff.innerText = isFlying ? "[ FLYING ]" : "[ LANDED ]";
      btnTakeoff.style.color = isFlying ? "#ff0" : "#0f0";
      btnTakeoff.style.borderColor = isFlying ? "#ff0" : "#0f0";
    }

    btnTakeoff.addEventListener('pointerdown', function(e) {
      btnTakeoff.setPointerCapture(e.pointerId);
      btnTakeoff.innerText = isFlying ? "HOLD TO LAND..." : "HOLD TO TAKEOFF...";
      btnTakeoff.style.color = "#f80";
      btnTakeoff.style.borderColor = "#f80";
      
      takeoffTimer = setTimeout(function() {
        isFlying = !isFlying;
        updateTakeoffUI();
        fetch('http://192.168.4.1/control?var=takeoff&val='+(isFlying?1:0));
        takeoffTimer = null;
      }, 2000); // 2 seconds hold
    });

    function cancelTakeoff(e) {
      if (takeoffTimer) {
        clearTimeout(takeoffTimer);
        takeoffTimer = null;
        updateTakeoffUI();
      }
      btnTakeoff.releasePointerCapture(e.pointerId);
    }
    btnTakeoff.addEventListener('pointerup', cancelTakeoff);
    btnTakeoff.addEventListener('pointercancel', cancelTakeoff);

    // ---- RSSI ----
    function fetchRSSI(){
      fetch('http://192.168.4.1/rssi').then(function(r){return r.text();}).then(function(v){
        var dbm=parseInt(v);
        if(isNaN(dbm)||dbm===0){sigEl.innerText='-- dBm';sigEl.style.color='#0f0';return;}
        sigEl.innerText=dbm+' dBm';
        sigEl.style.color=(dbm>=-60)?'#0f0':(dbm>=-80)?'yellow':'red';
      }).catch(function(){sigEl.innerText='-- dBm';sigEl.style.color='red';});
    }
    setInterval(fetchRSSI,2000);

    // ---- Sensors (Dynamic Bandwidth) ----
    function fetchSensors(){
      // Only request what the UI needs to show
      var q = '?alt='+(osdState.alt?1:0)+'&spd='+(osdState.spd?1:0)+'&ah='+(osdState.ah?1:0);
      fetch('http://192.168.4.1/sensors'+q).then(function(r){return r.json();}).then(function(d){
        if(d.alt !== undefined) altText.innerText=parseFloat(d.alt).toFixed(1);
        if(d.spd !== undefined) spdText.innerText=parseFloat(d.spd).toFixed(1);

        if(d.pitch !== undefined && d.roll !== undefined) {
          var pitchPx = Math.max(Math.min(parseFloat(d.pitch) * 1.5, 80), -80);
          ahHorizon.setAttribute('transform','rotate('+parseFloat(d.roll)+') translate(0,'+pitchPx+')');
        }
      }).catch(function(){});
    }
    setInterval(fetchSensors, 150); // ~6 Hz refresh

    // ---- Joystick ----
    var joyData={lx:0,ly:0,rx:0,ry:0};
    var joyIsSending=false;

    function initJoy(zoneId,knobId,isLeft){
      var zone=document.getElementById(zoneId);
      var knob=document.getElementById(knobId);
      var active=false,cx=75,maxR=50;

      zone.addEventListener('pointerdown',function(e){active=true;zone.setPointerCapture(e.pointerId);upd(e);});
      zone.addEventListener('pointermove',function(e){if(active)upd(e);});
      zone.addEventListener('pointerup',  function(e){active=false;zone.releasePointerCapture(e.pointerId);rst();});
      zone.addEventListener('pointercancel',function(){active=false;rst();});

      function upd(e){
        var r=zone.getBoundingClientRect();
        var x=e.clientX-r.left-cx, y=e.clientY-r.top-cx;
        var d=Math.sqrt(x*x+y*y);
        if(d>maxR){x=x/d*maxR;y=y/d*maxR;}
        knob.style.transform='translate('+x+'px,'+y+'px)';
        var mx=Math.round(x/maxR*100), my=Math.round(y/maxR*-100);
        if(isLeft){joyData.lx=mx;joyData.ly=my;}else{joyData.rx=mx;joyData.ry=my;}
        send();
      }
      function rst(){
        knob.style.transform='translate(0px,0px)';
        if(isLeft){joyData.lx=0;joyData.ly=0;}else{joyData.rx=0;joyData.ry=0;}
        send();
      }
    }

    function send(){
      if(joyIsSending)return;
      joyIsSending=true;
      fetch('http://192.168.4.1/joy?lx='+joyData.lx+'&ly='+joyData.ly+'&rx='+joyData.rx+'&ry='+joyData.ry)
        .then(function(){joyIsSending=false;}).catch(function(){joyIsSending=false;});
    }

    initJoy('joyL','knobL',true);
    initJoy('joyR','knobR',false);

    // Boot
    setTimeout(startStream,500);
    fetchRSSI();
    fetchSensors();
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
static const char* _STREAM_PART     = "Content-Type: image/jpeg\r\nContent-Length: %u\r\n\r\n";

// ===========================
// HTTP Handlers
// ===========================
static esp_err_t index_handler(httpd_req_t *req){
  httpd_resp_set_type(req,"text/html");
  return httpd_resp_send(req,INDEX_HTML,strlen(INDEX_HTML));
}

static esp_err_t rssi_handler(httpd_req_t *req){
  wifi_sta_list_t clients;
  int raw_rssi=0;
  if(esp_wifi_ap_get_sta_list(&clients)==ESP_OK && clients.num>0)
    raw_rssi=clients.sta[0].rssi;
  char buf[16];
  snprintf(buf,sizeof(buf),"%d",raw_rssi);
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  return httpd_resp_send(req,buf,strlen(buf));
}

// UPDATED: Dynamic /sensors → Only sends requested data based on query params to save bandwidth
static esp_err_t sensors_handler(httpd_req_t *req){
  char* qbuf; 
  size_t qbuf_len;
  bool req_alt = true, req_spd = true, req_ah = true;
  
  // 1. Read the GET query params from JavaScript
  qbuf_len = httpd_req_get_url_query_len(req) + 1;
  if(qbuf_len > 1){
    qbuf = (char*)malloc(qbuf_len);
    if(httpd_req_get_url_query_str(req, qbuf, qbuf_len) == ESP_OK){
      char val[5]={0};
      if(httpd_query_key_value(qbuf,"alt",val,sizeof(val))==ESP_OK) req_alt = atoi(val);
      if(httpd_query_key_value(qbuf,"spd",val,sizeof(val))==ESP_OK) req_spd = atoi(val);
      if(httpd_query_key_value(qbuf,"ah",val,sizeof(val))==ESP_OK)  req_ah  = atoi(val);
    }
    free(qbuf);
  }

  // 2. Build the JSON string dynamically
  char resp[150];
  int offset = 0;
  offset += snprintf(resp+offset, sizeof(resp)-offset, "{");
  
  bool first = true;
  if(req_alt) { 
    offset += snprintf(resp+offset, sizeof(resp)-offset, "\"alt\":%.2f", relAltitude); 
    first = false; 
  }
  if(req_spd) { 
    if(!first) { offset += snprintf(resp+offset, sizeof(resp)-offset, ","); }
    offset += snprintf(resp+offset, sizeof(resp)-offset, "\"spd\":%.1f", speed_kmh); 
    first = false; 
  }
  if(req_ah) { 
    if(!first) { offset += snprintf(resp+offset, sizeof(resp)-offset, ","); }
    offset += snprintf(resp+offset, sizeof(resp)-offset, "\"pitch\":%.1f,\"roll\":%.1f", pitch_deg, roll_deg); 
  }
  offset += snprintf(resp+offset, sizeof(resp)-offset, "}");

  // 3. Send
  httpd_resp_set_type(req,"application/json");
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  return httpd_resp_send(req, resp, strlen(resp));
}

static esp_err_t joy_handler(httpd_req_t *req){
  char* buf; size_t buf_len;
  char lx_s[10]={0},ly_s[10]={0},rx_s[10]={0},ry_s[10]={0};
  buf_len=httpd_req_get_url_query_len(req)+1;
  if(buf_len>1){
    buf=(char*)malloc(buf_len);
    if(httpd_req_get_url_query_str(req,buf,buf_len)==ESP_OK){
      httpd_query_key_value(buf,"lx",lx_s,sizeof(lx_s));
      httpd_query_key_value(buf,"ly",ly_s,sizeof(ly_s));
      httpd_query_key_value(buf,"rx",rx_s,sizeof(rx_s));
      httpd_query_key_value(buf,"ry",ry_s,sizeof(ry_s));
    }
    free(buf);
  }
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  return httpd_resp_send(req,NULL,0);
}

static esp_err_t stream_handler(httpd_req_t *req){
  camera_fb_t *fb=NULL;
  esp_err_t res=ESP_OK;
  size_t _jpg_buf_len=0;
  uint8_t *_jpg_buf=NULL;
  char part_buf[64];

  res=httpd_resp_set_type(req,_STREAM_CONTENT_TYPE);
  if(res!=ESP_OK)return res;
  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  res=httpd_resp_send_chunk(req,_STREAM_BOUNDARY,strlen(_STREAM_BOUNDARY));

  while(true){
    fb=esp_camera_fb_get();
    if(!fb){ res=ESP_FAIL; }
    else { _jpg_buf_len=fb->len; _jpg_buf=fb->buf; }

    if(res==ESP_OK){ size_t hlen=snprintf(part_buf,64,_STREAM_PART,_jpg_buf_len); res=httpd_resp_send_chunk(req,(const char*)part_buf,hlen); }
    if(res==ESP_OK){ res=httpd_resp_send_chunk(req,(const char*)_jpg_buf,_jpg_buf_len); }
    if(res==ESP_OK){ res=httpd_resp_send_chunk(req,_STREAM_BOUNDARY,strlen(_STREAM_BOUNDARY)); }

    if(fb){ esp_camera_fb_return(fb); fb=NULL; _jpg_buf=NULL; }
    if(res!=ESP_OK) break;
  }
  return res;
}

static esp_err_t cmd_handler(httpd_req_t *req){
  char* buf; size_t buf_len;
  char variable[32]={0},value[32]={0};
  buf_len=httpd_req_get_url_query_len(req)+1;
  if(buf_len>1){
    buf=(char*)malloc(buf_len);
    if(httpd_req_get_url_query_str(req,buf,buf_len)==ESP_OK){
      httpd_query_key_value(buf,"var",variable,sizeof(variable));
      httpd_query_key_value(buf,"val",value,sizeof(value));
    }
    free(buf);
  }
  int val=atoi(value);
  
  if(!strcmp(variable,"framesize")) {
    sensor_t *s=esp_camera_sensor_get();
    if(s->pixformat==PIXFORMAT_JPEG) s->set_framesize(s,(framesize_t)val);
  } 
  else if (!strcmp(variable, "arm")) {
    Serial.printf("UI COMMAND -> ARM STATE: %d\n", val);
  }
  else if (!strcmp(variable, "takeoff")) {
    Serial.printf("UI COMMAND -> TAKEOFF/LAND STATE: %d\n", val);
  }

  httpd_resp_set_hdr(req,"Access-Control-Allow-Origin","*");
  return httpd_resp_send(req,NULL,0);
}

// ===========================
// Server Start (Ports 80 & 81)
// ===========================
void startCameraServer(){
  httpd_config_t config=HTTPD_DEFAULT_CONFIG();
  config.server_port=80;

  httpd_uri_t index_uri   ={.uri="/",        .method=HTTP_GET,.handler=index_handler,  .user_ctx=NULL};
  httpd_uri_t cmd_uri     ={.uri="/control", .method=HTTP_GET,.handler=cmd_handler,    .user_ctx=NULL};
  httpd_uri_t joy_uri     ={.uri="/joy",     .method=HTTP_GET,.handler=joy_handler,    .user_ctx=NULL};
  httpd_uri_t rssi_uri    ={.uri="/rssi",    .method=HTTP_GET,.handler=rssi_handler,   .user_ctx=NULL};
  httpd_uri_t sensors_uri ={.uri="/sensors", .method=HTTP_GET,.handler=sensors_handler,.user_ctx=NULL};

  if(httpd_start(&camera_httpd,&config)==ESP_OK){
    httpd_register_uri_handler(camera_httpd,&index_uri);
    httpd_register_uri_handler(camera_httpd,&cmd_uri);
    httpd_register_uri_handler(camera_httpd,&joy_uri);
    httpd_register_uri_handler(camera_httpd,&rssi_uri);
    httpd_register_uri_handler(camera_httpd,&sensors_uri);
  }

  httpd_config_t config_stream=HTTPD_DEFAULT_CONFIG();
  config_stream.server_port=81;
  config_stream.ctrl_port=81;

  httpd_uri_t stream_uri={.uri="/stream",.method=HTTP_GET,.handler=stream_handler,.user_ctx=NULL};
  if(httpd_start(&stream_httpd,&config_stream)==ESP_OK)
    httpd_register_uri_handler(stream_httpd,&stream_uri);
}

// ===========================
// Setup & Loop
// ===========================
void setup(){
  Serial.begin(115200);
  Serial.setDebugOutput(false);

  camera_config_t config;
  config.ledc_channel=LEDC_CHANNEL_0;
  config.ledc_timer=LEDC_TIMER_0;
  config.pin_d0=Y2_GPIO_NUM; config.pin_d1=Y3_GPIO_NUM;
  config.pin_d2=Y4_GPIO_NUM; config.pin_d3=Y5_GPIO_NUM;
  config.pin_d4=Y6_GPIO_NUM; config.pin_d5=Y7_GPIO_NUM;
  config.pin_d6=Y8_GPIO_NUM; config.pin_d7=Y9_GPIO_NUM;
  config.pin_xclk=XCLK_GPIO_NUM; config.pin_pclk=PCLK_GPIO_NUM;
  config.pin_vsync=VSYNC_GPIO_NUM; config.pin_href=HREF_GPIO_NUM;
  config.pin_sccb_sda=SIOD_GPIO_NUM; config.pin_sccb_scl=SIOC_GPIO_NUM;
  config.pin_pwdn=PWDN_GPIO_NUM; config.pin_reset=RESET_GPIO_NUM;
  config.xclk_freq_hz=20000000;
  config.frame_size=FRAMESIZE_VGA;
  config.pixel_format=PIXFORMAT_JPEG;
  config.grab_mode=CAMERA_GRAB_LATEST;
  config.fb_location=CAMERA_FB_IN_PSRAM;
  config.jpeg_quality=12;
  config.fb_count=2;

  esp_err_t err=esp_camera_init(&config);
  if(err!=ESP_OK){ Serial.printf("Camera init failed 0x%x\n",err); return; }
  esp_camera_sensor_get()->set_vflip(esp_camera_sensor_get(),1);

  sensorsInit();

  Serial.println("Starting Access Point...");
  WiFi.mode(WIFI_AP);
  WiFi.softAP(ssid,password);

  startCameraServer();
  Serial.println("FPV Server Ready!");
}

void loop(){
  sensorsUpdate(); 
  delay(1);       
}
