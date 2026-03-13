#include <WiFi.h>
#include <AsyncTCP.h>
#include <ESPAsyncWebServer.h>

// --- Configuration ---
const char* ssid = "ESP32_Motor_Control";
const char* password = "password123";

const int ledPin = 4;
const int freq = 50;
const int resolution = 10; 

// ESC "Magic Numbers"
const int armSignal = 40;  
const int minSpin = 65;    
const int maxSpin = 250;   

// Control Variables
int targetValue = armSignal;
float currentVal = armSignal; // Float for smoother math
unsigned long lastUpdate = 0;
const int updateInterval = 20; // ms between ramp steps

AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// --- HTML Interface ---
const char index_html[] PROGMEM = R"rawliteral(
<!DOCTYPE HTML><html><head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>ESC Controller</title>
  <style>
    body { font-family: Arial; text-align: center; background: #1a1a1a; color: white; }
    .container { margin-top: 50px; }
    input[type=range] { width: 80%; height: 50px; }
    button { padding: 20px; font-size: 20px; margin: 10px; cursor: pointer; border-radius: 8px; border: none; }
    .reset { background: #ff4444; color: white; width: 80%; }
    .step { background: #4444ff; color: white; width: 35%; }
    h2 { color: #00ffcc; }
  </style>
</head><body>
  <div class="container">
    <h2>ESC Wireless Control</h2>
    <p>Current Power: <span id="valDisplay">0</span></p>
    <input type="range" id="pwmSlider" min="0" max="255" value="0" oninput="sendSlider(this.value)">
    <br>
    <button class="step" onclick="adjust(-2)">- 2</button>
    <button class="step" onclick="adjust(2)">+ 2</button>
    <br>
    <button class="reset" onclick="resetPWM()">INSTANT STOP</button>
  </div>
<script>
  var gateway = `ws://${window.location.hostname}/ws`;
  var websocket;
  window.addEventListener('load', onLoad);
  function onLoad() { websocket = new WebSocket(gateway); }
  
  function sendSlider(val) {
    document.getElementById("valDisplay").innerHTML = val;
    websocket.send(val);
  }
  function resetPWM() {
    document.getElementById("pwmSlider").value = 0;
    sendSlider(0);
  }
  function adjust(delta) {
    var s = document.getElementById("pwmSlider");
    s.value = parseInt(s.value) + delta;
    sendSlider(s.value);
  }
</script>
</body></html>
)rawliteral";

void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo*)arg;
  if (info->final && info->index == 0 && info->len == len && info->opcode == WS_TEXT) {
    data[len] = 0;
    int inputVal = atoi((char*)data);
    
    if (inputVal <= 0) {
      targetValue = armSignal;
      currentVal = armSignal; // Instant Stop
      ledcWrite(ledPin, armSignal);
      Serial.println(">>> EMERGENCY STOP <<<");
    } else {
      targetValue = map(inputVal, 1, 255, minSpin, maxSpin);
    }
  }
}

void onEvent(AsyncWebSocket *server, AsyncWebSocketClient *client, AwsEventType type, void *arg, uint8_t *data, size_t len) {
  if (type == WS_EVT_DATA) handleWebSocketMessage(arg, data, len);
}

void setup() {
  Serial.begin(115200);
  ledcAttach(ledPin, freq, resolution);
  ledcWrite(ledPin, armSignal);

  WiFi.softAP(ssid, password);
  Serial.println("Access Point Started");
  Serial.print("IP Address: "); Serial.println(WiFi.softAPIP());

  ws.onEvent(onEvent);
  server.addHandler(&ws);
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *request){
    request->send_P(200, "text/html", index_html);
  });

  server.begin();
}

void loop() {
  ws.cleanupClients();

  // Smoothing Logic (Ease-out)
  if (millis() - lastUpdate > updateInterval) {
    if (abs(currentVal - targetValue) > 0.5) {
      // Logic: Move 10% of the distance to the target every 20ms
      // This creates a smooth "slow start, slow finish" effect
      float diff = targetValue - currentVal;
      currentVal += diff * 0.1; 
      
      ledcWrite(ledPin, (int)currentVal);
      
      // Monitor Performance
      Serial.print("Target: "); Serial.print(targetValue);
      Serial.print(" | Actual PWM: "); Serial.println((int)currentVal);
    }
    lastUpdate = millis();
  }
}
