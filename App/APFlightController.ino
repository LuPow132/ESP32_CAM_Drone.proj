/*
 * ============================================================
 *  DRONE FLIGHT CONTROLLER  —  ESP32-S3
 * ============================================================
 *  Motor layout (X-frame, top view):
 *    M2(FL,GPIO1) | M4(FR,GPIO42)
 *    M1(BL,GPIO4) | M3(BR,GPIO2)
 *
 *  ESC: DL03  50Hz PWM  0.9ms(stop) → 2.0ms(max)
 *  10-bit resolution → stop=46, idle=54(bench-verify), max=102
 *
 *  I2C: SDA=38, SCL=39
 *  MPU6050: 0x68
 *
 *  Modes: ANGLE (self-level) / ACRO (rate)
 *
 *  Architecture: stabilization runs in a fixed 500Hz FreeRTOS task
 *  (controlTask), fully decoupled from WiFi. The web handlers only
 *  write setpoints; a link-loss failsafe levels & idles the craft if
 *  no /drive packet arrives within FAILSAFE_MS.
 * ============================================================
 */

#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <ArduinoJson.h>

// ─── WiFi AP ────────────────────────────────────────────────
const char* AP_SSID = "DroneFC";
const char* AP_PASS = "12345678";

// ─── Motor GPIO ─────────────────────────────────────────────
#define M1_PIN  4   // Back  Left  (CCW)
#define M2_PIN  1   // Front Left  (CW)
#define M3_PIN  2   // Back  Right (CW)
#define M4_PIN  42  // Front Right (CCW)

// ─── ESC PWM params ─────────────────────────────────────────
#define PWM_FREQ       50
#define PWM_RESOLUTION 10          // 10-bit → 1024 steps, 1 step ≈ 0.01953ms
#define ESC_STOP       46          // 0.9ms  → 46.08 ≈ 46
#define ESC_IDLE       54          // armed idle: ALL 4 props must spin here.
                                   //   Bench-verify props-off; sandbox found ~65 needed.
#define ESC_MAX        102         // 2.0ms  → 102.4 ≈ 102
#define ESC_ARM_DELAY  2200        // ms holding stop signal at boot for ESC init

// ─── Control loop ───────────────────────────────────────────
#define LOOP_HZ      500                          // fixed-rate stabilization loop
#define LOOP_PERIOD  pdMS_TO_TICKS(1000 / LOOP_HZ)
#define FAILSAFE_MS  400                          // no /drive packet → level & idle
#define I_LIMIT      40.0f                         // integral clamp
#define PID_LIMIT    30.0f                         // per-axis correction clamp (ESC units)

// ─── Mixer sign trims ───────────────────────────────────────
// Flip any of these to -1 if the props-off tilt test shows an axis
// correcting the WRONG way (watch the motor values in /telem or serial).
#define ROLL_SIGN   (+1)
#define PITCH_SIGN  (+1)
#define YAW_SIGN    (+1)

// ─── I2C / MPU6050 ──────────────────────────────────────────
#define SDA_PIN   38
#define SCL_PIN   39
#define MPU_ADDR  0x68

// ─── PID defaults ───────────────────────────────────────────
struct PIDGains { float kp, ki, kd; };

PIDGains pidRoll  = {1.2f, 0.05f, 0.08f};
PIDGains pidPitch = {1.2f, 0.05f, 0.08f};
PIDGains pidYaw   = {2.0f, 0.10f, 0.00f};

// ─── State ──────────────────────────────────────────────────
enum FlightMode { ANGLE_MODE, ACRO_MODE };
FlightMode flightMode = ANGLE_MODE;

bool armed = false;

// ─── Setpoints (written by /drive, read by control task) ────
//   normalized -1..+1.  spThrottle useful range 0..+1.
volatile float spThrottle = 0, spRoll = 0, spPitch = 0, spYaw = 0;
volatile uint32_t lastDriveMs = 0;

// Gyro bias (deg/s), measured & removed at boot by calibrateGyro()
float gxBias = 0, gyBias = 0, gzBias = 0;

// Last motor outputs (ESC units), for telemetry / props-off bench test
float lastM[4] = {0, 0, 0, 0};

// ─── IMU data ───────────────────────────────────────────────
struct IMUData {
  float ax, ay, az;   // g
  float gx, gy, gz;   // deg/s
  float roll, pitch;  // degrees (complementary filter)
};
IMUData imu;

// ─── PID state ──────────────────────────────────────────────
struct PIDState {
  float integral;
  float prevError;
  unsigned long lastTime;
};
PIDState stRoll = {}, stPitch = {}, stYaw = {};

// ─── Low-pass filter for accel ──────────────────────────────
struct LPF { float val; float alpha; };
LPF lpAx = {0, 0.1f}, lpAy = {0, 0.1f}, lpAz = {0, 0.1f};
LPF lpGx = {0, 0.5f}, lpGy = {0, 0.5f}, lpGz = {0, 0.5f};  // lighter → less phase lag

inline float lpfUpdate(LPF &f, float raw) {
  f.val = f.alpha * raw + (1.0f - f.alpha) * f.val;
  return f.val;
}

WebServer server(80);

// ============================================================
//  ESC / Motor helpers
// ============================================================
int motorPins[4] = {M1_PIN, M2_PIN, M3_PIN, M4_PIN};
int motorCh[4]   = {0, 1, 2, 3};   // LEDC channels

void escInit() {
  for (int i = 0; i < 4; i++) {
    ledcAttach(motorPins[i], PWM_FREQ, PWM_RESOLUTION);
    ledcWrite(motorPins[i], ESC_STOP);
  }
}

void setMotorRaw(int idx, int val) {
  val = constrain(val, ESC_STOP, ESC_MAX);
  ledcWrite(motorPins[idx], val);
}

void stopAllMotors() {
  for (int i = 0; i < 4; i++) ledcWrite(motorPins[i], ESC_STOP);
}

void resetPID() {
  stRoll.integral = stPitch.integral = stYaw.integral = 0;
  stRoll.prevError = stPitch.prevError = stYaw.prevError = 0;
}

// ESCs are already initialized (stop signal held) at boot, so runtime
// arm/disarm just gates whether the control task drives throttle.
void armESC() {
  resetPID();
  lastDriveMs = millis();   // start with a fresh failsafe window
  armed = true;
  Serial.println("[ESC] Armed — motors at idle.");
}

void disarmESC() {
  armed = false;
  stopAllMotors();
  Serial.println("[ESC] Disarmed.");
}

// ============================================================
//  controlUpdate(dt)  — runs at LOOP_HZ inside controlTask.
//  Reads latest setpoints, computes PID, mixes to motors.
//  Setpoint convention (from /drive):
//    spThrottle 0..+1 (up)   spYaw -1..+1   spPitch -1..+1 (fwd)   spRoll -1..+1 (right)
// ============================================================
void controlUpdate(float dt) {
  if (!armed) { stopAllMotors(); resetPID(); return; }

  // Snapshot volatile setpoints once.
  float thr = spThrottle, sRoll = spRoll, sPitch = spPitch, sYaw = spYaw;

  // ── Link-loss failsafe: level & settle to idle ──────────
  if (millis() - lastDriveMs > FAILSAFE_MS) {
    thr = 0; sRoll = 0; sPitch = 0; sYaw = 0;
  }

  float throttle01 = constrain(thr, 0.0f, 1.0f);
  float tBase = ESC_IDLE + throttle01 * (ESC_MAX - ESC_IDLE);

  // ── PID correction ──────────────────────────────────────
  float cRoll, cPitch, cYaw;
  if (flightMode == ANGLE_MODE) {
    // Setpoints in degrees; D damps on the measured angular rate (gyro).
    float spRollDeg  = sRoll  * 25.0f;   // max ±25°
    float spPitchDeg = sPitch * 25.0f;
    cRoll  = computePID(stRoll,  pidRoll,  spRollDeg  - imu.roll,  imu.gx, dt);
    cPitch = computePID(stPitch, pidPitch, spPitchDeg - imu.pitch, imu.gy, dt);
    cYaw   = computePID(stYaw,   pidYaw,   sYaw * 90.0f - imu.gz,   0.0f,   dt);
  } else {
    // ACRO: setpoints are target rates (deg/s).
    float spRollRate  = sRoll  * 200.0f;
    float spPitchRate = sPitch * 200.0f;
    float spYawRate   = sYaw   * 150.0f;
    cRoll  = computePID(stRoll,  pidRoll,  spRollRate  - imu.gx, 0.0f, dt);
    cPitch = computePID(stPitch, pidPitch, spPitchRate - imu.gy, 0.0f, dt);
    cYaw   = computePID(stYaw,   pidYaw,   spYawRate   - imu.gz, 0.0f, dt);
  }

  // Per-axis sign trims (set via #define after bench test).
  cRoll *= ROLL_SIGN;  cPitch *= PITCH_SIGN;  cYaw *= YAW_SIGN;

  // Avoid integral wind-up while sitting at idle (props not lifting).
  if (throttle01 < 0.05f) resetPID();

  // ── X-frame mixing ──────────────────────────────────────
  // Motor spin direction:
  //   M1 BL CCW, M2 FL CW, M3 BR CW, M4 FR CCW
  //
  //  Roll(R+)  -1 -1 +1 +1   (right motors up)
  //  Pitch(F+) +1 -1 +1 -1   (rear motors up when pitching forward)
  //  Yaw       +1 -1 -1 +1   (CCW motors increase for CW yaw)
  float m1 = tBase + cPitch - cRoll + cYaw; // BL CCW
  float m2 = tBase - cPitch - cRoll - cYaw; // FL CW
  float m3 = tBase + cPitch + cRoll - cYaw; // BR CW
  float m4 = tBase - cPitch + cRoll + cYaw; // FR CCW

  setMotorRaw(0, (int)m1);
  setMotorRaw(1, (int)m2);
  setMotorRaw(2, (int)m3);
  setMotorRaw(3, (int)m4);

  lastM[0] = m1; lastM[1] = m2; lastM[2] = m3; lastM[3] = m4;
}

// ============================================================
//  PID compute  (returns correction in ESC units, clamped)
//  measRate = rate of the *measured* variable (deg/s).  We damp on
//  it (derivative-on-measurement) instead of d(error)/dt to avoid
//  the "derivative kick" when the stick / setpoint jumps.
// ============================================================
float computePID(PIDState &st, PIDGains &g, float error, float measRate, float dt) {
  st.integral += error * dt;
  st.integral  = constrain(st.integral, -I_LIMIT, I_LIMIT); // anti-windup

  float out = g.kp * error + g.ki * st.integral - g.kd * measRate;
  return constrain(out, -PID_LIMIT, PID_LIMIT);
}

// ============================================================
//  MPU6050
// ============================================================
void mpuInit() {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); Wire.write(0);  // wake
  Wire.endTransmission(true);
  // DLPF bandwidth 44Hz (reg 0x1A, val 3) — helps with vibration
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1A); Wire.write(3);
  Wire.endTransmission(true);
  // Gyro ±500 deg/s (reg 0x1B, val 0x08)
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x1B); Wire.write(0x08);
  Wire.endTransmission(true);
}

// Raw burst read → engineering units (g and deg/s). No bias/filter.
void mpuReadRaw(float &ax, float &ay, float &az,
                float &gx, float &gy, float &gz) {
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, (uint8_t)14, true);

  int16_t rAx = Wire.read()<<8|Wire.read();
  int16_t rAy = Wire.read()<<8|Wire.read();
  int16_t rAz = Wire.read()<<8|Wire.read();
  Wire.read(); Wire.read();  // temp skip
  int16_t rGx = Wire.read()<<8|Wire.read();
  int16_t rGy = Wire.read()<<8|Wire.read();
  int16_t rGz = Wire.read()<<8|Wire.read();

  ax = (float)rAx / 16384.0f;
  ay = (float)rAy / 16384.0f;
  az = (float)rAz / 16384.0f;
  gx = (float)rGx / 65.5f;   // ±500 deg/s sensitivity
  gy = (float)rGy / 65.5f;
  gz = (float)rGz / 65.5f;
}

// Average gyro at rest to find bias. Craft MUST be still.
void calibrateGyro() {
  Serial.println("[IMU] Calibrating gyro — keep still ~2s...");
  const int N = 1000;
  float sx = 0, sy = 0, sz = 0, ax, ay, az, gx, gy, gz;
  for (int i = 0; i < N; i++) {
    mpuReadRaw(ax, ay, az, gx, gy, gz);
    sx += gx; sy += gy; sz += gz;
    delay(2);
  }
  gxBias = sx / N; gyBias = sy / N; gzBias = sz / N;
  Serial.printf("[IMU] Gyro bias: gx=%.2f gy=%.2f gz=%.2f deg/s\n",
                gxBias, gyBias, gzBias);
}

void mpuRead() {
  float rawAx, rawAy, rawAz, rawGx, rawGy, rawGz;
  mpuReadRaw(rawAx, rawAy, rawAz, rawGx, rawGy, rawGz);
  rawGx -= gxBias; rawGy -= gyBias; rawGz -= gzBias;

  // Software LPF (vibration rejection on top of the MPU's 44Hz DLPF)
  imu.ax = lpfUpdate(lpAx, rawAx);
  imu.ay = lpfUpdate(lpAy, rawAy);
  imu.az = lpfUpdate(lpAz, rawAz);
  imu.gx = lpfUpdate(lpGx, rawGx);
  imu.gy = lpfUpdate(lpGy, rawGy);
  imu.gz = lpfUpdate(lpGz, rawGz);

  // Complementary filter  (alpha = 0.98 → trust gyro, small accel correction)
  static unsigned long lastUs = 0;
  float dt = lastUs ? (micros() - lastUs) / 1e6f : (1.0f / LOOP_HZ);
  lastUs = micros();
  dt = constrain(dt, 0.0005f, 0.02f);

  float accRoll  = atan2f(imu.ay, imu.az) * 57.2958f;
  float accPitch = atan2f(-imu.ax, sqrtf(imu.ay*imu.ay + imu.az*imu.az)) * 57.2958f;

  imu.roll  = 0.98f * (imu.roll  + imu.gx * dt) + 0.02f * accRoll;
  imu.pitch = 0.98f * (imu.pitch + imu.gy * dt) + 0.02f * accPitch;
}

// ============================================================
//  Control task — fixed LOOP_HZ, pinned to a core, independent
//  of WiFi.  IMU → attitude → PID → motors every tick.
// ============================================================
void controlTask(void *pv) {
  TickType_t lastWake = xTaskGetTickCount();
  uint32_t lastUs = micros();
  for (;;) {
    uint32_t now = micros();
    float dt = (now - lastUs) / 1e6f;
    lastUs = now;
    dt = constrain(dt, 0.0005f, 0.02f);

    mpuRead();
    controlUpdate(dt);

    vTaskDelayUntil(&lastWake, LOOP_PERIOD);
  }
}

// ============================================================
//  Web UI  (single HTML page, served from flash)
// ============================================================
const char INDEX_HTML[] PROGMEM = R"rawhtml(
<!DOCTYPE html><html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1,user-scalable=no">
<title>DroneFC</title>
<style>
  :root{
    --bg:#0a0c10;--surface:#12161e;--accent:#00e5ff;--warn:#ff4d4d;
    --text:#e8ecf0;--sub:#6b7a8d;--rad:14px;
  }
  *{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}
  body{background:var(--bg);color:var(--text);font-family:'Courier New',monospace;
       display:flex;flex-direction:column;align-items:center;min-height:100vh;padding:12px;gap:12px;}
  h1{font-size:1.1rem;letter-spacing:.2em;color:var(--accent);text-transform:uppercase;margin-top:4px;}

  /* Status bar */
  #statusBar{display:flex;gap:10px;align-items:center;flex-wrap:wrap;justify-content:center;}
  .pill{padding:4px 12px;border-radius:99px;font-size:.7rem;letter-spacing:.1em;
        border:1px solid;text-transform:uppercase;}
  #armPill{border-color:var(--warn);color:var(--warn);}
  #armPill.on{border-color:#00e676;color:#00e676;}
  #modePill{border-color:var(--accent);color:var(--accent);}

  /* Buttons */
  .btn{padding:8px 20px;border-radius:8px;border:none;font-family:inherit;
       font-size:.8rem;letter-spacing:.1em;cursor:pointer;text-transform:uppercase;transition:.15s;}
  #btnArm{background:var(--warn);color:#fff;}
  #btnArm.on{background:#00e676;color:#000;}
  #btnMode{background:var(--surface);color:var(--accent);border:1px solid var(--accent);}

  /* Joysticks */
  #joysticks{display:flex;gap:24px;justify-content:center;flex-wrap:wrap;}
  .jContainer{display:flex;flex-direction:column;align-items:center;gap:6px;}
  .jLabel{font-size:.65rem;color:var(--sub);letter-spacing:.15em;}
  canvas{border-radius:50%;background:var(--surface);border:2px solid #1e2530;touch-action:none;}

  /* Telemetry */
  #telem{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;width:100%;max-width:420px;}
  .tBox{background:var(--surface);border-radius:var(--rad);padding:10px;text-align:center;}
  .tVal{font-size:1.1rem;color:var(--accent);}
  .tLbl{font-size:.6rem;color:var(--sub);letter-spacing:.1em;margin-top:2px;}

  /* PID panel */
  #pidPanel{width:100%;max-width:420px;background:var(--surface);border-radius:var(--rad);padding:12px;}
  #pidPanel summary{cursor:pointer;font-size:.75rem;letter-spacing:.15em;color:var(--sub);
                    text-transform:uppercase;outline:none;}
  #pidPanel summary:hover{color:var(--accent);}
  .pidGrid{display:grid;grid-template-columns:repeat(3,1fr);gap:8px;margin-top:10px;}
  .pidGroup label{font-size:.6rem;color:var(--sub);display:block;margin-bottom:3px;letter-spacing:.08em;}
  .pidGroup input{width:100%;background:#0a0c10;border:1px solid #1e2530;border-radius:6px;
                  color:var(--text);font-family:inherit;font-size:.85rem;padding:4px 6px;text-align:center;}
  .pidGroup input:focus{outline:none;border-color:var(--accent);}
  #btnPID{margin-top:10px;width:100%;background:var(--accent);color:#000;font-weight:bold;}

  /* Log */
  #log{width:100%;max-width:420px;background:var(--surface);border-radius:var(--rad);
       padding:10px;font-size:.65rem;color:var(--sub);height:60px;overflow-y:auto;}
</style>
</head>
<body>
<h1>⬡ DroneFC</h1>

<div id="statusBar">
  <span class="pill" id="armPill">DISARMED</span>
  <span class="pill" id="modePill">ANGLE</span>
  <button class="btn" id="btnArm" onclick="toggleArm()">ARM</button>
  <button class="btn" id="btnMode" onclick="toggleMode()">MODE</button>
</div>

<div id="joysticks">
  <div class="jContainer">
    <div class="jLabelUp" style="font-size:.6rem;color:var(--sub)">THROTTLE / YAW</div>
    <canvas id="jLeft" width="150" height="150"></canvas>
    <div class="jLabel">L-STICK</div>
  </div>
  <div class="jContainer">
    <div class="jLabelUp" style="font-size:.6rem;color:var(--sub)">PITCH / ROLL</div>
    <canvas id="jRight" width="150" height="150"></canvas>
    <div class="jLabel">R-STICK</div>
  </div>
</div>

<div id="telem">
  <div class="tBox"><div class="tVal" id="tRoll">0.0°</div><div class="tLbl">ROLL</div></div>
  <div class="tBox"><div class="tVal" id="tPitch">0.0°</div><div class="tLbl">PITCH</div></div>
  <div class="tBox"><div class="tVal" id="tYawR">0°/s</div><div class="tLbl">YAW RATE</div></div>
</div>

<details id="pidPanel">
  <summary>⚙ PID Tuning</summary>
  <div class="pidGrid">
    <div class="pidGroup">
      <label>ROLL Kp</label><input id="rKp" type="number" step="0.01" value="1.2">
      <label>ROLL Ki</label><input id="rKi" type="number" step="0.01" value="0.05">
      <label>ROLL Kd</label><input id="rKd" type="number" step="0.01" value="0.08">
    </div>
    <div class="pidGroup">
      <label>PITCH Kp</label><input id="pKp" type="number" step="0.01" value="1.2">
      <label>PITCH Ki</label><input id="pKi" type="number" step="0.01" value="0.05">
      <label>PITCH Kd</label><input id="pKd" type="number" step="0.01" value="0.08">
    </div>
    <div class="pidGroup">
      <label>YAW Kp</label><input id="yKp" type="number" step="0.01" value="2.0">
      <label>YAW Ki</label><input id="yKi" type="number" step="0.01" value="0.1">
      <label>YAW Kd</label><input id="yKd" type="number" step="0.01" value="0.0">
    </div>
  </div>
  <button class="btn btnPID" id="btnPID" onclick="sendPID()" style="margin-top:10px;width:100%;background:var(--accent);color:#000;font-weight:bold;">APPLY PID</button>
</details>

<div id="log">Ready.</div>

<script>
// ── Joystick ─────────────────────────────────────────────────
function makeJoystick(canvasId, sticky) {
  const cv = document.getElementById(canvasId);
  const ctx = cv.getContext('2d');
  const R = cv.width / 2;
  const deadzone = 8;
  let ox = R, oy = R, tx = R, ty = R;
  let active = false;
  let vx = 0, vy = 0;

  function draw() {
    ctx.clearRect(0, 0, cv.width, cv.height);
    // base
    ctx.beginPath(); ctx.arc(R,R,R-4,0,Math.PI*2);
    ctx.fillStyle='#0a0c10'; ctx.fill();
    // crosshair
    ctx.strokeStyle='#1e2530'; ctx.lineWidth=1;
    ctx.beginPath(); ctx.moveTo(R,4); ctx.lineTo(R,cv.height-4);
    ctx.moveTo(4,R); ctx.lineTo(cv.width-4,R); ctx.stroke();
    // rim
    ctx.beginPath(); ctx.arc(R,R,R-4,0,Math.PI*2);
    ctx.strokeStyle='#1e2530'; ctx.lineWidth=2; ctx.stroke();
    // thumb
    const maxR = R - 18;
    const dx = tx-R, dy = ty-R;
    const dist = Math.hypot(dx,dy);
    const cx2 = dist > maxR ? R + dx/dist*maxR : tx;
    const cy2 = dist > maxR ? R + dy/dist*maxR : ty;
    const grad = ctx.createRadialGradient(cx2,cy2,2,cx2,cy2,18);
    grad.addColorStop(0,'#00e5ff'); grad.addColorStop(1,'#006e9e');
    ctx.beginPath(); ctx.arc(cx2,cy2,18,0,Math.PI*2);
    ctx.fillStyle=grad; ctx.fill();
    // compute normalized
    const maxR2 = R - 18;
    vx = Math.max(-1,Math.min(1,(cx2-R)/maxR2));
    vy = Math.max(-1,Math.min(1,(cy2-R)/maxR2));
    if (Math.abs(vx)<deadzone/maxR2) vx=0;
    if (Math.abs(vy)<deadzone/maxR2) vy=0;
  }

  function getPos(e) {
    const rect = cv.getBoundingClientRect();
    const touch = e.touches ? e.touches[0] : e;
    return [touch.clientX - rect.left, touch.clientY - rect.top];
  }

  cv.addEventListener('mousedown', e=>{ active=true; [ox,oy]=[tx,ty]=getPos(e); draw(); });
  cv.addEventListener('touchstart', e=>{ e.preventDefault(); active=true; [ox,oy]=[tx,ty]=getPos(e); draw(); },{passive:false});
  window.addEventListener('mousemove', e=>{ if(active){[tx,ty]=getPos(e); draw();} });
  window.addEventListener('touchmove', e=>{ if(active){e.preventDefault();[tx,ty]=getPos(e); draw();} },{passive:false});
  window.addEventListener('mouseup', ()=>{
    if(!active) return; active=false;
    if(!sticky){tx=R;ty=R;} draw();
  });
  window.addEventListener('touchend', ()=>{
    if(!active) return; active=false;
    if(!sticky){tx=R;ty=R;} draw();
  });

  draw();
  return { get x(){ return vx; }, get y(){ return vy; } };
}

// Left stick: sticky=false so throttle drops when released (safety)
// Set sticky=true if you prefer throttle hold
const jL = makeJoystick('jLeft', false);
const jR = makeJoystick('jRight', false);

// ── Send loop ────────────────────────────────────────────────
let isArmed = false;
let mode = 'ANGLE';

function toggleArm(){
  const cmd = isArmed ? 'disarm' : 'arm';
  fetch('/cmd?action='+cmd).then(r=>r.text()).then(t=>log(t));
  isArmed = !isArmed;
  document.getElementById('armPill').textContent = isArmed?'ARMED':'DISARMED';
  document.getElementById('armPill').className = isArmed?'pill on':'pill';
  document.getElementById('btnArm').textContent = isArmed?'DISARM':'ARM';
  document.getElementById('btnArm').className = isArmed?'btn on':'btn';
}

function toggleMode(){
  mode = mode==='ANGLE'?'ACRO':'ANGLE';
  fetch('/cmd?action=mode&val='+mode).then(r=>r.text()).then(t=>log(t));
  document.getElementById('modePill').textContent = mode;
}

function sendPID(){
  const p = {
    rKp:+document.getElementById('rKp').value,
    rKi:+document.getElementById('rKi').value,
    rKd:+document.getElementById('rKd').value,
    pKp:+document.getElementById('pKp').value,
    pKi:+document.getElementById('pKi').value,
    pKd:+document.getElementById('pKd').value,
    yKp:+document.getElementById('yKp').value,
    yKi:+document.getElementById('yKi').value,
    yKd:+document.getElementById('yKd').value,
  };
  fetch('/pid',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(p)})
    .then(r=>r.text()).then(t=>log(t));
}

function log(msg){
  const el=document.getElementById('log');
  el.textContent=new Date().toLocaleTimeString()+': '+msg+'\n'+el.textContent;
}

// Drive loop: 20Hz
setInterval(()=>{
  if(!isArmed) return;
  const LX=jL.x, LY=-jL.y;   // Y inverted (up = positive)
  const RX=jR.x, RY=-jR.y;
  fetch(`/drive?LY=${LY.toFixed(3)}&LX=${LX.toFixed(3)}&RY=${RY.toFixed(3)}&RX=${RX.toFixed(3)}`);
}, 50);

// Telemetry: 5Hz
setInterval(()=>{
  fetch('/telem').then(r=>r.json()).then(d=>{
    document.getElementById('tRoll').textContent = d.roll.toFixed(1)+'°';
    document.getElementById('tPitch').textContent = d.pitch.toFixed(1)+'°';
    document.getElementById('tYawR').textContent = d.gz.toFixed(0)+'°/s';
  }).catch(()=>{});
},200);
</script>
</body></html>
)rawhtml";

// ============================================================
//  Web Server routes
// ============================================================
void setupServer() {
  server.on("/", HTTP_GET, [](){
    server.send_P(200, "text/html", INDEX_HTML);
  });

  server.on("/cmd", HTTP_GET, [](){
    String action = server.arg("action");
    String val    = server.arg("val");
    if (action == "arm")    { armESC();    server.send(200,"text/plain","Armed"); }
    else if (action == "disarm") { disarmESC(); server.send(200,"text/plain","Disarmed"); }
    else if (action == "mode") {
      flightMode = (val == "ACRO") ? ACRO_MODE : ANGLE_MODE;
      server.send(200,"text/plain", val == "ACRO" ? "Acro mode" : "Angle mode");
    } else {
      server.send(400,"text/plain","Unknown command");
    }
  });

  // Only stores setpoints + a timestamp; the control task does the flying.
  server.on("/drive", HTTP_GET, [](){
    spThrottle  = server.arg("LY").toFloat();   // throttle
    spYaw       = server.arg("LX").toFloat();   // yaw
    spPitch     = server.arg("RY").toFloat();   // pitch
    spRoll      = server.arg("RX").toFloat();   // roll
    lastDriveMs = millis();                     // pet the failsafe
    server.send(200,"text/plain","ok");
  });

  server.on("/telem", HTTP_GET, [](){
    StaticJsonDocument<192> doc;
    doc["roll"]  = imu.roll;
    doc["pitch"] = imu.pitch;
    doc["gz"]    = imu.gz;
    doc["armed"] = armed;
    doc["m1"]    = (int)lastM[0];
    doc["m2"]    = (int)lastM[1];
    doc["m3"]    = (int)lastM[2];
    doc["m4"]    = (int)lastM[3];
    String out;
    serializeJson(doc, out);
    server.send(200,"application/json", out);
  });

  server.on("/pid", HTTP_POST, [](){
    if (!server.hasArg("plain")) { server.send(400,"text/plain","No body"); return; }
    StaticJsonDocument<256> doc;
    deserializeJson(doc, server.arg("plain"));
    pidRoll.kp  = doc["rKp"]; pidRoll.ki  = doc["rKi"]; pidRoll.kd  = doc["rKd"];
    pidPitch.kp = doc["pKp"]; pidPitch.ki = doc["pKi"]; pidPitch.kd = doc["pKd"];
    pidYaw.kp   = doc["yKp"]; pidYaw.ki   = doc["yKi"]; pidYaw.kd   = doc["yKd"];
    // Reset integrators on PID change
    stRoll.integral = stPitch.integral = stYaw.integral = 0;
    server.send(200,"text/plain","PID updated");
    Serial.printf("[PID] R(%.2f,%.2f,%.2f) P(%.2f,%.2f,%.2f) Y(%.2f,%.2f,%.2f)\n",
      pidRoll.kp, pidRoll.ki, pidRoll.kd,
      pidPitch.kp,pidPitch.ki,pidPitch.kd,
      pidYaw.kp,  pidYaw.ki,  pidYaw.kd);
  });

  server.begin();
  Serial.println("[WiFi] Server started.");
}

// ============================================================
//  Setup & Loop
// ============================================================
void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== DroneFC Boot ===");

  // I2C
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  mpuInit();
  Serial.println("[IMU] MPU6050 init OK");

  // Gyro bias — craft must be still on the ground here.
  calibrateGyro();

  // ESC init — hold stop signal so ESCs arm during boot.
  escInit();
  Serial.println("[ESC] Stop signal sent, waiting 2s for ESC boot...");
  delay(ESC_ARM_DELAY);
  Serial.println("[ESC] ESC ready. Motors stay OFF until 'arm' command.");

  // WiFi AP
  WiFi.softAP(AP_SSID, AP_PASS);
  Serial.print("[WiFi] AP IP: ");
  Serial.println(WiFi.softAPIP());

  setupServer();

  // Stabilization runs on its own fixed-rate task (core 1, high prio),
  // so WiFi hiccups can never stall the control loop.
  xTaskCreatePinnedToCore(controlTask, "ctrl", 4096, NULL, 5, NULL, 1);
  Serial.printf("[CTRL] Control task started @ %dHz\n", LOOP_HZ);
}

void loop() {
  server.handleClient();

  // ~10Hz bench readout — watch these during the props-off tilt test.
  static uint32_t t = 0;
  if (millis() - t >= 100) {
    t = millis();
    Serial.printf("R%+6.1f P%+6.1f Yz%+6.1f | M %3.0f %3.0f %3.0f %3.0f | %s\n",
                  imu.roll, imu.pitch, imu.gz,
                  lastM[0], lastM[1], lastM[2], lastM[3],
                  armed ? "ARM" : "---");
  }
  delay(1);
}

/*
What I rewrote in App/APFlightController.ino
The old code stabilized the drone only when a WiFi packet arrived (~30Hz, with stalls up to 624ms in your logs). That's the #1 reason it "doesn't correct fast enough." The rewrite fixes the four ranked root causes:

Fixed-rate control loop, decoupled from WiFi — stabilization now runs in a 500Hz FreeRTOS task (controlTask, pinned to core 1, high priority) that reads the IMU, runs PID, and drives motors every 2ms no matter what the radio is doing. The web handlers now only write setpoints. This is the big one.
Gyro calibration at boot (calibrateGyro) — averages 1000 samples while still and subtracts the bias your logs showed (1–5°/s), so the angle estimate and yaw stop drifting. The drone must be sitting still for the first ~2s after power-on.
Armed idle + link-loss failsafe — motors hold a safe idle (ESC_IDLE) while armed instead of cutting to zero on a stick blip; if no /drive packet arrives within 400ms it auto-levels and settles to idle. Full stop only on disarm.
PID cleanup — derivative-on-measurement (no more "derivative kick" on stick moves), output clamps, and idle anti-windup.
I also added ROLL_SIGN/PITCH_SIGN/YAW_SIGN trim macros and put the four motor values into /telem and the serial output, so you can verify signs on the bench.

Structure checks pass (braces/parens balanced, no dangling references). I couldn't compile — there's no Arduino toolchain on this machine — so step 1 is to open it in the Arduino IDE (ESP32-S3) and Verify.

⚠️ Bench test BEFORE flying — props OFF
Watch the serial monitor @115200 (streams R<roll> P<pitch> Yz<yawrate> | M m1 m2 m3 m4). Arm, give a little throttle so all four spin, then:

Tilt right → the two left motors (M1 BL, M2 FL) should speed up. If not, flip ROLL_SIGN to (-1).
Tilt nose-down → the two rear motors (M1 BL, M3 BR) speed up. If not, flip PITCH_SIGN.
Yaw the frame → the correct diagonal pair responds. If not, flip YAW_SIGN.
Confirm all four reliably spin at idle — your sandbox ESC test needed ~65/1024, but I set ESC_IDLE=54 as a starting point. If any motor doesn't spin, raise ESC_IDLE.
One extra thing to watch: if tilting roll makes the pitch number move (or vice-versa), your gyro is physically mounted with X/Y swapped — that's a code fix (swap gx/gy), not just a sign flip. Tell me and I'll handle it.

Then test the failsafe (arm + throttle, kill WiFi → motors should drop to idle within ~0.4s), and only then do a first low hover in ANGLE mode in open space. If it oscillates, lower the P gains; if it's sluggish, raise them.

Want me to also add an explicit "motor test" web button (spin one motor at a time) to make the props-off checkout easier, or is the serial readout enough?
*/
