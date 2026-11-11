// Mini Soccer Robot - ESP32 + 1x L298N, direct WiFi (AP) + WebSocket joystick
// 2 motors, one motor per L298N channel (2WD, left motor / right motor)
// Arduino IDE: board "ESP32 Dev Module" (classic ESP32), ESP32 core 3.x
// Library needed: "WebSockets" by Markus Sattler (arduinoWebSockets)
// Put this file in a folder named "minisoccer_l298n".

#include <WiFi.h>
#include <WebServer.h>
#include <WebSocketsServer.h>

// ---------------- WiFi ----------------
const char*   AP_SSID        = "MiniSoccer";
const char*   AP_PASS        = "minisoccer1971";
const uint8_t AP_CHANNEL     = 1;      // use 1, 6 or 11; pick the emptiest one at the venue
const uint8_t AP_MAX_CLIENTS = 2;

// ---------------- Pins ----------------
// Single L298N module: channel A = left motor, channel B = right motor
struct Motor { int en; int in1; int in2; bool invert; };

// Pins are classic ESP32 DevKit GPIO numbers (D13 = GPIO 13, etc.)
// NOTE: GPIO12 is a boot-strapping pin. If the ESP32 won't boot/upload, unplug the IN1 wire first.
Motor leftMotor  = { 25, 27, 26, true };   // left : ENB=D25, IN3=D27, IN4=D26 (channel B)
Motor rightMotor = { 13, 12, 14, true };   // right: ENA=D13, IN1=D12, IN2=D14 (channel A)
// If a motor spins the wrong way, set its invert flag to true.

// ---------------- PWM ----------------
const uint32_t PWM_FREQ = 4000;                // L298N is slow: 20 kHz weakens low duty cycles (big dead zone). 2000-5000 works best
const uint8_t  PWM_BITS = 10;
const int      PWM_MAX  = (1 << PWM_BITS) - 1; // 1023

// ---------------- Tuning ----------------
// 1.0 = 100% PWM. The L298N drops about 1.5-2 V and is limited to 2 A per channel,
// so at 100% the motors and drivers run hot. Lower it (e.g. 0.7) if they overheat.
const float MAX_POWER  = 1.0f;
const float DEADZONE   = 0.0f;
const float EXPO       = 0.0f;   // 0 = fully linear
const float TURN_GAIN  = 0.85f;
// Smallest motor power that actually moves the robot (gearbox friction + L298N voltage drop).
// Any stick movement starts at this level and scales up to 100%. Raise it if small movements
// still do nothing, lower it if the robot lurches at the slightest touch.
const float MIN_START  = 0.55f;

const uint32_t FAILSAFE_MS = 250;
const uint32_t UPDATE_MS   = 5;
const int ACCEL_STEP = 300;       // per 5 ms update: full power in ~17 ms (near-instant). Lower it (e.g. 100) for a softer start
const int DECEL_STEP = 300;
const uint32_t BRAKE_MS = 50;     // hold at zero (brake) when the direction reverses

// ---------------- State ----------------
// Per-side state for the reversal brake (must be declared before any function uses it,
// otherwise the Arduino auto-prototype fails with "'Side' was not declared")
struct Side { int cur = 0; int lastDir = 0; uint32_t holdUntil = 0; };
Side sideL, sideR;

WebServer server(80);
WebSocketsServer ws(81);

volatile float cmdX = 0, cmdY = 0;
volatile uint32_t lastCmdMs = 0;
int curL = 0, curR = 0;
uint32_t lastUpdate = 0;

// ---------------- Web page ----------------
const char PAGE[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>DURGO</title>
<style>
html,body{margin:0;height:100%;background:#000;overflow:hidden;touch-action:none;user-select:none;-webkit-user-select:none;font-family:system-ui,-apple-system,"Segoe UI",Roboto,sans-serif}
#app{position:fixed;left:0;top:0;transform-origin:0 0;background:#000}
#title{position:absolute;top:12px;left:0;right:0;text-align:center;font-size:32px;font-weight:800;line-height:1;letter-spacing:.4em;padding-left:.4em;color:#39ff14;text-shadow:0 0 6px #39ff14,0 0 18px #39ff14,0 0 40px #1fcf0a}
#st{position:absolute;top:16px;right:18px;width:10px;height:10px;border-radius:50%;background:#ff3b3b}
#st.ok{background:#39ff14;box-shadow:0 0 8px #39ff14}
#pads{position:absolute;left:0;right:0;top:60px;bottom:0;display:flex;align-items:center;justify-content:space-between;padding:0 5%}
.track{position:relative;border:1.5px solid #164016;border-radius:60px;background:#030803;touch-action:none}
.track:before{content:"";position:absolute;background:#0f2a0f}
#thr{width:96px;height:min(calc(var(--H) - 100px),380px)}
#thr:before{left:20%;right:20%;top:50%;height:1px}
#str{width:min(calc(var(--W) * .46),440px);height:96px}
#str:before{top:20%;bottom:20%;left:50%;width:1px}
.knob{position:absolute;left:50%;top:50%;width:72px;height:72px;border-radius:50%;background:#39ff14;opacity:.8;transform:translate(-50%,-50%);box-shadow:0 0 14px rgba(57,255,20,.5),0 0 34px rgba(57,255,20,.25);pointer-events:none}
.knob.on{opacity:1;box-shadow:0 0 20px rgba(57,255,20,.8),0 0 50px rgba(57,255,20,.4)}
</style></head><body>
<div id="app">
  <div id="title">DURGO</div>
  <div id="st"></div>
  <div id="pads">
    <div id="thr" class="track"><div class="knob"></div></div>
    <div id="str" class="track"><div class="knob"></div></div>
  </div>
</div>
<script>
const app=document.getElementById('app'),st=document.getElementById('st');
let thr=0,str=0,sock,portrait=false;
// The page is always laid out as landscape; in portrait it is rotated 90 degrees.
function layout(){
  const w=innerWidth,h=innerHeight;portrait=h>w;
  const W=portrait?h:w,H=portrait?w:h;
  app.style.width=W+'px';app.style.height=H+'px';
  app.style.setProperty('--W',W+'px');app.style.setProperty('--H',H+'px');
  app.style.transform=portrait?'rotate(90deg) translateY(-100%)':'none';
}
function toLocal(x,y){return portrait?{x:y,y:innerWidth-x}:{x:x,y:y};}
addEventListener('resize',layout);addEventListener('orientationchange',layout);layout();
// best effort: fullscreen + landscape lock on the first touch (ignored where unsupported)
document.addEventListener('pointerdown',()=>{
  const d=document.documentElement;
  if(!document.fullscreenElement&&d.requestFullscreen){
    d.requestFullscreen().then(()=>{try{screen.orientation.lock('landscape').catch(()=>{});}catch(e){}}).catch(()=>{});
  }
},{once:true});

function pad(el,vert){
  const knob=el.querySelector('.knob'),R=36;let id=null;
  function set(v){
    const half=(vert?el.clientHeight:el.clientWidth)/2-R;
    knob.style.transform=vert?'translate(-50%,-50%) translateY('+(-v*half)+'px)':'translate(-50%,-50%) translateX('+(v*half)+'px)';
    if(vert)thr=v;else str=v;
    send();
  }
  function upd(e){
    const r=el.getBoundingClientRect();
    const c=toLocal(r.left+r.width/2,r.top+r.height/2),p=toLocal(e.clientX,e.clientY);
    let v;
    if(vert)v=(c.y-p.y)/(el.offsetHeight/2-R);
    else v=(p.x-c.x)/(el.offsetWidth/2-R);
    set(Math.max(-1,Math.min(1,v)));
  }
  function release(){id=null;knob.classList.remove('on');set(0);}
  el.addEventListener('pointerdown',e=>{id=e.pointerId;el.setPointerCapture(id);knob.classList.add('on');upd(e);});
  el.addEventListener('pointermove',e=>{if(e.pointerId===id)upd(e);});
  el.addEventListener('pointerup',e=>{if(e.pointerId===id)release();});
  el.addEventListener('pointercancel',e=>{if(e.pointerId===id)release();});
  el.addEventListener('lostpointercapture',e=>{if(e.pointerId===id)release();});
  el.addEventListener('contextmenu',e=>e.preventDefault());
  return release;
}
const relThr=pad(document.getElementById('thr'),true);
const relStr=pad(document.getElementById('str'),false);
function stopAll(){relThr();relStr();}
addEventListener('blur',stopAll);
document.addEventListener('visibilitychange',()=>{if(document.hidden)stopAll();});
function connect(){
  sock=new WebSocket('ws://'+location.hostname+':81/');
  sock.onopen=()=>{st.className='ok';};
  sock.onclose=()=>{st.className='';setTimeout(connect,400);};
  sock.onerror=()=>sock.close();
}
// constant ~30 Hz stream doubles as heartbeat for the failsafe: "steering,throttle"
function send(){
  if(sock&&sock.readyState===1)sock.send(Math.round(str*100)+','+Math.round(thr*100));
}
setInterval(send,16);
connect();
</script></body></html>
)HTML";

// ---------------- Helpers ----------------
static inline float clampf(float v, float lo, float hi) { return v < lo ? lo : (v > hi ? hi : v); }

float shape(float v) {
  float a = fabsf(v);
  if (a < DEADZONE) return 0;
  a = (a - DEADZONE) / (1.0f - DEADZONE);
  a = EXPO * a * a * a + (1.0f - EXPO) * a;
  return v < 0 ? -a : a;
}

// stick value -1..1 -> motor duty, skipping the range where the motors don't move
int toDuty(float v) {
  float a = fabsf(clampf(v, -1, 1));
  if (a < 0.001f) return 0;
  int d = (int)lroundf((MIN_START + (1.0f - MIN_START) * a) * MAX_POWER * PWM_MAX);
  return v < 0 ? -d : d;
}

int ramp(int cur, int tgt) {
  int diff = tgt - cur;
  bool accelerating = abs(tgt) > abs(cur) && (cur == 0 || ((tgt > 0) == (cur > 0)));
  int step = accelerating ? ACCEL_STEP : DECEL_STEP;
  if (diff > step)  return cur + step;
  if (diff < -step) return cur - step;
  return tgt;
}

// Brake on direction change: slow to zero, hold the brake for BRAKE_MS, then go the other way.
// This avoids driving the motors backwards while the wheels still spin forward (current spike).
int stepSide(Side& s, int want, uint32_t now) {
  int dir = (want > 0) - (want < 0);
  int tgt = want;
  if (s.lastDir != 0 && dir != 0 && dir != s.lastDir) {
    tgt = 0;                                   // reversing: stop first
    if (s.cur == 0) {                          // at zero the driver is braking
      if (s.holdUntil == 0) s.holdUntil = now + BRAKE_MS;
      else if ((int32_t)(now - s.holdUntil) >= 0) { s.lastDir = 0; s.holdUntil = 0; tgt = want; }
    }
  } else {
    s.holdUntil = 0;
  }
  s.cur = ramp(s.cur, tgt);
  if (s.cur != 0) s.lastDir = (s.cur > 0) - (s.cur < 0);
  return s.cur;
}

// val: -PWM_MAX..PWM_MAX. Zero = brake (EN high, IN1 = IN2 = low).
void driveMotor(const Motor& m, int val) {
  if (m.invert) val = -val;
  if (val > 0) {
    digitalWrite(m.in1, HIGH); digitalWrite(m.in2, LOW);
    ledcWrite(m.en, val);
  } else if (val < 0) {
    digitalWrite(m.in1, LOW);  digitalWrite(m.in2, HIGH);
    ledcWrite(m.en, -val);
  } else {
    digitalWrite(m.in1, LOW);  digitalWrite(m.in2, LOW);
    ledcWrite(m.en, PWM_MAX);   // brake
  }
}

void onWs(uint8_t num, WStype_t type, uint8_t* payload, size_t len) {
  if (type == WStype_DISCONNECTED) {
    cmdX = 0; cmdY = 0;
  } else if (type == WStype_TEXT) {
    int xi, yi;
    if (sscanf((const char*)payload, "%d,%d", &xi, &yi) == 2) {
      cmdX = clampf(xi / 100.0f, -1, 1);
      cmdY = clampf(yi / 100.0f, -1, 1);
      lastCmdMs = millis();
    }
  }
}

void initMotor(const Motor& m) {
  pinMode(m.in1, OUTPUT);
  pinMode(m.in2, OUTPUT);
  ledcAttach(m.en, PWM_FREQ, PWM_BITS);
  driveMotor(m, 0);
}

// ---------------- Setup / loop ----------------
void setup() {
  Serial.begin(115200);

  initMotor(leftMotor);
  initMotor(rightMotor);

  WiFi.mode(WIFI_AP);
  WiFi.setSleep(false);
  WiFi.softAP(AP_SSID, AP_PASS, AP_CHANNEL, 0, AP_MAX_CLIENTS);
  WiFi.setTxPower(WIFI_POWER_19_5dBm);
  Serial.print("AP up, open http://");
  Serial.println(WiFi.softAPIP());

  server.on("/", []() { server.send_P(200, "text/html", PAGE); });
  server.begin();
  ws.begin();
  ws.onEvent(onWs);
}

void loop() {
  ws.loop();
  server.handleClient();

  uint32_t now = millis();
  if (now - lastUpdate < UPDATE_MS) return;
  lastUpdate = now;

  float x = cmdX, y = cmdY;
  if (now - lastCmdMs > FAILSAFE_MS) { x = 0; y = 0; }   // failsafe: brake

  x = shape(x);
  y = shape(y);

  float l = y + x * TURN_GAIN;
  float r = y - x * TURN_GAIN;
  float m = fmaxf(fabsf(l), fabsf(r));
  if (m > 1.0f) { l /= m; r /= m; }

  int tgtL = toDuty(l);
  int tgtR = toDuty(r);

  curL = stepSide(sideL, tgtL, now);
  curR = stepSide(sideR, tgtR, now);

  driveMotor(leftMotor,  curL);
  driveMotor(rightMotor, curR);
}
