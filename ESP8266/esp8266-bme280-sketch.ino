#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include <math.h>
#include <string.h>
#include <stdlib.h>
#include <stddef.h>

#define RELAY_PIN 12
#define BOOT_BTN_PIN 0
#define SDA_PIN 5
#define SCL_PIN 4

#define EEPROM_SIZE 128
#define MAGIC 0xA5B6C7D8UL
#define FW_VERSION 1

#define SENSOR_TICK_MS 1000UL
#define BTN_POLL_MS 50UL
#define BTN_HOLD_MS 5000UL
#define SAVE_DEBOUNCE_MS 60000UL
#define SENSOR_COOLDOWN_MS 30000UL
#define SENSOR_FAIL_THRESHOLD 3

struct Settings {
  uint32_t magic;
  uint32_t version;
  int64_t tempLow;
  int64_t tempHigh;
  int64_t humLow;
  int64_t humHigh;
  int64_t pressLow;
  int64_t pressHigh;
  uint8_t relayMode;
  uint8_t pad[3];
  uint8_t crc;
};

static Settings cfg;
static Adafruit_BME280 bme;
static ESP8266WebServer server(80);
static DNSServer dns;
static bool bmeOk = false;

static bool cfgDirty = false;
static uint32_t cfgDirtySince = 0;
static uint32_t lastSampleMs = 0;
static uint32_t lastBtnMs = 0;
static uint32_t btnDownAtMs = 0;
static bool btnDown = false;
static int64_t lastTemp = 0;
static int64_t lastHum = 0;
static int64_t lastPress = 0;
static bool sensorValid = false;
static uint8_t sensorFailStreak = 0;
static uint32_t sensorCooldownUntil = 0;
static bool sensorCooldown = false;

static uint64_t s_upAccMs = 0;
static uint32_t s_upLast = 0;

static const char AP_SSID[] PROGMEM = "ESP8266_BME280";
static const char AP_PASS[] PROGMEM = "12345678";

static const char PAGE_INDEX[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>BME280 Dashboard</title><style>
*{box-sizing:border-box;font-family:system-ui,sans-serif}body{margin:0;background:#0f172a;color:#e2e8f0}
header{background:#1e293b;padding:16px;display:flex;flex-wrap:wrap;gap:12px;align-items:center;justify-content:space-between}
h1{font-size:18px;margin:0}nav{display:flex;gap:8px;flex-wrap:wrap}
nav a{color:#94a3b8;text-decoration:none;padding:8px 12px;border-radius:6px;background:#334155;font-size:14px}
nav a.active,nav a:hover{background:#3b82f6;color:#fff}
main{padding:16px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}
.card{background:#1e293b;border-radius:10px;padding:16px;border:1px solid #334155}
.card h3{margin:0 0 8px;font-size:13px;color:#94a3b8;text-transform:uppercase;letter-spacing:.5px}
.val{font-size:28px;font-weight:700;color:#38bdf8;font-variant-numeric:tabular-nums}
.unit{font-size:14px;color:#94a3b8;margin-left:4px}
.row{display:flex;align-items:center;gap:8px;margin:10px 0;flex-wrap:wrap}
input,select,button{font-size:14px;padding:10px 12px;border-radius:6px;border:1px solid #475569;background:#0f172a;color:#e2e8f0}
button{background:#3b82f6;border:none;color:#fff;cursor:pointer;font-weight:600;min-height:40px}
button:hover{background:#2563eb}button.danger{background:#ef4444}button.danger:hover{background:#dc2626}
label{font-size:13px;color:#94a3b8;min-width:90px}
.status{display:flex;align-items:center;gap:6px;font-size:14px}
.dot{width:10px;height:10px;border-radius:50%;background:#ef4444}
.dot.on{background:#22c55e}
table{width:100%;border-collapse:collapse}td,th{padding:8px;text-align:left;border-bottom:1px solid #334155;font-size:14px}
.notice{padding:10px;border-radius:6px;background:#1e40af;color:#dbeafe;margin-bottom:12px;display:none}
@media(max-width:600px){.val{font-size:22px}header{padding:12px}h1{font-size:16px}nav a{padding:10px 12px}}
</style></head><body>
<header><h1>BME280 Dashboard</h1><nav>
<a href="/" class="active">Dashboard</a><a href="/config">Thresholds</a><a href="/wifi">WiFi</a>
</nav></header>
<main>
<div id="notice" class="notice"></div>
<div class="grid">
<div class="card"><h3>Temperature</h3><div><span class="val" id="temp">--</span><span class="unit">&deg;C</span></div></div>
<div class="card"><h3>Humidity</h3><div><span class="val" id="hum">--</span><span class="unit">%</span></div></div>
<div class="card"><h3>Pressure</h3><div><span class="val" id="press">--</span><span class="unit">hPa</span></div></div>
<div class="card"><h3>Relay</h3><div class="status"><span class="dot" id="relaydot"></span><span id="relaytxt">OFF</span></div>
<div style="margin-top:8px;font-size:13px;color:#94a3b8" id="relaymode">Auto</div></div>
<div class="card"><h3>Sensor</h3><div class="status"><span class="dot" id="bmedot"></span><span id="bmetxt">--</span></div></div>
<div class="card"><h3>Uptime</h3><div class="val" id="uptime">--</div></div>
<div class="card"><h3>Free Heap</h3><div class="val" id="heap">--</div></div>
<div class="card"><h3>RSSI</h3><div class="val" id="rssi">--</div></div>
</div>
<div class="card" style="margin-top:12px"><h3>Manual Relay</h3>
<div class="row"><button onclick="relayCmd(1)">Force ON</button>
<button onclick="relayCmd(2)">Force OFF</button>
<button class="danger" onclick="relayCmd(0)">Auto</button></div></div>
</main>
<script>
let errCount=0;
function fmt(n,d){if(n===null||n===undefined||isNaN(n))return'--';return Number(n).toFixed(d);}
function poll(){
 fetch('/api/status',{cache:'no-store'}).then(r=>{if(!r.ok)throw new Error();return r.json();}).then(d=>{
  errCount=0;
  document.getElementById('temp').textContent=d.valid?fmt(d.t,2):'--';
  document.getElementById('hum').textContent=d.valid?fmt(d.h,2):'--';
  document.getElementById('press').textContent=d.valid?fmt(d.p,2):'--';
  document.getElementById('uptime').textContent=d.up;
  document.getElementById('heap').textContent=d.heap;
  document.getElementById('rssi').textContent=d.rssi+' dBm';
  const rd=document.getElementById('relaydot'),rt=document.getElementById('relaytxt');
  if(d.relay){rd.classList.add('on');rt.textContent='ON';}else{rd.classList.remove('on');rt.textContent='OFF';}
  document.getElementById('relaymode').textContent=d.mode===0?'Auto':(d.mode===1?'Forced ON':'Forced OFF');
  const bd=document.getElementById('bmedot'),bt=document.getElementById('bmetxt');
  if(d.bme&&d.valid){bd.classList.add('on');bt.textContent='OK';}
  else if(d.bme&&d.cooldown){bd.classList.remove('on');bt.textContent='COOLDOWN';}
  else{bd.classList.remove('on');bt.textContent='ERROR';}
 }).catch(()=>{errCount++;if(errCount>2){show('Connection lost');}});
}
function show(m){const n=document.getElementById('notice');n.textContent=m;n.style.display='block';setTimeout(()=>n.style.display='none',3000);}
function relayCmd(m){fetch('/api/relay?m='+m,{method:'POST'}).then(()=>poll()).catch(()=>show('Relay command failed'));}
poll();setInterval(poll,2000);
document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});
</script></body></html>)HTML";

static const char PAGE_CONFIG[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>Thresholds</title><style>
*{box-sizing:border-box;font-family:system-ui,sans-serif}body{margin:0;background:#0f172a;color:#e2e8f0}
header{background:#1e293b;padding:16px;display:flex;flex-wrap:wrap;gap:12px;align-items:center;justify-content:space-between}
h1{font-size:18px;margin:0}nav{display:flex;gap:8px;flex-wrap:wrap}
nav a{color:#94a3b8;text-decoration:none;padding:8px 12px;border-radius:6px;background:#334155;font-size:14px}
nav a.active,nav a:hover{background:#3b82f6;color:#fff}
main{padding:16px;max-width:800px;margin:0 auto}
.card{background:#1e293b;border-radius:10px;padding:16px;border:1px solid #334155;margin-bottom:12px}
.card h3{margin:0 0 12px;font-size:14px;color:#94a3b8;text-transform:uppercase}
.row{display:flex;align-items:center;gap:8px;margin:10px 0;flex-wrap:wrap}
input,button{font-size:14px;padding:10px 12px;border-radius:6px;border:1px solid #475569;background:#0f172a;color:#e2e8f0}
button{background:#3b82f6;border:none;color:#fff;cursor:pointer;font-weight:600;min-height:40px}button:hover{background:#2563eb}
button.danger{background:#ef4444}label{font-size:14px;min-width:130px}
.notice{padding:10px;border-radius:6px;background:#166534;color:#dcfce7;margin-bottom:12px;display:none}
@media(max-width:600px){header{padding:12px}h1{font-size:16px}nav a{padding:10px 12px}}
</style></head><body>
<header><h1>Thresholds</h1><nav><a href="/">Dashboard</a><a href="/config" class="active">Thresholds</a><a href="/wifi">WiFi</a></nav></header>
<main>
<div id="notice" class="notice"></div>
<div class="card"><h3>Temperature (&deg;C)</h3>
<div class="row"><label>Low</label><input id="tL" type="number" step="1" min="-40" max="85" inputmode="numeric"></div>
<div class="row"><label>High</label><input id="tH" type="number" step="1" min="-40" max="85" inputmode="numeric"></div></div>
<div class="card"><h3>Humidity (%)</h3>
<div class="row"><label>Low</label><input id="hL" type="number" step="1" min="0" max="100" inputmode="numeric"></div>
<div class="row"><label>High</label><input id="hH" type="number" step="1" min="0" max="100" inputmode="numeric"></div></div>
<div class="card"><h3>Pressure (hPa)</h3>
<div class="row"><label>Low</label><input id="pL" type="number" step="1" min="300" max="1100" inputmode="numeric"></div>
<div class="row"><label>High</label><input id="pH" type="number" step="1" min="300" max="1100" inputmode="numeric"></div></div>
<div class="row"><button onclick="save()">Save</button><button class="danger" onclick="reset()">Factory Reset</button></div>
</main>
<script>
function show(m,err){const n=document.getElementById('notice');n.textContent=m;n.style.background=err?'#991b1b':'#166534';n.style.color=err?'#fee2e2':'#dcfce7';n.style.display='block';setTimeout(()=>n.style.display='none',3000);}
function load(){fetch('/api/config',{cache:'no-store'}).then(r=>r.json()).then(d=>{
 document.getElementById('tL').value=d.tL;document.getElementById('tH').value=d.tH;
 document.getElementById('hL').value=d.hL;document.getElementById('hH').value=d.hH;
 document.getElementById('pL').value=d.pL;document.getElementById('pH').value=d.pH;
}).catch(()=>show('Load failed',true));}
load();
function save(){
 const b={tL:+document.getElementById('tL').value,tH:+document.getElementById('tH').value,
 hL:+document.getElementById('hL').value,hH:+document.getElementById('hH').value,
 pL:+document.getElementById('pL').value,pH:+document.getElementById('pH').value};
 if([b.tL,b.tH,b.hL,b.hH,b.pL,b.pH].some(x=>!isFinite(x))){show('Invalid number',true);return;}
 if(b.tL>=b.tH||b.hL>=b.hH||b.pL>=b.pH){show('Low must be < High',true);return;}
 fetch('/api/config',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)})
 .then(r=>{if(!r.ok)throw 0;return r.json();}).then(()=>show('Saved')).catch(()=>show('Save failed',true));
}
function reset(){if(!confirm('Factory reset?'))return;
 fetch('/api/factory',{method:'POST'}).then(()=>show('Reset - rebooting')).catch(()=>show('Reset failed',true));}
document.addEventListener('visibilitychange',()=>{if(!document.hidden)load();});
</script></body></html>)HTML";

static const char PAGE_WIFI[] PROGMEM = R"HTML(<!DOCTYPE html><html><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><title>WiFi</title><style>
*{box-sizing:border-box;font-family:system-ui,sans-serif}body{margin:0;background:#0f172a;color:#e2e8f0}
header{background:#1e293b;padding:16px;display:flex;flex-wrap:wrap;gap:12px;align-items:center;justify-content:space-between}
h1{font-size:18px;margin:0}nav{display:flex;gap:8px;flex-wrap:wrap}
nav a{color:#94a3b8;text-decoration:none;padding:8px 12px;border-radius:6px;background:#334155;font-size:14px}
nav a.active,nav a:hover{background:#3b82f6;color:#fff}
main{padding:16px;max-width:700px;margin:0 auto}
.card{background:#1e293b;border-radius:10px;padding:16px;border:1px solid #334155;margin-bottom:12px}
.card h3{margin:0 0 12px;font-size:14px;color:#94a3b8;text-transform:uppercase}
.row{display:flex;align-items:center;gap:8px;margin:10px 0;flex-wrap:wrap}
input,button{font-size:14px;padding:10px 12px;border-radius:6px;border:1px solid #475569;background:#0f172a;color:#e2e8f0;width:100%}
button{background:#3b82f6;border:none;color:#fff;cursor:pointer;font-weight:600;width:auto;min-height:40px}button:hover{background:#2563eb}
label{font-size:13px;color:#94a3b8}.notice{padding:10px;border-radius:6px;background:#166534;color:#dcfce7;margin-bottom:12px;display:none}
table{width:100%;border-collapse:collapse}td{padding:8px;border-bottom:1px solid #334155;font-size:14px}
@media(max-width:600px){header{padding:12px}h1{font-size:16px}nav a{padding:10px 12px}}
</style></head><body>
<header><h1>WiFi</h1><nav><a href="/">Dashboard</a><a href="/config">Thresholds</a><a href="/wifi" class="active">WiFi</a></nav></header>
<main>
<div id="notice" class="notice"></div>
<div class="card"><h3>AP Info</h3><table>
<tr><td>SSID</td><td id="ssid">--</td></tr>
<tr><td>IP</td><td id="ip">--</td></tr>
<tr><td>Clients</td><td id="clients">--</td></tr></table></div>
<div class="card"><h3>Station Status</h3><table>
<tr><td>State</td><td id="staState">--</td></tr>
<tr><td>IP</td><td id="staIP">--</td></tr></table></div>
<div class="card"><h3>Station Mode</h3>
<div class="row"><label>SSID</label><input id="staS" maxlength="32" autocapitalize="off" autocomplete="off"></div>
<div class="row"><label>Password</label><input id="staP" type="password" maxlength="64" autocomplete="off"></div>
<div class="row"><button onclick="connect()">Connect</button><button onclick="disconnect()" class="danger" style="background:#ef4444">Disconnect</button></div></div>
</main>
<script>
function show(m){const n=document.getElementById('notice');n.textContent=m;n.style.display='block';setTimeout(()=>n.style.display='none',3000);}
function poll(){fetch('/api/wifi',{cache:'no-store'}).then(r=>r.json()).then(d=>{
 document.getElementById('ssid').textContent=d.ssid;document.getElementById('ip').textContent=d.ip;
 document.getElementById('clients').textContent=d.clients;
 const st=document.getElementById('staState'),sip=document.getElementById('staIP');
 st.textContent=d.staConnected?'Connected':'Not connected';
 sip.textContent=d.staConnected?d.staIP:'--';
}).catch(()=>{});}
poll();setInterval(poll,3000);
function connect(){const s=document.getElementById('staS').value.trim(),p=document.getElementById('staP').value;
 if(s.length===0){show('SSID required');return;}
 if(p.length>0&&p.length<8){show('Password min 8 chars');return;}
 fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:s,pass:p})})
 .then(r=>{if(!r.ok)throw 0;return r.json();}).then(()=>{show('Connecting...');setTimeout(poll,1500);}).catch(()=>show('Failed'));}
function disconnect(){fetch('/api/wifi',{method:'DELETE'}).then(()=>show('Disconnected')).catch(()=>show('Failed'));}
document.addEventListener('visibilitychange',()=>{if(!document.hidden)poll();});
</script></body></html>)HTML";

static void sendJsonNoCopy(int code, const char* body, size_t len) {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("Connection", "close");
  server.setContentLength(len);
  server.send(code, "application/json", "");
  server.sendContent(body, len);
  server.client().stop();
}

static uint8_t crc8(const uint8_t* d, size_t n) {
  uint8_t c = 0;
  for (size_t i = 0; i < n; i++) {
    c ^= d[i];
    for (uint8_t b = 0; b < 8; b++) c = (c & 0x80) ? (uint8_t)((c << 1) ^ 0x07) : (uint8_t)(c << 1);
  }
  return c;
}

static void defaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.magic = MAGIC;
  cfg.version = FW_VERSION;
  cfg.tempLow = 500;
  cfg.tempHigh = 3000;
  cfg.humLow = 2000;
  cfg.humHigh = 8000;
  cfg.pressLow = 95000;
  cfg.pressHigh = 105000;
  cfg.relayMode = 0;
  cfg.crc = crc8((const uint8_t*)&cfg, offsetof(Settings, crc));
}

static bool loadCfg() {
  EEPROM.get(0, cfg);
  if (cfg.magic != MAGIC || cfg.version != FW_VERSION) return false;
  Settings tmp = cfg;
  uint8_t saved = tmp.crc;
  return crc8((const uint8_t*)&tmp, offsetof(Settings, crc)) == saved;
}

static bool saveCfg() {
  cfg.crc = crc8((const uint8_t*)&cfg, offsetof(Settings, crc));
  EEPROM.put(0, cfg);
  if (EEPROM.commit()) return true;
  yield();
  ESP.wdtFeed();
  EEPROM.put(0, cfg);
  return EEPROM.commit();
}

static inline uint32_t elapsed32(uint32_t start, uint32_t now) { return now - start; }

static inline uint64_t uptimeMs() {
  uint32_t now = millis();
  uint32_t d = now - s_upLast;
  s_upLast = now;
  s_upAccMs += (uint64_t)d;
  return s_upAccMs;
}

static inline uint64_t uptimeSeconds() { return uptimeMs() / 1000ULL; }

static void relayControl(int64_t t, int64_t h, int64_t p, bool valid) {
  if (cfg.relayMode == 1) { digitalWrite(RELAY_PIN, HIGH); return; }
  if (cfg.relayMode == 2) { digitalWrite(RELAY_PIN, LOW); return; }
  if (!valid) { digitalWrite(RELAY_PIN, LOW); return; }
  bool trigger = false;
  if (t < cfg.tempLow || t > cfg.tempHigh) trigger = true;
  if (h < cfg.humLow || h > cfg.humHigh) trigger = true;
  if (p < cfg.pressLow || p > cfg.pressHigh) trigger = true;
  digitalWrite(RELAY_PIN, trigger ? HIGH : LOW);
}

static void factoryReset() {
  cfgDirty = false;
  sensorCooldown = false;
  sensorFailStreak = 0;
  EEPROM.begin(EEPROM_SIZE);
  for (int i = 0; i < EEPROM_SIZE; i++) EEPROM.write(i, 0xFF);
  EEPROM.commit();
  EEPROM.end();
  cfgDirty = false;
  WiFi.persistent(false);
  ESP.restart();
}

static void setupWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_OFF);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(AP_SSID, AP_PASS);
  dns.setErrorReplyCode(DNSReplyCode::NoError);
  dns.start(53, "*", WiFi.softAPIP());
}

static void setupBME() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(400000);
  Wire.setClockStretchLimit(1500);
  bmeOk = bme.begin(0x76, &Wire);
  if (!bmeOk) bmeOk = bme.begin(0x77, &Wire);
  if (bmeOk) {
    bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X16,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_X4,
                    Adafruit_BME280::STANDBY_MS_500);
  }
}

static void setupWeb() {
  server.on("/", HTTP_GET, []() { server.send_P(200, "text/html", PAGE_INDEX); });
  server.on("/config", HTTP_GET, []() { server.send_P(200, "text/html", PAGE_CONFIG); });
  server.on("/wifi", HTTP_GET, []() { server.send_P(200, "text/html", PAGE_WIFI); });

  server.on("/api/status", HTTP_GET, []() {
    int64_t t = 0, h = 0, p = 0;
    bool valid = false;
    if (bmeOk && sensorValid) {
      t = lastTemp;
      h = lastHum;
      p = lastPress;
      valid = true;
    }
    uint64_t up = uptimeSeconds();
    uint32_t hrs = (uint32_t)(up / 3600ULL);
    uint32_t mins = (uint32_t)((up % 3600ULL) / 60ULL);
    uint32_t secs = (uint32_t)(up % 60ULL);
    char tStr[24];
    char hStr[24];
    char pStr[24];
    if (valid) {
      snprintf(tStr, sizeof(tStr), "%.2f", (double)t / 100.0);
      snprintf(hStr, sizeof(hStr), "%.2f", (double)h / 100.0);
      snprintf(pStr, sizeof(pStr), "%.2f", (double)p / 100.0);
    } else {
      strcpy(tStr, "null");
      strcpy(hStr, "null");
      strcpy(pStr, "null");
    }
    char out[352];
    int n = snprintf(out, sizeof(out),
      "{\"t\":%s,\"h\":%s,\"p\":%s,"
      "\"relay\":%d,\"mode\":%u,\"bme\":%d,\"valid\":%d,\"cooldown\":%d,"
      "\"up\":\"%luh %lum %lus\",\"heap\":%u,\"rssi\":%d}",
      tStr, hStr, pStr,
      digitalRead(RELAY_PIN) ? 1 : 0,
      (unsigned)cfg.relayMode,
      bmeOk ? 1 : 0,
      valid ? 1 : 0,
      sensorCooldown ? 1 : 0,
      (unsigned long)hrs, (unsigned long)mins, (unsigned long)secs,
      (unsigned)ESP.getFreeHeap(),
      (int)WiFi.RSSI());
    if (n < 0 || n >= (int)sizeof(out)) {
      server.send(500, "application/json", F("{\"e\":500}"));
      return;
    }
    sendJsonNoCopy(200, out, (size_t)n);
  });

  server.on("/api/config", HTTP_GET, []() {
    char out[192];
    int n = snprintf(out, sizeof(out),
      "{\"tL\":%ld,\"tH\":%ld,\"hL\":%ld,\"hH\":%ld,\"pL\":%ld,\"pH\":%ld}",
      (long)(cfg.tempLow / 100),  (long)(cfg.tempHigh / 100),
      (long)(cfg.humLow / 100),   (long)(cfg.humHigh / 100),
      (long)(cfg.pressLow / 100), (long)(cfg.pressHigh / 100));
    if (n < 0 || n >= (int)sizeof(out)) {
      server.send(500, "application/json", F("{\"e\":500}"));
      return;
    }
    sendJsonNoCopy(200, out, (size_t)n);
  });

  server.on("/api/config", HTTP_POST, []() {
    if (!server.hasArg("plain")) { server.send(400, "application/json", F("{\"e\":1}")); return; }
    String body = server.arg("plain");
    if (body.length() > 256) { server.send(413, "application/json", F("{\"e\":413}")); return; }
    auto getVal = [&](const char* key, bool& ok, long lo, long hi) -> long {
      String k = String("\"") + key + "\":";
      int idx = body.indexOf(k);
      if (idx < 0) { ok = false; return 0; }
      idx += k.length();
      int end = idx;
      while (end < (int)body.length() && body[end] != ',' && body[end] != '}') end++;
      String v = body.substring(idx, end);
      v.trim();
      if (v.length() == 0) { ok = false; return 0; }
      for (int i = 0; i < (int)v.length(); i++) {
        char c = v[i];
        if (!((c >= '0' && c <= '9') || (i == 0 && c == '-'))) { ok = false; return 0; }
      }
      long val = v.toInt();
      if (val < lo || val > hi) { ok = false; return 0; }
      return val;
    };
    bool ok = true;
    long tL = getVal("tL", ok, -40, 85);
    long tH = getVal("tH", ok, -40, 85);
    long hL = getVal("hL", ok, 0, 100);
    long hH = getVal("hH", ok, 0, 100);
    long pL = getVal("pL", ok, 300, 1100);
    long pH = getVal("pH", ok, 300, 1100);
    if (!ok) { server.send(400, "application/json", F("{\"e\":2}")); return; }
    if (tL >= tH || hL >= hH || pL >= pH) {
      server.send(400, "application/json", F("{\"e\":4}"));
      return;
    }
    Settings n = cfg;
    n.tempLow = (int64_t)tL * 100LL;
    n.tempHigh = (int64_t)tH * 100LL;
    n.humLow = (int64_t)hL * 100LL;
    n.humHigh = (int64_t)hH * 100LL;
    n.pressLow = (int64_t)pL * 100LL;
    n.pressHigh = (int64_t)pH * 100LL;
    n.magic = MAGIC;
    n.version = FW_VERSION;
    cfg = n;
    cfgDirty = true;
    cfgDirtySince = millis();
    server.send(200, "application/json", F("{\"ok\":1}"));
  });

  server.on("/api/relay", HTTP_POST, []() {
    if (!server.hasArg("m")) { server.send(400, "application/json", F("{\"e\":1}")); return; }
    int m = server.arg("m").toInt();
    if (m == 0 || m == 1 || m == 2) {
      if (cfg.relayMode != (uint8_t)m) {
        cfg.relayMode = (uint8_t)m;
        cfgDirty = true;
        cfgDirtySince = millis();
      }
      if (m == 1) digitalWrite(RELAY_PIN, HIGH);
      else if (m == 2) digitalWrite(RELAY_PIN, LOW);
      else relayControl(lastTemp, lastHum, lastPress, sensorValid);
      server.send(200, "application/json", F("{\"ok\":1}"));
    } else {
      server.send(400, "application/json", F("{\"e\":2}"));
    }
  });

  server.on("/api/wifi", HTTP_GET, []() {
    char out[256];
    IPAddress ip = WiFi.softAPIP();
    IPAddress sip = WiFi.localIP();
    int sta = (WiFi.status() == WL_CONNECTED) ? 1 : 0;
    int n = snprintf(out, sizeof(out),
      "{\"ssid\":\"%s\",\"ip\":\"%u.%u.%u.%u\",\"clients\":%u,"
      "\"staConnected\":%d,\"staIP\":\"%u.%u.%u.%u\",\"staStatus\":%d}",
      AP_SSID,
      (unsigned)ip[0], (unsigned)ip[1], (unsigned)ip[2], (unsigned)ip[3],
      (unsigned)WiFi.softAPgetStationNum(),
      sta,
      (unsigned)sip[0], (unsigned)sip[1], (unsigned)sip[2], (unsigned)sip[3],
      (int)WiFi.status());
    if (n < 0 || n >= (int)sizeof(out)) {
      server.send(500, "application/json", F("{\"e\":500}"));
      return;
    }
    sendJsonNoCopy(200, out, (size_t)n);
  });

  server.on("/api/wifi", HTTP_POST, []() {
    if (!server.hasArg("plain")) { server.send(400, "application/json", F("{\"e\":1}")); return; }
    String body = server.arg("plain");
    if (body.length() > 256) { server.send(413, "application/json", F("{\"e\":413}")); return; }
    int si = body.indexOf("\"ssid\":\"");
    int pi = body.indexOf("\"pass\":\"");
    if (si < 0 || pi < 0) { server.send(400, "application/json", F("{\"e\":2}")); return; }
    si += 8;
    int se = body.indexOf('"', si);
    pi += 8;
    int pe = body.indexOf('"', pi);
    if (se < 0 || pe < 0) { server.send(400, "application/json", F("{\"e\":3}")); return; }
    String ssid = body.substring(si, se);
    String pass = body.substring(pi, pe);
    if (ssid.length() == 0 || ssid.length() > 32 || pass.length() > 64 ||
        (pass.length() > 0 && pass.length() < 8)) {
      server.send(400, "application/json", F("{\"e\":5}"));
      return;
    }
    WiFi.mode(WIFI_AP_STA);
    WiFi.persistent(false);
    WiFi.begin(ssid.c_str(), pass.c_str());
    server.send(200, "application/json", F("{\"ok\":1,\"started\":1}"));
  });

  server.on("/api/wifi", HTTP_DELETE, []() {
    WiFi.disconnect(false);
    WiFi.mode(WIFI_AP);
    server.send(200, "application/json", F("{\"ok\":1}"));
  });

  server.on("/api/factory", HTTP_POST, []() {
    server.send(200, "application/json", F("{\"ok\":1}"));
    server.client().stop();
    factoryReset();
  });

  server.onNotFound([]() {
    String uri = server.uri();
    if (uri.startsWith("/api/")) {
      server.send(404, "application/json", F("{\"e\":404}"));
      return;
    }
    if (server.method() == HTTP_GET) {
      server.sendHeader("Location", "http://192.168.4.1/", true);
      server.send(302, "text/plain", "");
    } else {
      server.send(404, "application/json", F("{\"e\":404}"));
    }
  });

  server.begin();
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);

  EEPROM.begin(EEPROM_SIZE);
  if (!loadCfg()) {
    defaults();
    saveCfg();
  }

  setupWiFi();
  setupBME();
  setupWeb();

  s_upLast = millis();
  s_upAccMs = 0;
}

void loop() {
  dns.processNextRequest();
  server.handleClient();
  yield();

  uint32_t now = millis();

  if (sensorCooldown) {
    if ((int32_t)(now - sensorCooldownUntil) >= 0) {
      sensorCooldown = false;
      sensorFailStreak = 0;
    }
  }

  if (cfgDirty && elapsed32(cfgDirtySince, now) >= SAVE_DEBOUNCE_MS) {
    cfgDirty = false;
    saveCfg();
  }

  if (elapsed32(lastBtnMs, now) >= BTN_POLL_MS) {
    lastBtnMs = now;
    bool pressed = digitalRead(BOOT_BTN_PIN) == LOW;
    if (pressed && !btnDown) {
      btnDown = true;
      btnDownAtMs = now;
    } else if (!pressed && btnDown) {
      btnDown = false;
    } else if (pressed && btnDown) {
      if (elapsed32(btnDownAtMs, now) >= BTN_HOLD_MS) {
        factoryReset();
      }
    }
  }

  if (elapsed32(lastSampleMs, now) >= SENSOR_TICK_MS && !sensorCooldown) {
    lastSampleMs = now;
    if (bmeOk) {
      ESP.wdtFeed();
      float tf = bme.readTemperature();
      ESP.wdtFeed();
      bool okT = !isnan(tf);
      if (okT) {
        lastTemp = (int64_t)llround(tf * 100.0f);
      }

      ESP.wdtFeed();
      float hf = bme.readHumidity();
      ESP.wdtFeed();
      bool okH = !isnan(hf);
      if (okH) {
        lastHum = (int64_t)llround(hf * 100.0f);
      }

      ESP.wdtFeed();
      float pf = bme.readPressure();
      ESP.wdtFeed();
      bool okP = !isnan(pf);
      if (okP) {
        lastPress = (int64_t)llround(pf);
      }

      sensorValid = okT && okH && okP;
      if (sensorValid) {
        sensorFailStreak = 0;
      } else {
        if (sensorFailStreak < 255) sensorFailStreak++;
        if (sensorFailStreak >= SENSOR_FAIL_THRESHOLD) {
          sensorCooldown = true;
          sensorCooldownUntil = now + SENSOR_COOLDOWN_MS;
          sensorFailStreak = 0;
        }
      }
      relayControl(lastTemp, lastHum, lastPress, sensorValid);
    }
  }
}
