/*
 * ESP32 BME280 Controller
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_timer.h>
#include <stddef.h>

#define PIN_SDA     21
#define PIN_SCL     22
#define PIN_RELAY   25
#define PIN_BOOT    0

static const char* AP_SSID = "ESP32_BME280";
static const char* AP_PASS = "12345678";
static const IPAddress AP_IP(192, 168, 4, 1);

static const uint64_t BOOT_HOLD_MS         = 5000ULL;
static const uint64_t SENSOR_PERIOD_MS     = 2000ULL;
static const uint64_t RELAY_MIN_TOGGLE_MS  = 1000ULL;
static const uint64_t SAVE_MIN_INTERVAL_MS = 60000ULL;
static const uint64_t SENSOR_RETRY_MS      = 5000ULL;
static const uint64_t LOOP_DELAY_MS        = 2ULL;
static const uint64_t REBOOT_DELAY_MS      = 500ULL;

static const int MAX_BODY_SIZE = 8192;
static const char* NVS_NS  = "bme280cfg";
static const uint32_t CFG_MAGIC = 0xB6E28042UL;

struct Config {
  uint32_t magic;
  int64_t  relayMode;
  int64_t  invertRelay;
  int64_t  sensorAddr;
  double   tempOn, tempOff;
  double   humOn,  humOff;
  double   presOn, presOff;
  uint32_t crc;
};

static Config  g_cfg;
static volatile bool g_cfgDirty = false;
static uint64_t g_lastSaveMs = 0;

static Adafruit_BME280 g_bme;
static bool     g_bmeOk           = false;
static float    g_temperature     = NAN;
static float    g_humidity        = NAN;
static float    g_pressure        = NAN;
static bool     g_relayOn         = false;
static uint64_t g_lastRelayMs     = 0;
static uint64_t g_lastSensorMs    = 0;
static uint64_t g_lastSensorTryMs = 0;
static uint64_t g_bootMs          = 0;
static uint64_t g_lastBootCheckMs = 0;
static bool     g_bootHeld        = false;
static uint64_t g_bootHoldStart   = 0;
static bool     g_resetRequested  = false;
static bool     g_rebootRequested = false;
static uint64_t g_rebootAtMs      = 0;

static WebServer   g_server(80);
static DNSServer   g_dns;
static Preferences g_prefs;

static inline uint64_t nowMs() { return (uint64_t)(esp_timer_get_time() / 1000ULL); }

static String fmtU64(uint64_t v) {
  char buf[24]; snprintf(buf, sizeof(buf), "%llu", (unsigned long long)v);
  return String(buf);
}

static uint32_t cfgChecksum(const Config& c) {
  const uint8_t* p = (const uint8_t*)&c;
  size_t n = offsetof(Config, crc);
  uint32_t h = 2166136261UL;
  for (size_t i = 0; i < n; ++i) { h ^= p[i]; h *= 16777619UL; }
  return h;
}
static double clampD(double v, double lo, double hi, double dflt) {
  if (isnan(v) || isinf(v)) return dflt;
  if (v < lo) return lo; if (v > hi) return hi; return v;
}
static int64_t clampI(int64_t v, int64_t lo, int64_t hi, int64_t dflt) {
  if (v < lo || v > hi) return dflt; return v;
}

static void loadDefaults(Config& c) {
  c.magic=CFG_MAGIC; c.relayMode=0; c.invertRelay=0; c.sensorAddr=0;
  c.tempOn=30.0; c.tempOff=28.0; c.humOn=70.0; c.humOff=65.0;
  c.presOn=1020.0; c.presOff=1015.0; c.crc=0; c.crc=cfgChecksum(c);
}
static bool cfgValid(const Config& c) {
  if (c.magic != CFG_MAGIC) return false;
  if (cfgChecksum(c) != c.crc) return false;
  if (c.relayMode < 0 || c.relayMode > 3) return false;
  if (c.invertRelay != 0 && c.invertRelay != 1) return false;
  if (c.sensorAddr != 0 && c.sensorAddr != 0x76 && c.sensorAddr != 0x77) return false;
  if (!(c.tempOn  >= -40 && c.tempOn  <= 85))  return false;
  if (!(c.tempOff >= -40 && c.tempOff <= 85))  return false;
  if (!(c.humOn   >= 0   && c.humOn   <= 100)) return false;
  if (!(c.humOff  >= 0   && c.humOff  <= 100)) return false;
  if (!(c.presOn  >= 300 && c.presOn  <= 1100)) return false;
  if (!(c.presOff >= 300 && c.presOff <= 1100)) return false;
  return true;
}
static void sanitizeConfig(Config& c) {
  c.relayMode   = clampI(c.relayMode,   0, 3, 0);
  c.invertRelay = clampI(c.invertRelay, 0, 1, 0);
  if (c.sensorAddr != 0 && c.sensorAddr != 0x76 && c.sensorAddr != 0x77) c.sensorAddr = 0;
  c.tempOn  = clampD(c.tempOn,  -40, 85,   30.0);
  c.tempOff = clampD(c.tempOff, -40, 85,   28.0);
  c.humOn   = clampD(c.humOn,    0, 100,   70.0);
  c.humOff  = clampD(c.humOff,   0, 100,   65.0);
  c.presOn  = clampD(c.presOn,  300, 1100, 1020.0);
  c.presOff = clampD(c.presOff, 300, 1100, 1015.0);
  c.magic   = CFG_MAGIC;
  c.crc     = cfgChecksum(c);
}

static void loadConfig() {
  Config def; loadDefaults(def);
  if (!g_prefs.begin(NVS_NS, true)) { g_cfg = def; g_cfgDirty = true; return; }
  size_t n = g_prefs.getBytesLength("cfg");
  if (n == sizeof(Config)) {
    Config loaded;
    size_t got = g_prefs.getBytes("cfg", &loaded, sizeof(Config));
    if (got == sizeof(Config) && cfgValid(loaded)) {
      g_cfg = loaded; g_prefs.end(); return;
    }
  }
  g_prefs.end(); g_cfg = def; g_cfgDirty = true;
}
static void saveConfigNow() {
  Config tmp = g_cfg; sanitizeConfig(tmp);
  if (!g_prefs.begin(NVS_NS, false)) { g_lastSaveMs = nowMs(); return; }
  g_prefs.putBytes("cfg", &tmp, sizeof(Config));
  g_prefs.end();
  g_cfg = tmp; g_lastSaveMs = nowMs(); g_cfgDirty = false;
}
static void maybeSaveConfig() {
  if (!g_cfgDirty) return;
  if ((nowMs() - g_lastSaveMs) >= SAVE_MIN_INTERVAL_MS) saveConfigNow();
}
static void markConfigDirty() { sanitizeConfig(g_cfg); g_cfgDirty = true; }

static void configureSampling() {
  g_bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X16,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_1000);
}
static bool initSensor() {
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  delay(10);
  int64_t locked = g_cfg.sensorAddr;
  if (locked == 0x76 || locked == 0x77) {
    if (g_bme.begin((uint8_t)locked, &Wire)) { configureSampling(); return true; }
  }
  if (g_bme.begin(0x76, &Wire)) { g_cfg.sensorAddr = 0x76; configureSampling(); return true; }
  if (g_bme.begin(0x77, &Wire)) { g_cfg.sensorAddr = 0x77; configureSampling(); return true; }
  return false;
}
static void readSensor() {
  float t = g_bme.readTemperature();
  float h = g_bme.readHumidity();
  float p = g_bme.readPressure() / 100.0f;
  if (!isnan(t) && !isnan(h) && !isnan(p)) {
    g_temperature = t; g_humidity = h; g_pressure = p;
  }
}
static void writeRelayPin(bool on) {
  int level = on ? HIGH : LOW;
  if (g_cfg.invertRelay) level = (level == HIGH) ? LOW : HIGH;
  digitalWrite(PIN_RELAY, level);
}
static void setRelay(bool on, bool force) {
  (void)force;
  uint64_t t = nowMs();
  if (on == g_relayOn) return;
  if ((t - g_lastRelayMs) < RELAY_MIN_TOGGLE_MS) return;
  writeRelayPin(on);
  g_relayOn = on;
  g_lastRelayMs = t;
}
static void evaluateRelay() {
  int64_t mode = g_cfg.relayMode;
  if (mode == 0) { setRelay(false, false); return; }
  if (mode == 1) {
    if (isnan(g_temperature)) return;
    if (!g_relayOn && g_temperature >= g_cfg.tempOn)  setRelay(true, false);
    else if (g_relayOn && g_temperature <= g_cfg.tempOff) setRelay(false, false);
  } else if (mode == 2) {
    if (isnan(g_humidity)) return;
    if (!g_relayOn && g_humidity >= g_cfg.humOn)  setRelay(true, false);
    else if (g_relayOn && g_humidity <= g_cfg.humOff) setRelay(false, false);
  } else if (mode == 3) {
    if (isnan(g_pressure)) return;
    if (!g_relayOn && g_pressure >= g_cfg.presOn)  setRelay(true, false);
    else if (g_relayOn && g_pressure <= g_cfg.presOff) setRelay(false, false);
  }
}
static void serviceBootButton() {
  uint64_t t = nowMs();
  if ((t - g_lastBootCheckMs) < 50ULL) return;
  g_lastBootCheckMs = t;
  int level = digitalRead(PIN_BOOT);
  if (level == LOW) {
    if (!g_bootHeld) { g_bootHeld = true; g_bootHoldStart = t; }
    else if ((t - g_bootHoldStart) >= BOOT_HOLD_MS) g_resetRequested = true;
  } else { g_bootHeld = false; g_bootHoldStart = 0; }
}
static void performFactoryReset() {
  setRelay(false, true);
  g_lastRelayMs = nowMs();
  if (g_prefs.begin(NVS_NS, false)) { g_prefs.clear(); g_prefs.end(); }
  Config def; loadDefaults(def);
  g_cfg = def; g_cfgDirty = true;
  saveConfigNow();
  g_resetRequested = false;
}

// ============ HTML ============
static const char HTML_HEAD[] PROGMEM = R"HTML(<!DOCTYPE html><html lang="en"><head>
<meta charset="utf-8"/>
<meta name="viewport" content="width=device-width,initial-scale=1,viewport-fit=cover"/>
<meta name="theme-color" content="#0f172a"/>
<title>ESP32 BME280</title>
<style>
:root{--bg:#0f172a;--card:#1e293b;--fg:#e2e8f0;--mut:#94a3b8;--acc:#38bdf8;--ok:#22c55e;--bad:#ef4444;--warn:#f59e0b;--bd:#334155}
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
html,body{margin:0;padding:0;background:var(--bg);color:var(--fg);font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;font-size:16px;line-height:1.45}
header{position:sticky;top:0;z-index:10;background:rgba(15,23,42,.96);border-bottom:1px solid var(--bd);padding:10px 14px;display:flex;align-items:center;justify-content:space-between;backdrop-filter:blur(6px);gap:10px;flex-wrap:wrap}
header h1{font-size:18px;margin:0;font-weight:600}
nav{display:flex;gap:6px;flex-wrap:wrap}
nav a{color:var(--mut);text-decoration:none;padding:8px 12px;border-radius:9px;font-size:14px;border:1px solid transparent;transition:background .15s,color .15s}
nav a.active,nav a:hover{color:#fff;background:#334155;border-color:#475569}
main{padding:14px;max-width:1100px;margin:0 auto}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(220px,1fr));gap:12px}
.card{background:var(--card);border:1px solid var(--bd);border-radius:14px;padding:14px;margin-bottom:12px}
.card h2{margin:0 0 10px;font-size:15px;color:var(--mut);text-transform:uppercase;letter-spacing:.05em;font-weight:600}
.metric{font-size:34px;font-weight:700;letter-spacing:-.02em}
.metric small{font-size:16px;color:var(--mut);font-weight:500}
.row{display:flex;justify-content:space-between;gap:10px;padding:7px 0;border-bottom:1px solid var(--bd);font-size:14px}
.row:last-child{border-bottom:none}
.row span:first-child{color:var(--mut)}
.row span:last-child{text-align:right;word-break:break-word}
.badge{display:inline-block;padding:2px 9px;border-radius:999px;font-size:12px;font-weight:600}
.b-ok{background:rgba(34,197,94,.18);color:var(--ok)}
.b-bad{background:rgba(239,68,68,.18);color:var(--bad)}
.b-warn{background:rgba(245,158,11,.18);color:var(--warn)}
label{display:block;font-size:13px;color:var(--mut);margin-top:10px}
input,select{width:100%;padding:10px 12px;margin-top:4px;background:#0b1220;color:var(--fg);border:1px solid var(--bd);border-radius:9px;font-size:16px;outline:none;transition:border-color .15s}
input:focus,select:focus{border-color:var(--acc)}
button{padding:11px 16px;border-radius:9px;border:1px solid var(--bd);background:#334155;color:#fff;font-size:15px;font-weight:600;cursor:pointer;margin-top:12px;transition:filter .15s,transform .05s}
button.primary{background:var(--acc);border-color:var(--acc);color:#00121f}
button.danger{background:var(--bad);border-color:var(--bad)}
button:hover{filter:brightness(1.1)}
button:active{transform:translateY(1px)}
.btns{display:flex;flex-wrap:wrap;gap:8px}
.btns button{margin-top:0}
.form-grid{display:grid;grid-template-columns:1fr 1fr;gap:10px}
.msg{margin-top:10px;font-size:14px;min-height:20px}
.msg.ok{color:var(--ok)}
.msg.err{color:var(--bad)}
@media(max-width:600px){
  .form-grid{grid-template-columns:1fr}
  .metric{font-size:28px}
  main{padding:10px}
  header{padding:8px 10px}
  nav{width:100%;justify-content:space-between}
  nav a{flex:1;text-align:center;padding:10px 6px;font-size:13px}
  .btns button{flex:1 1 45%}
}
.toast{position:fixed;left:50%;bottom:20px;transform:translateX(-50%);background:#334155;color:#fff;padding:10px 16px;border-radius:10px;font-size:14px;opacity:0;pointer-events:none;transition:opacity .25s;z-index:999}
.toast.show{opacity:1}
footer{text-align:center;color:var(--mut);font-size:12px;padding:16px}
</style></head><body>
)HTML";

static const char HTML_NAV_FMT[] PROGMEM =
"<header><h1>ESP32 BME280</h1><nav>"
"<a href='/' class='%s'>Dashboard</a>"
"<a href='/sensor' class='%s'>Sensor</a>"
"<a href='/wifi' class='%s'>WiFi</a>"
"</nav></header><main>";

static const char HTML_TAIL[] PROGMEM =
"</main><div id='toast' class='toast'></div>"
"<script>"
"function t(m){var e=document.getElementById('toast');if(!e)return;e.textContent=m;e.classList.add('show');clearTimeout(window._tt);window._tt=setTimeout(function(){e.classList.remove('show');},1800);}"
"function post(u,d){return fetch(u,{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(d||{})}).then(function(r){return r.json();});}"
"function reload(){location.reload();}"
"</script></body></html>";

static void sendPage(const String& body, const char* active) {
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.setContentLength(CONTENT_LENGTH_UNKNOWN);
  g_server.send(200, "text/html; charset=utf-8", "");
  char nav[sizeof(HTML_NAV_FMT) + 32];
  snprintf(nav, sizeof(nav), HTML_NAV_FMT,
           strcmp(active, "dash")   == 0 ? "active" : "",
           strcmp(active, "sensor") == 0 ? "active" : "",
           strcmp(active, "wifi")   == 0 ? "active" : "");
  g_server.sendContent_P(HTML_HEAD);
  g_server.sendContent(nav);
  g_server.sendContent(body);
  g_server.sendContent_P(HTML_TAIL);
  g_server.sendContent("", 0);
}
static String fmtF(float v, int d) { if (isnan(v)) return String("--"); return String(v, d); }

static String dashboardBody() {
  String b; b.reserve(2800);
  b += F("<div class='grid'>");
  b += F("<div class='card'><h2>Temperature</h2><div class='metric'>"); b += fmtF(g_temperature,2); b += F("<small>&nbsp;&deg;C</small></div></div>");
  b += F("<div class='card'><h2>Humidity</h2><div class='metric'>");    b += fmtF(g_humidity,2);    b += F("<small>&nbsp;%RH</small></div></div>");
  b += F("<div class='card'><h2>Pressure</h2><div class='metric'>");    b += fmtF(g_pressure,2);    b += F("<small>&nbsp;hPa</small></div></div>");
  b += F("</div>");
  b += F("<div class='card'><h2>Status</h2>");
  b += F("<div class='row'><span>Sensor</span><span class='badge "); b += g_bmeOk ? F("b-ok'>OK") : F("b-bad'>FAULT"); b += F("</span></div>");
  b += F("<div class='row'><span>Relay</span><span class='badge ");  b += g_relayOn ? F("b-ok'>ON") : F("b-warn'>OFF"); b += F("</span></div>");
  b += F("<div class='row'><span>Relay Mode</span><span>");
  if (g_cfg.relayMode==0) b += F("Disabled");
  else if (g_cfg.relayMode==1) b += F("Temperature");
  else if (g_cfg.relayMode==2) b += F("Humidity");
  else b += F("Pressure");
  b += F("</span></div>");
  b += F("<div class='row'><span>IP</span><span>192.168.4.1</span></div>");
  b += F("<div class='row'><span>Uptime</span><span>");  b += fmtU64((nowMs()-g_bootMs)/1000ULL); b += F(" s</span></div>");
  b += F("<div class='row'><span>Free Heap</span><span>"); b += fmtU64((uint64_t)ESP.getFreeHeap()); b += F(" B</span></div>");
  b += F("<div class='row'><span>Min Free Heap</span><span>"); b += fmtU64((uint64_t)ESP.getMinFreeHeap()); b += F(" B</span></div>");
  b += F("<div class='row'><span>BME280 Addr</span><span>");
  if (g_cfg.sensorAddr==0x76) b += F("0x76"); else if (g_cfg.sensorAddr==0x77) b += F("0x77"); else b += F("--");
  b += F("</span></div></div>");
  b += F("<div class='card'><h2>Quick Relay</h2><div class='btns'>");
  b += F("<button onclick=\"post('/api/relay',{on:true}).then(function(r){t(r.ok?'Relay ON':'Error');setTimeout(reload,400);})\">Force ON</button>");
  b += F("<button onclick=\"post('/api/relay',{on:false}).then(function(r){t(r.ok?'Relay OFF':'Error');setTimeout(reload,400);})\">Force OFF</button>");
  b += F("<button onclick=\"post('/api/relaymode',{mode:0}).then(function(r){t(r.ok?'Mode disabled':'Error');setTimeout(reload,400);})\">Disable Mode</button>");
  b += F("</div></div>");
  return b;
}

static String sensorBody() {
  String b; b.reserve(3200);
  b += F("<div class='card'><h2>Sensor Management</h2><form id='f'>");
  b += F("<label>Relay Trigger Source</label><select id='mode'>");
  b += F("<option value='0'"); if (g_cfg.relayMode==0) b += F(" selected"); b += F(">Disabled</option>");
  b += F("<option value='1'"); if (g_cfg.relayMode==1) b += F(" selected"); b += F(">Temperature</option>");
  b += F("<option value='2'"); if (g_cfg.relayMode==2) b += F(" selected"); b += F(">Humidity</option>");
  b += F("<option value='3'"); if (g_cfg.relayMode==3) b += F(" selected"); b += F(">Pressure</option>");
  b += F("</select>");
  b += F("<div class='form-grid'>");
  b += F("<div><label>Temp ON (&deg;C)</label><input id='ton'  type='number' step='0.1' min='-40'  max='85'   value='"); b += String((double)g_cfg.tempOn, 2);  b += F("'></div>");
  b += F("<div><label>Temp OFF (&deg;C)</label><input id='toff' type='number' step='0.1' min='-40'  max='85'   value='"); b += String((double)g_cfg.tempOff,2);  b += F("'></div>");
  b += F("<div><label>Hum ON (%RH)</label><input id='hon'  type='number' step='0.1' min='0'    max='100'  value='"); b += String((double)g_cfg.humOn, 2);   b += F("'></div>");
  b += F("<div><label>Hum OFF (%RH)</label><input id='hoff' type='number' step='0.1' min='0'    max='100'  value='"); b += String((double)g_cfg.humOff,2);   b += F("'></div>");
  b += F("<div><label>Pres ON (hPa)</label><input id='pon'  type='number' step='0.1' min='300'  max='1100' value='"); b += String((double)g_cfg.presOn, 2);  b += F("'></div>");
  b += F("<div><label>Pres OFF (hPa)</label><input id='poff' type='number' step='0.1' min='300'  max='1100' value='"); b += String((double)g_cfg.presOff,2);  b += F("'></div>");
  b += F("</div>");
  b += F("<label>Relay Polarity</label><select id='inv'>");
  b += F("<option value='0'"); if (!g_cfg.invertRelay) b += F(" selected"); b += F(">Active HIGH</option>");
  b += F("<option value='1'"); if ( g_cfg.invertRelay) b += F(" selected"); b += F(">Active LOW</option>");
  b += F("</select>");
  b += F("<button class='primary' type='submit'>Save Settings</button>");
  b += F("<div id='m' class='msg'></div>");
  b += F("</form></div>");
  b += F("<div class='card'><h2>Live Reading</h2>");
  b += F("<div class='row'><span>Temperature</span><span>"); b += fmtF(g_temperature,2); b += F(" &deg;C</span></div>");
  b += F("<div class='row'><span>Humidity</span><span>");    b += fmtF(g_humidity,2);    b += F(" %RH</span></div>");
  b += F("<div class='row'><span>Pressure</span><span>");    b += fmtF(g_pressure,2);    b += F(" hPa</span></div>");
  b += F("<div class='row'><span>Sensor Status</span><span class='badge "); b += g_bmeOk ? F("b-ok'>OK") : F("b-bad'>FAULT"); b += F("</span></div>");
  b += F("<div class='row'><span>I2C Address</span><span>");
  if (g_cfg.sensorAddr==0x76) b += F("0x76"); else if (g_cfg.sensorAddr==0x77) b += F("0x77"); else b += F("--");
  b += F("</span></div></div>");
  b += F("<script>"
         "document.getElementById('f').addEventListener('submit',function(e){"
         "e.preventDefault();var m=document.getElementById('m');m.className='msg';m.textContent='Saving...';"
         "var d={mode:+document.getElementById('mode').value,"
         "ton:+document.getElementById('ton').value,toff:+document.getElementById('toff').value,"
         "hon:+document.getElementById('hon').value,hoff:+document.getElementById('hoff').value,"
         "pon:+document.getElementById('pon').value,poff:+document.getElementById('poff').value,"
         "inv:+document.getElementById('inv').value};"
         "post('/api/sensor',d).then(function(r){"
         "if(r.ok){m.className='msg ok';m.textContent='Saved';t('Saved');setTimeout(reload,600);}"
         "else{m.className='msg err';m.textContent=(r.msg||'Error');t(r.msg||'Error');}"
         "}).catch(function(){m.className='msg err';m.textContent='Network error';});"
         "});</script>");
  return b;
}

static String wifiBody() {
  String b; b.reserve(1800);
  b += F("<div class='card'><h2>Access Point</h2>");
  b += F("<div class='row'><span>SSID</span><span>"); b += AP_SSID; b += F("</span></div>");
  b += F("<div class='row'><span>Password</span><span>"); b += AP_PASS; b += F("</span></div>");
  b += F("<div class='row'><span>IP Address</span><span>192.168.4.1</span></div>");
  b += F("<div class='row'><span>Security</span><span>WPA2-PSK</span></div>");
  b += F("<div class='row'><span>Channel</span><span>"); b += fmtU64((uint64_t)WiFi.channel()); b += F("</span></div>");
  b += F("<div class='row'><span>Clients</span><span>"); b += fmtU64((uint64_t)WiFi.softAPgetStationNum()); b += F("</span></div>");
  b += F("<div class='row'><span>Status</span><span class='badge b-ok'>ACTIVE</span></div>");
  b += F("</div>");
  b += F("<div class='card'><h2>Actions</h2><div class='btns'>");
  b += F("<button onclick=\"if(confirm('Reboot device?')){post('/api/reboot',{});t('Rebooting...');setTimeout(reload,6000);}\">Reboot</button>");
  b += F("<button class='danger' onclick=\"if(confirm('Factory reset ALL settings?')){post('/api/factoryreset',{});t('Resetting...');setTimeout(reload,7000);}\">Factory Reset</button>");
  b += F("</div></div>");
  return b;
}

static void handleRoot()   { sendPage(dashboardBody(), "dash");   }
static void handleSensor() { sendPage(sensorBody(),    "sensor"); }
static void handleWifi()   { sendPage(wifiBody(),      "wifi");   }

static void sendJsonOk()  { g_server.sendHeader("Cache-Control","no-store"); g_server.send(200,"application/json","{\"ok\":true}"); }
static void sendJsonErr(const char* m) {
  String j; j.reserve(64);
  j += F("{\"ok\":false,\"msg\":\""); j += m; j += F("\"}");
  g_server.sendHeader("Cache-Control","no-store"); g_server.send(200,"application/json",j);
}

static void handleStatus() {
  String j; j.reserve(380);
  j += F("{\"ok\":true,\"sensor\":"); j += g_bmeOk ? "true" : "false";
  j += F(",\"temperature\":"); j += isnan(g_temperature) ? String("null") : String(g_temperature,2);
  j += F(",\"humidity\":");    j += isnan(g_humidity)    ? String("null") : String(g_humidity,2);
  j += F(",\"pressure\":");    j += isnan(g_pressure)    ? String("null") : String(g_pressure,2);
  j += F(",\"relay\":");       j += g_relayOn ? "true" : "false";
  j += F(",\"mode\":");        j += String((long)g_cfg.relayMode);
  j += F(",\"addr\":");        j += String((long)g_cfg.sensorAddr);
  j += F(",\"heap\":");        j += fmtU64((uint64_t)ESP.getFreeHeap());
  j += F(",\"uptime\":");      j += fmtU64((nowMs()-g_bootMs)/1000ULL);
  j += F("}");
  g_server.sendHeader("Cache-Control","no-store"); g_server.send(200,"application/json",j);
}

static void handleApiRelay() {
  if (!g_server.hasArg("plain")) { sendJsonErr("no body"); return; }
  String body = g_server.arg("plain");
  if ((int)body.length() > MAX_BODY_SIZE) { sendJsonErr("too large"); return; }
  bool on = body.indexOf("\"on\":true") >= 0;
  setRelay(on, true);
  sendJsonOk();
}
static void handleApiRelayMode() {
  if (!g_server.hasArg("plain")) { sendJsonErr("no body"); return; }
  String body = g_server.arg("plain");
  int idx = body.indexOf("\"mode\":");
  if (idx < 0) { sendJsonErr("bad body"); return; }
  long m = body.substring(idx + 7).toInt();
  if (m < 0 || m > 3) { sendJsonErr("range"); return; }
  if (g_cfg.relayMode != (int64_t)m) {
    g_cfg.relayMode = (int64_t)m;
    markConfigDirty();
  }
  sendJsonOk();
}
static double parseField(const String& body, const char* key, double dflt, bool* found) {
  if (found) *found = false;
  String k; k.reserve(24); k += '"'; k += key; k += F("\":");
  int idx = body.indexOf(k);
  if (idx < 0) return dflt;
  int s = idx + k.length();
  int e = s;
  while (e < (int)body.length()) {
    char c = body[e];
    if (c==',' || c=='}' || c=='\n' || c==' ' || c=='\r') break;
    e++;
  }
  if (e <= s) return dflt;
  if (found) *found = true;
  return body.substring(s, e).toFloat();
}
static void handleApiSensor() {
  if (!g_server.hasArg("plain")) { sendJsonErr("no body"); return; }
  String body = g_server.arg("plain");
  if ((int)body.length() > MAX_BODY_SIZE) { sendJsonErr("too large"); return; }

  Config nc = g_cfg;
  bool f; double v;
  v = parseField(body,"mode",(double)nc.relayMode,&f);    if (f) nc.relayMode=(int64_t)v;
  v = parseField(body,"ton", nc.tempOn, &f);              if (f) nc.tempOn=v;
  v = parseField(body,"toff",nc.tempOff,&f);              if (f) nc.tempOff=v;
  v = parseField(body,"hon", nc.humOn,  &f);              if (f) nc.humOn=v;
  v = parseField(body,"hoff",nc.humOff, &f);              if (f) nc.humOff=v;
  v = parseField(body,"pon", nc.presOn, &f);              if (f) nc.presOn=v;
  v = parseField(body,"poff",nc.presOff,&f);              if (f) nc.presOff=v;
  v = parseField(body,"inv",(double)nc.invertRelay,&f);   if (f) nc.invertRelay=(int64_t)v;

  if (nc.relayMode < 0 || nc.relayMode > 3)       { sendJsonErr("mode range");       return; }
  if (nc.invertRelay != 0 && nc.invertRelay != 1) { sendJsonErr("polarity range");   return; }
  if (!(nc.tempOn  >= -40  && nc.tempOn  <= 85))  { sendJsonErr("tempOn range");     return; }
  if (!(nc.tempOff >= -40  && nc.tempOff <= 85))  { sendJsonErr("tempOff range");    return; }
  if (!(nc.humOn   >= 0    && nc.humOn   <= 100)) { sendJsonErr("humOn range");      return; }
  if (!(nc.humOff  >= 0    && nc.humOff  <= 100)) { sendJsonErr("humOff range");     return; }
  if (!(nc.presOn  >= 300  && nc.presOn  <= 1100)){ sendJsonErr("presOn range");     return; }
  if (!(nc.presOff >= 300  && nc.presOff <= 1100)){ sendJsonErr("presOff range");    return; }
  if (nc.relayMode == 1 && nc.tempOn <= nc.tempOff) { sendJsonErr("temp ON must exceed OFF"); return; }
  if (nc.relayMode == 2 && nc.humOn  <= nc.humOff)  { sendJsonErr("hum ON must exceed OFF");  return; }
  if (nc.relayMode == 3 && nc.presOn <= nc.presOff) { sendJsonErr("pres ON must exceed OFF"); return; }

  bool polarityChanged = (nc.invertRelay != g_cfg.invertRelay);
  nc.magic = CFG_MAGIC; nc.crc = 0; nc.crc = cfgChecksum(nc);

  if (memcmp(&nc, &g_cfg, offsetof(Config, crc)) != 0) {
    g_cfg = nc;
    g_cfgDirty = true;
  }
  if (polarityChanged) writeRelayPin(g_relayOn);
  sendJsonOk();
}
static void handleApiReboot() {
  sendJsonOk();
  g_rebootRequested = true;
  g_rebootAtMs = nowMs() + REBOOT_DELAY_MS;
}
static void handleApiFactoryReset() {
  sendJsonOk();
  g_resetRequested = true;
}
static void handleCaptive() {
  g_server.sendHeader("Location", "http://192.168.4.1/", true);
  g_server.sendHeader("Cache-Control", "no-store");
  g_server.send(302, "text/plain", "");
}

static void setupWifi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPsetHostname("esp32-bme280");
  WiFi.softAPConfig(AP_IP, AP_IP, IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
}
static void setupDns() {
  g_dns.setErrorReplyCode(DNSReplyCode::NoError);
  g_dns.start(53, "*", AP_IP);
}
static void setupServer() {
  g_server.on("/",                 HTTP_GET,  handleRoot);
  g_server.on("/sensor",           HTTP_GET,  handleSensor);
  g_server.on("/wifi",             HTTP_GET,  handleWifi);
  g_server.on("/api/status",       HTTP_GET,  handleStatus);
  g_server.on("/api/relay",        HTTP_POST, handleApiRelay);
  g_server.on("/api/relaymode",    HTTP_POST, handleApiRelayMode);
  g_server.on("/api/sensor",       HTTP_POST, handleApiSensor);
  g_server.on("/api/reboot",       HTTP_POST, handleApiReboot);
  g_server.on("/api/factoryreset", HTTP_POST, handleApiFactoryReset);

  g_server.on("/generate_204",              HTTP_ANY, handleCaptive);
  g_server.on("/gen_204",                   HTTP_ANY, handleCaptive);
  g_server.on("/hotspot-detect.html",       HTTP_ANY, handleCaptive);
  g_server.on("/library/test/success.html", HTTP_ANY, handleCaptive);
  g_server.on("/connecttest.txt",           HTTP_ANY, handleCaptive);
  g_server.on("/ncsi.txt",                  HTTP_ANY, handleCaptive);
  g_server.on("/fwlink",                    HTTP_ANY, handleCaptive);
  g_server.on("/redirect",                  HTTP_ANY, handleCaptive);
  g_server.on("/canonical.html",            HTTP_ANY, handleCaptive);
  g_server.on("/success.txt",               HTTP_ANY, handleCaptive);
  g_server.on("/kindle-wifi/wifistub.html", HTTP_ANY, handleCaptive);
  g_server.on("/captiveportal",             HTTP_ANY, handleCaptive);

  g_server.onNotFound(handleCaptive);
  g_server.begin();
}

void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  g_bootMs = nowMs();
  g_lastSaveMs = 0;
  loadConfig();

  int64_t prevAddr = g_cfg.sensorAddr;
  g_bmeOk = initSensor();
  if (g_cfg.sensorAddr != prevAddr) markConfigDirty();

  setupWifi();
  setupDns();
  setupServer();

  writeRelayPin(false);
  g_relayOn = false;
  g_lastRelayMs = nowMs();
  g_lastSensorMs = nowMs();
  g_lastSensorTryMs = nowMs();

  if (g_cfgDirty) saveConfigNow();
}

void loop() {
  g_server.handleClient();
  g_dns.processNextRequest();
  serviceBootButton();

  if (g_resetRequested) {
    performFactoryReset();
    delay(30);
    ESP.restart();
    return;
  }
  if (g_rebootRequested && nowMs() >= g_rebootAtMs) {
    delay(20);
    ESP.restart();
    return;
  }

  uint64_t t = nowMs();

  if ((t - g_lastSensorMs) >= SENSOR_PERIOD_MS) {
    g_lastSensorMs = t;
    if (!g_bmeOk) {
      if ((t - g_lastSensorTryMs) >= SENSOR_RETRY_MS) {
        g_lastSensorTryMs = t;
        int64_t prev = g_cfg.sensorAddr;
        g_bmeOk = initSensor();
        if (g_bmeOk && g_cfg.sensorAddr != prev) markConfigDirty();
      }
    } else {
      readSensor();
      evaluateRelay();
    }
  }

  maybeSaveConfig();

  if ((nowMs() - t) < LOOP_DELAY_MS) delay(LOOP_DELAY_MS);
}
