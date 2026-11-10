/*
 * ESP8266 BME280 Dashboard
 * AP: ESP8266_BME280 / 12345678
 * I2C: SDA=GPIO5(D1) SCL=GPIO4(D2)
 * Relay: GPIO12 (D6)
 * Boot reset: GPIO0 hold 5s
 */

#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266mDNS.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <Preferences.h>
#include <ArduinoJson.h>

#define AP_SSID_DEFAULT   "ESP8266_BME280"
#define AP_PASS_DEFAULT   "12345678"
#define AP_CHANNEL        6
#define AP_HIDDEN         0
#define AP_MAX_CONN       4

#define I2C_SDA_PIN       5
#define I2C_SCL_PIN       4
#define RELAY_PIN         12
#define RELAY_ACTIVE_LOW  true

#define BOOT_BTN_PIN      0
#define FACTORY_HOLD_MS   5000UL

#define STATUS_LED_PIN    2
#define STATUS_LED_ACTIVE_LOW true

#define SENSOR_PERIOD_MS  2000UL
#define CONTROL_PERIOD_MS 500UL
#define SENSOR_STALE_MS   15000UL
#define NVS_SAVE_DEBOUNCE 60000UL
#define MDNS_UPDATE_MS    50UL

#define DNS_PORT 53

enum RelayMode : uint8_t { MODE_AUTO = 0, MODE_MANUAL_ON = 1, MODE_MANUAL_OFF = 2 };

struct PersistConfig {
  float   tempMin;
  float   tempMax;
  float   humMin;
  float   humMax;
  float   pressMin;
  float   pressMax;
  uint8_t relayMode;
  bool    relayInvert;
  bool    relayEnabled;
  uint32_t magic;
};

static const uint32_t CFG_MAGIC = 0xBME280CF;
static bool nvsOpened = false;

PersistConfig cfg;
Preferences prefs;

Adafruit_BME280 bme;
bool bmeOk       = false;
bool i2cStarted  = false;
bool mdnsStarted = false;
bool dnsStarted  = false;
float curTemp = NAN, curHum = NAN, curPress = NAN;
uint32_t sensorErrCount  = 0;
uint32_t lastSensorOkMs  = 0;
uint32_t lastSensorTryMs = 0;
uint8_t  sensorErrStreak = 0;
bool     everReadSensor  = false;
bool relayState    = false;
bool relayPhysical = false;
ESP8266WebServer server(80);
DNSServer dnsServer;
uint32_t lastControlMs = 0;
uint32_t lastNvsSaveMs = 0;
uint32_t lastMdnsMs    = 0;
bool nvsDirty = false;
uint32_t bootPressStartMs      = 0;
bool     bootPressed           = false;
bool     factoryResetArmed     = false;
bool     pendingRestart        = false;
uint32_t pendingRestartAt      = 0;
bool     pendingFactoryReset   = false;
uint32_t pendingFactoryResetAt = 0;
inline bool timeReached(uint32_t now, uint32_t start, uint32_t interval) {
  return (uint32_t)(now - start) >= interval;
}
inline bool timeAtOrAfter(uint32_t now, uint32_t target) {
  return (int32_t)(now - target) >= 0;
}
inline bool isFiniteF(float v) { return !isnan(v) && !isinf(v); }
inline void ledWrite(bool on) {
#if STATUS_LED_ACTIVE_LOW
  digitalWrite(STATUS_LED_PIN, on ? LOW : HIGH);
#else
  digitalWrite(STATUS_LED_PIN, on ? HIGH : LOW);
#endif
}

void scheduleRestart(uint32_t delayMs = 250) {
  pendingRestart   = true;
  pendingRestartAt = millis() + delayMs;
}

void scheduleFactoryReset(uint32_t delayMs = 800) {
  pendingFactoryReset   = true;
  pendingFactoryResetAt = millis() + delayMs;
  factoryResetArmed     = true;
}

void configDefaults(PersistConfig& c) {
  c.tempMin      = 20.0f;
  c.tempMax      = 30.0f;
  c.humMin       = 30.0f;
  c.humMax       = 70.0f;
  c.pressMin     = 950.0f;
  c.pressMax     = 1050.0f;
  c.relayMode    = MODE_AUTO;
  c.relayInvert  = RELAY_ACTIVE_LOW;
  c.relayEnabled = true;
  c.magic        = CFG_MAGIC;
}
bool validateConfig(const PersistConfig& c) {
  if (c.magic != CFG_MAGIC) return false;
  if (!isFiniteF(c.tempMin)  || !isFiniteF(c.tempMax)  || !(c.tempMin  < c.tempMax))  return false;
  if (!isFiniteF(c.humMin)   || !isFiniteF(c.humMax)   || !(c.humMin   < c.humMax))   return false;
  if (!isFiniteF(c.pressMin) || !isFiniteF(c.pressMax) || !(c.pressMin < c.pressMax)) return false;
  if (c.tempMin  < -40.0f || c.tempMax  > 85.0f)   return false;
  if (c.humMin   <   0.0f || c.humMax   > 100.0f)  return false;
  if (c.pressMin < 300.0f || c.pressMax > 1100.0f) return false;
  if (c.relayMode > MODE_MANUAL_OFF)               return false;
  return true;
}
bool saveConfigNow() {
  if (!prefs.begin("bme280cfg", false)) {
    nvsOpened = false;
    return false;
  }
  nvsOpened = true;
  size_t written = prefs.putBytes("cfg", &cfg, sizeof(cfg));
  prefs.end();
  nvsOpened = false;
  if (written == sizeof(cfg)) {
    nvsDirty      = false;
    lastNvsSaveMs = millis();
    return true;
  }
  return false;
}

void loadConfig() {
  configDefaults(cfg);
  bool needPersist = true;
  if (prefs.begin("bme280cfg", true)) {
    size_t n = prefs.getBytesLength("cfg");
    if (n == sizeof(PersistConfig)) {
      PersistConfig tmp;
      if (prefs.getBytes("cfg", &tmp, sizeof(tmp)) == sizeof(tmp) && validateConfig(tmp)) {
        cfg = tmp;
        needPersist = false;
      }
    }
    prefs.end();
  }
  if (needPersist) saveConfigNow();
}

void markDirty() { nvsDirty = true; }

void factoryResetNow() {
  factoryResetArmed = true;
  if (mdnsStarted) { MDNS.end(); mdnsStarted = false; }
  if (dnsStarted)  { dnsServer.stop(); dnsStarted = false; }
  server.stop();
  WiFi.softAPdisconnect(true);

  if (prefs.begin("bme280cfg", false)) {
    prefs.clear();
    prefs.end();
  }
  nvsOpened = false;

  ESP.restart();
}

void applyRelayPhysical(bool on) {
  relayPhysical = on;
  bool level = cfg.relayInvert ? !on : on;
  digitalWrite(RELAY_PIN, level ? HIGH : LOW);
}

void setRelay(bool on) {
  if (relayPhysical == on) { relayState = on; return; }
  relayState = on;
  applyRelayPhysical(on);
}

void initSensor() {
  if (!i2cStarted) {
    Wire.begin(I2C_SDA_PIN, I2C_SCL_PIN);
    Wire.setClock(100000);
    i2cStarted = true;
  }
  if (bme.begin(0x76, &Wire) || bme.begin(0x77, &Wire)) {
    bme.setSampling(
      Adafruit_BME280::MODE_FORCED,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::SAMPLING_X1,
      Adafruit_BME280::FILTER_OFF,
      Adafruit_BME280::STANDBY_MS_1000
    );
    bmeOk = true;
  } else {
    bmeOk = false;
  }
}
bool readSensor() {
  if (!bmeOk) return false;
  if (!bme.takeForcedMeasurement()) return false;
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0f;
  if (!isFiniteF(t) || !isFiniteF(h) || !isFiniteF(p)) return false;
  if (t < -40.0f || t > 85.0f)   return false;
  if (h < 0.0f   || h > 100.0f)  return false;
  if (p < 300.0f || p > 1100.0f) return false;
  curTemp  = t;
  curHum   = h;
  curPress = p;
  lastSensorOkMs = millis();
  everReadSensor = true;
  return true;
}
bool sensorFresh(uint32_t now) {
  return everReadSensor && timeReached(now, lastSensorOkMs, SENSOR_STALE_MS);
}

void applyControl() {
  if (cfg.relayMode == MODE_MANUAL_ON)  { setRelay(true);  return; }
  if (cfg.relayMode == MODE_MANUAL_OFF) { setRelay(false); return; }
  if (!cfg.relayEnabled) { setRelay(false); return; }
  if (!sensorFresh(millis()) || !isFiniteF(curTemp) || !isFiniteF(curHum) || !isFiniteF(curPress)) {
    setRelay(false);
    return;
  }

  bool anyAboveMax =
      (curTemp  > cfg.tempMax ) ||
      (curHum   > cfg.humMax  ) ||
      (curPress > cfg.pressMax);

  bool anyBelowMin =
      (curTemp  < cfg.tempMin ) ||
      (curHum   < cfg.humMin  ) ||
      (curPress < cfg.pressMin);

  if (anyAboveMax)      setRelay(true);
  else if (anyBelowMin) setRelay(false);
}

void controlLoopTick() {
  uint32_t now = millis();
  if (!timeReached(now, lastControlMs, CONTROL_PERIOD_MS)) return;
  lastControlMs = now;
  applyControl();
}

static const char PAGE_INDEX[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>BME280 Dashboard</title>
<style>
*{box-sizing:border-box}body{font-family:system-ui,Arial;margin:0;background:#111;color:#eee}
.wrap{max-width:900px;margin:0 auto;padding:16px}h1{font-size:20px;margin:8px 0 16px}
.card{background:#1c1c1c;border-radius:10px;padding:14px;margin-bottom:12px}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(150px,1fr));gap:10px}
.val{font-size:26px;font-weight:600}.lbl{font-size:12px;color:#9aa;text-transform:uppercase;letter-spacing:1px}
.row{display:flex;gap:8px;align-items:center;flex-wrap:wrap;margin:6px 0}
input{background:#222;color:#eee;border:1px solid #333;border-radius:6px;padding:6px;width:90px}
button{background:#2b7de9;color:#fff;border:0;border-radius:6px;padding:8px 14px;font-weight:600;cursor:pointer}
button.warn{background:#c33}button.ok{background:#2a9d3a}
.badge{display:inline-block;padding:3px 8px;border-radius:999px;font-size:12px}
.badge.on{background:#2a9d3a}.badge.off{background:#555}
a{color:#6cb4ff}small{color:#888}
</style></head><body><div class="wrap"><h1>BME280 Dashboard</h1>
<div class="card"><div class="grid">
<div><div class="lbl">Temperature</div><div class="val"><span id="t">--</span> &deg;C</div></div>
<div><div class="lbl">Humidity</div><div class="val"><span id="h">--</span> %</div></div>
<div><div class="lbl">Pressure</div><div class="val"><span id="p">--</span> hPa</div></div>
<div><div class="lbl">Relay</div><div class="val"><span id="rs" class="badge off">--</span></div></div>
</div></div>
<div class="card"><div class="lbl">Relay control</div>
<div class="row"><button onclick="setMode(0)">Auto</button>
<button onclick="setMode(1)">Force ON</button>
<button onclick="setMode(2)">Force OFF</button>
<small>Mode: <span id="mode">--</span></small></div></div>
<div class="card"><div class="lbl">Thresholds</div>
<div class="row">Temp min<input id="tmin"> max<input id="tmax">&deg;C</div>
<div class="row">Hum min<input id="hmin"> max<input id="hmax">%</div>
<div class="row">Press min<input id="pmin"> max<input id="pmax">hPa</div>
<div class="row"><button class="ok" onclick="saveThr()">Save thresholds</button>
<small>Stored in NVS (debounced)</small></div></div>
<div class="card"><div class="lbl">System</div>
<div class="row"><small>Uptime: <span id="up">--</span>s &middot; Heap: <span id="hp">--</span> &middot; Sensor: <span id="sen">--</span></small></div>
<div class="row"><a href="/wifi">WiFi Settings</a> &middot; <a href="/reset" onclick="return confirm('Factory reset?')">Factory Reset</a></div></div>
</div>
<script>
var thrLoaded=false;
function fmt(x,d){return (x===null||x===undefined||isNaN(Number(x)))?'--':Number(x).toFixed(d)}
async function poll(){
 try{
  var r=await fetch('/api/status',{cache:'no-store'}); if(!r.ok) throw 0;
  var j=await r.json();
  document.getElementById('t').textContent=fmt(j.t,2);
  document.getElementById('h').textContent=fmt(j.h,2);
  document.getElementById('p').textContent=fmt(j.p,2);
  var rs=document.getElementById('rs');
  rs.textContent=j.relay?'ON':'OFF'; rs.className='badge '+(j.relay?'on':'off');
  document.getElementById('mode').textContent=['Auto','Force ON','Force OFF'][j.mode]||'?';
  document.getElementById('up').textContent=j.up;
  document.getElementById('hp').textContent=j.heap;
  document.getElementById('sen').textContent=j.bme?(j.fresh?'OK':'stale'):'missing';
  if(!thrLoaded && j.tmin!==undefined && j.tmin!==null){
   document.getElementById('tmin').value=Number(j.tmin).toFixed(1);
   document.getElementById('tmax').value=Number(j.tmax).toFixed(1);
   document.getElementById('hmin').value=Number(j.hmin).toFixed(1);
   document.getElementById('hmax').value=Number(j.hmax).toFixed(1);
   document.getElementById('pmin').value=Number(j.pmin).toFixed(1);
   document.getElementById('pmax').value=Number(j.pmax).toFixed(1);
   thrLoaded=true;
  }
 }catch(e){}
}
async function setMode(m){try{await fetch('/api/mode?m='+m,{method:'POST'});}catch(e){}await poll();}
async function saveThr(){
 var b={tmin:Number(document.getElementById('tmin').value),
        tmax:Number(document.getElementById('tmax').value),
        hmin:Number(document.getElementById('hmin').value),
        hmax:Number(document.getElementById('hmax').value),
        pmin:Number(document.getElementById('pmin').value),
        pmax:Number(document.getElementById('pmax').value)};
 try{
  var r=await fetch('/api/thresholds',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify(b)});
  var j=await r.json();alert(j.ok?'Saved':'Error: '+(j.err||'unknown'));
 }catch(e){alert('Network error');}
}
setInterval(poll,2000); poll();
</script></body></html>)HTML";

static const char PAGE_WIFI[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1">
<title>WiFi Settings</title>
<style>body{font-family:system-ui,Arial;background:#111;color:#eee;margin:0;padding:16px}
.wrap{max-width:600px;margin:0 auto}.card{background:#1c1c1c;padding:14px;border-radius:10px;margin-bottom:12px}
input{width:100%;background:#222;color:#eee;border:1px solid #333;border-radius:6px;padding:8px;margin:4px 0;box-sizing:border-box}
button{background:#2b7de9;color:#fff;border:0;border-radius:6px;padding:8px 14px;font-weight:600;cursor:pointer}
a{color:#6cb4ff}small{color:#888}</style></head><body><div class="wrap">
<h1>WiFi AP Settings</h1>
<div class="card"><div><b>Current SSID:</b> <span id="cs">--</span></div>
<div><b>Connected stations:</b> <span id="st">--</span></div>
<div><b>IP:</b> <span id="ip">--</span></div></div>
<div class="card"><div>SSID</div><input id="ssid" maxlength="31">
<div>Password (8-63 chars)</div><input id="pass" maxlength="63" type="text">
<small>Changing WiFi reboots the device.</small><br><br>
<button onclick="saveW()">Save &amp; Reboot</button></div>
<div class="card"><a href="/">Back to Dashboard</a></div></div>
<script>
async function load(){
 try{
  var r=await fetch('/api/wifi',{cache:'no-store'}); var j=await r.json();
  document.getElementById('cs').textContent=j.ssid;
  document.getElementById('st').textContent=j.sta;
  document.getElementById('ip').textContent=j.ip;
  document.getElementById('ssid').value=j.ssid;
 }catch(e){}
}
async function saveW(){
 var s=document.getElementById('ssid').value.trim();
 var p=document.getElementById('pass').value;
 if(s.length<1||s.length>31){alert('SSID 1-31 chars');return}
 if(p.length<8||p.length>63){alert('Password 8-63 chars');return}
 try{
  var r=await fetch('/api/wifi',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({ssid:s,pass:p})});
  var j=await r.json();
  alert(j.ok?'Saved, rebooting...':'Error');
 }catch(e){alert('Network error');}
}
load();
</script></body></html>)HTML";

static const char PAGE_RESET[] PROGMEM = R"HTML(<!DOCTYPE html><html><head>
<meta charset="utf-8"><title>Factory Reset</title></head>
<body style="font-family:Arial;background:#111;color:#eee;text-align:center;padding:40px">
<h2>Factory reset triggered</h2><p>Rebooting &mdash; reconnect to <b>ESP8266_BME280</b></p>
</body></html>)HTML";

void sendNoCache(const char* ct, const String& body, int code = 200) {
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(code, ct, body);
}

void handleRoot() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE_INDEX);
}
void handleWifiPage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE_WIFI);
}
void handleResetPage() {
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "text/html", PAGE_RESET);
  nvsDirty = false;
  scheduleFactoryReset(800);
}

void handleStatus() {
  StaticJsonDocument<768> d;
  if (isFiniteF(curTemp))  d["t"] = curTemp;  else d["t"] = nullptr;
  if (isFiniteF(curHum))   d["h"] = curHum;   else d["h"] = nullptr;
  if (isFiniteF(curPress)) d["p"] = curPress; else d["p"] = nullptr;
  d["relay"] = relayState;
  d["mode"]  = cfg.relayMode;
  d["tmin"]  = cfg.tempMin;  d["tmax"] = cfg.tempMax;
  d["hmin"]  = cfg.humMin;   d["hmax"] = cfg.humMax;
  d["pmin"]  = cfg.pressMin; d["pmax"] = cfg.pressMax;
  d["up"]    = millis() / 1000UL;
  d["heap"]  = ESP.getFreeHeap();
  d["bme"]   = bmeOk;
  d["fresh"] = sensorFresh(millis());
  d["serr"]  = sensorErrCount;
  d["nvs"]   = nvsOpened;
  String out;
  out.reserve(640);
  serializeJson(d, out);
  sendNoCache("application/json", out);
}

void handleThresholds() {
  if (!server.hasArg("plain")) {
    sendNoCache("application/json", F("{\"ok\":false,\"err\":\"no body\"}"), 400);
    return;
  }
  StaticJsonDocument<256> d;
  if (deserializeJson(d, server.arg("plain"))) {
    sendNoCache("application/json", F("{\"ok\":false,\"err\":\"json\"}"), 400);
    return;
  }
  PersistConfig next = cfg;
  if (d.containsKey("tmin")) next.tempMin  = d["tmin"].as<float>();
  if (d.containsKey("tmax")) next.tempMax  = d["tmax"].as<float>();
  if (d.containsKey("hmin")) next.humMin   = d["hmin"].as<float>();
  if (d.containsKey("hmax")) next.humMax   = d["hmax"].as<float>();
  if (d.containsKey("pmin")) next.pressMin = d["pmin"].as<float>();
  if (d.containsKey("pmax")) next.pressMax = d["pmax"].as<float>();
  next.magic = CFG_MAGIC;
  if (!validateConfig(next)) {
    sendNoCache("application/json", F("{\"ok\":false,\"err\":\"range\"}"), 400);
    return;
  }
  cfg = next;
  markDirty();
  applyControl();
  sendNoCache("application/json", F("{\"ok\":true}"));
}

void handleMode() {
  if (!server.hasArg("m")) {
    sendNoCache("application/json", F("{\"ok\":false}"), 400);
    return;
  }
  const String& arg = server.arg("m");
  bool numeric = arg.length() > 0 && arg.length() <= 2;
  for (size_t i = 0; numeric && i < arg.length(); i++) {
    if (arg[i] < '0' || arg[i] > '9') numeric = false;
  }
  if (!numeric) {
    sendNoCache("application/json", F("{\"ok\":false}"), 400);
    return;
  }
  long m = arg.toInt();
  if (m < 0 || m > 2) {
    sendNoCache("application/json", F("{\"ok\":false}"), 400);
    return;
  }
  cfg.relayMode = (uint8_t)m;
  markDirty();
  applyControl();
  sendNoCache("application/json", F("{\"ok\":true}"));
}

void handleWifiGet() {
  StaticJsonDocument<256> d;
  d["ssid"] = WiFi.softAPSSID();
  d["sta"]  = WiFi.softAPgetStationNum();
  d["ip"]   = WiFi.softAPIP().toString();
  String out; out.reserve(256);
  serializeJson(d, out);
  sendNoCache("application/json", out);
}

void handleWifiPost() {
  if (!server.hasArg("plain")) {
    sendNoCache("application/json", F("{\"ok\":false}"), 400); return;
  }
  StaticJsonDocument<256> d;
  if (deserializeJson(d, server.arg("plain"))) {
    sendNoCache("application/json", F("{\"ok\":false}"), 400); return;
  }
  const char* sPtr = d["ssid"] | (const char*)nullptr;
  const char* pPtr = d["pass"] | (const char*)nullptr;
  if (!sPtr || !pPtr) {
    sendNoCache("application/json", F("{\"ok\":false}"), 400); return;
  }
  String s(sPtr), p(pPtr);
  if (s.length() < 1 || s.length() > 31 || p.length() < 8 || p.length() > 63) {
    sendNoCache("application/json", F("{\"ok\":false}"), 400); return;
  }
  if (prefs.begin("bme280cfg", false)) {
    prefs.putString("ssid", s);
    prefs.putString("pass", p);
    prefs.end();
  }
  sendNoCache("application/json", F("{\"ok\":true}"));
  scheduleRestart(400);
}

void redirectToPortal() {
  String url = String("http://") + WiFi.softAPIP().toString() + "/";
  server.sendHeader("Location", url, true);
  server.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate");
  server.send(302, "text/plain", "");
}
void handleCaptiveOk()   { redirectToPortal(); }
void handleGenerate204() { server.send(204, "text/plain", ""); }
void handleFavicon()     { server.send(204, "text/plain", ""); }
void handleNotFound() {
  if (server.uri().startsWith("/api/")) {
    sendNoCache("application/json", F("{\"ok\":false,\"err\":\"not found\"}"), 404);
    return;
  }
  redirectToPortal();
}

void startAP() {
  String ssid = AP_SSID_DEFAULT;
  String pass = AP_PASS_DEFAULT;
  if (prefs.begin("bme280cfg", true)) {
    String s = prefs.getString("ssid", "");
    String p = prefs.getString("pass", "");
    prefs.end();
    bool customValid = (s.length() >= 1 && s.length() <= 31 &&
                        p.length() >= 8 && p.length() <= 63);
    if (customValid) { ssid = s; pass = p; }
  }
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPdisconnect(true);
  delay(50);
  bool ok = WiFi.softAP(ssid.c_str(), pass.c_str(), AP_CHANNEL, AP_HIDDEN, AP_MAX_CONN);
  if (!ok && (ssid != AP_SSID_DEFAULT || pass != AP_PASS_DEFAULT)) 
    WiFi.softAP(AP_SSID_DEFAULT, AP_PASS_DEFAULT, AP_CHANNEL, AP_HIDDEN, AP_MAX_CONN);
    if (prefs.begin("bme280cfg", false)) {
      prefs.remove("ssid");
      prefs.remove("pass");
      prefs.end();
    }
  }
  uint32_t start = millis();
  while (WiFi.softAPIP() == IPAddress(0, 0, 0, 0) &&
         !timeReached(millis(), start, 500)) {
    delay(10);
  }
}

void startDns() {
  if (dnsStarted) return;
  if (WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) return;
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", WiFi.softAPIP());
  dnsStarted = true;
}

void setup() {
  digitalWrite(STATUS_LED_PIN, HIGH);
  pinMode(STATUS_LED_PIN, OUTPUT);
  ledWrite(false);
  loadConfig();
  digitalWrite(RELAY_PIN, cfg.relayInvert ? HIGH : LOW);
  pinMode(RELAY_PIN, OUTPUT);
  pinMode(BOOT_BTN_PIN, INPUT_PULLUP);
  relayState = false;
  applyRelayPhysical(false);
  initSensor();
  startAP();
  startDns();
  server.on("/",                    HTTP_GET,  handleRoot);
  server.on("/wifi",                HTTP_GET,  handleWifiPage);
  server.on("/reset",               HTTP_GET,  handleResetPage);
  server.on("/api/status",          HTTP_GET,  handleStatus);
  server.on("/api/thresholds",      HTTP_POST, handleThresholds);
  server.on("/api/mode",            HTTP_POST, handleMode);
  server.on("/api/wifi",            HTTP_GET,  handleWifiGet);
  server.on("/api/wifi",            HTTP_POST, handleWifiPost);
  server.on("/generate_204",        HTTP_GET,  handleGenerate204);
  server.on("/gen_204",             HTTP_GET,  handleGenerate204);
  server.on("/hotspot-detect.html", HTTP_GET,  handleCaptiveOk);
  server.on("/connecttest.txt",     HTTP_GET,  handleCaptiveOk);
  server.on("/redirect",            HTTP_GET,  handleCaptiveOk);
  server.on("/canonical.html",      HTTP_GET,  handleCaptiveOk);
  server.on("/ncsi.txt",            HTTP_GET,  handleCaptiveOk);
  server.on("/favicon.ico",         HTTP_GET,  handleFavicon);
  server.onNotFound(handleNotFound);
  server.begin();
  if (MDNS.begin("bme280")) {
    MDNS.addService("http", "tcp", 80);
    mdnsStarted = true;
  }
  lastSensorOkMs  = 0;
  lastSensorTryMs = millis();
  lastControlMs   = millis();
  lastNvsSaveMs   = millis();
  lastMdnsMs      = millis();
}

void loop() {
  uint32_t now = millis();
  if (pendingFactoryReset) {
    server.handleClient();
    if (timeAtOrAfter(now, pendingFactoryResetAt)) {
      pendingFactoryReset = false;
      factoryResetNow();
    }
    return;
  }
  if (!dnsStarted) startDns();
  server.handleClient();
  if (dnsStarted) dnsServer.processNextRequest();
  if (mdnsStarted && timeReached(now, lastMdnsMs, MDNS_UPDATE_MS)) {
    lastMdnsMs = now;
    MDNS.update();
  }
  if (pendingRestart && timeAtOrAfter(now, pendingRestartAt)) {
    pendingRestart = false;
    ESP.restart();
  }
  if (timeReached(now, lastSensorTryMs, SENSOR_PERIOD_MS)) {
    lastSensorTryMs = now;
    if (readSensor()) {
      sensorErrStreak = 0;
    } else {
      sensorErrCount++;
      if (sensorErrStreak < 255) sensorErrStreak++;
      if (sensorErrStreak >= 5) {
        sensorErrStreak = 0;
        initSensor();
      }
    }
  }
  controlLoopTick();
  if (nvsDirty && timeReached(now, lastNvsSaveMs, NVS_SAVE_DEBOUNCE)) {
    saveConfigNow();
  }
  if (!factoryResetArmed) {
    bool pressed = (digitalRead(BOOT_BTN_PIN) == LOW);
    if (pressed) {
      if (!bootPressed) { bootPressed = true; bootPressStartMs = now; }
      else if (timeReached(now, bootPressStartMs, FACTORY_HOLD_MS)) {
        ledWrite(true);
        scheduleFactoryReset(0);
      }
    } else {
      bootPressed = false;
    }
  }
  if (!factoryResetArmed) {
    static uint32_t ledBlinkMs = 0;
    static bool     ledPhase   = false;
    bool healthy = bmeOk && sensorFresh(now) && isFiniteF(curTemp);
    if (healthy) {
      ledWrite(false);
      ledPhase = false;
    } else {
      if (timeReached(now, ledBlinkMs, 500)) {
        ledBlinkMs = now;
        ledPhase = !ledPhase;
        ledWrite(ledPhase);
      }
    }
  }
}


