#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>

#define SDA_PIN 5
#define SCL_PIN 4
#define RELAY_PIN 12
#define BOOT_BUTTON_PIN 0
#define RELAY_ACTIVE_LOW 0

struct Config {
  uint16_t magic;
  int64_t tempHigh;
  int64_t tempLow;
  int64_t humHigh;
  int64_t humLow;
  int64_t presHigh;
  int64_t presLow;
  uint8_t relayMode;
};

Config cfg;

static inline void relayWrite(bool on) {
#if RELAY_ACTIVE_LOW
  digitalWrite(RELAY_PIN, on ? LOW : HIGH);
#else
  digitalWrite(RELAY_PIN, on ? HIGH : LOW);
#endif
}

const char* AP_SSID = "ESP8266_BME280";
const char* AP_PASS = "12345678";

#define EEPROM_SIZE 128
#define ADDR_CONFIG 4
#define EEPROM_MAGIC 0xB5C0u
#define READ_INTERVAL 2000UL
#define DNS_PORT 53

Adafruit_BME280 bme;
bool bmeOK = false;

ESP8266WebServer server(80);
DNSServer dnsServer;

int64_t lastTemp = 0, lastHum = 0, lastPres = 0;
uint32_t lastRead = 0;
bool hasValidReading = false;

uint32_t buttonDownTime = 0;
bool buttonDown = false;
bool factoryResetTriggered = false;

static uint32_t msPrev = 0;
static uint64_t msOverflow = 0;
static uint64_t uptimeMsCache = 0;

static inline bool timeReached(uint32_t now, uint32_t start, uint32_t interval) {
  return (uint32_t)(now - start) >= interval;
}

uint32_t crc32Update(uint32_t crc, const uint8_t* data, size_t len) {
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (int j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xEDB88320u;
      else crc >>= 1;
    }
  }
  return crc;
}

uint32_t cfgCrc(const Config& c) {
  uint32_t crc = 0xFFFFFFFFu;
  crc = crc32Update(crc, (const uint8_t*)&c.magic, sizeof(c.magic));
  crc = crc32Update(crc, (const uint8_t*)&c.tempHigh, sizeof(c.tempHigh));
  crc = crc32Update(crc, (const uint8_t*)&c.tempLow, sizeof(c.tempLow));
  crc = crc32Update(crc, (const uint8_t*)&c.humHigh, sizeof(c.humHigh));
  crc = crc32Update(crc, (const uint8_t*)&c.humLow, sizeof(c.humLow));
  crc = crc32Update(crc, (const uint8_t*)&c.presHigh, sizeof(c.presHigh));
  crc = crc32Update(crc, (const uint8_t*)&c.presLow, sizeof(c.presLow));
  crc = crc32Update(crc, (const uint8_t*)&c.relayMode, sizeof(c.relayMode));
  return ~crc;
}

void loadDefaults() {
  cfg.magic = EEPROM_MAGIC;
  cfg.tempHigh = 3000;
  cfg.tempLow = 1000;
  cfg.humHigh = 7000;
  cfg.humLow = 3000;
  cfg.presHigh = 102000;
  cfg.presLow = 98000;
  cfg.relayMode = 1;
}

bool saveConfig() {
  cfg.magic = EEPROM_MAGIC;
  EEPROM.put(ADDR_CONFIG, cfg);
  uint32_t crc = cfgCrc(cfg);
  EEPROM.put(ADDR_CONFIG + sizeof(Config), crc);
  return EEPROM.commit();
}

void loadConfig() {
  EEPROM.get(ADDR_CONFIG, cfg);
  uint32_t stored = 0;
  EEPROM.get(ADDR_CONFIG + sizeof(Config), stored);
  if (cfg.magic != EEPROM_MAGIC || stored != cfgCrc(cfg)) {
    loadDefaults();
    saveConfig();
  }
}

void factoryResetNow() {
  factoryResetTriggered = true;
  loadDefaults();
  saveConfig();
  WiFi.disconnect(true);
  delay(200);
  ESP.restart();
}

const char PAGE_HTML[] PROGMEM = R"HTML(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1,maximum-scale=1,user-scalable=no">
<title>ESP8266 BME280</title>
<style>
*{box-sizing:border-box;-webkit-tap-highlight-color:transparent}
body{font-family:-apple-system,BlinkMacSystemFont,'Segoe UI',Roboto,sans-serif;margin:0;padding:12px;background:#0f172a;color:#e2e8f0;font-size:16px}
.card{background:#1e293b;border-radius:12px;padding:16px;margin-bottom:12px;box-shadow:0 2px 8px rgba(0,0,0,.3)}
h1{font-size:1.3em;margin:0 0 12px}
h2{font-size:1.1em;margin:0 0 10px;color:#94a3b8}
.row{display:flex;justify-content:space-between;align-items:center;padding:8px 0;border-bottom:1px solid #334155}
.row:last-child{border-bottom:none}
.val{font-weight:700;font-size:1.15em;color:#38bdf8}
.grid{display:grid;grid-template-columns:1fr 1fr;gap:8px}
label{display:block;font-size:.85em;color:#94a3b8;margin-bottom:4px}
input,select{width:100%;padding:10px;border-radius:8px;border:1px solid #334155;background:#0f172a;color:#e2e8f0;font-size:16px;-webkit-appearance:none;appearance:none}
button{width:100%;padding:12px;border-radius:8px;border:none;background:#0284c7;color:#fff;font-size:16px;font-weight:600;cursor:pointer;margin-top:8px}
button:active{background:#0369a1}
button.danger{background:#dc2626}
button.danger:active{background:#b91c1c}
.badge{display:inline-block;padding:3px 8px;border-radius:6px;font-size:.75em;font-weight:700}
.on{background:#16a34a;color:#fff}
.off{background:#475569;color:#e2e8f0}
.toast{position:fixed;bottom:20px;left:50%;transform:translateX(-50%) translateY(120px);background:#334155;padding:12px 20px;border-radius:8px;transition:transform .3s;z-index:999;font-weight:600}
.toast.show{transform:translateX(-50%) translateY(0)}
@media(min-width:600px){body{max-width:640px;margin:0 auto;padding:24px}}
</style>
</head>
<body>
<h1>BME280 Dashboard</h1>
<div class="card">
<h2>Live Readings</h2>
<div class="row"><span>Temperature</span><span class="val" id="temp">--</span></div>
<div class="row"><span>Humidity</span><span class="val" id="hum">--</span></div>
<div class="row"><span>Pressure</span><span class="val" id="pres">--</span></div>
<div class="row"><span>Relay</span><span class="val"><span id="relayState" class="badge off">--</span></span></div>
</div>
<div class="card">
<h2>Thresholds</h2>
<div class="grid">
<div><label>Temp High (C)</label><input type="number" id="tH" step="0.01"></div>
<div><label>Temp Low (C)</label><input type="number" id="tL" step="0.01"></div>
<div><label>Hum High (%)</label><input type="number" id="hH" step="0.01"></div>
<div><label>Hum Low (%)</label><input type="number" id="hL" step="0.01"></div>
<div><label>Pres High (hPa)</label><input type="number" id="pH" step="0.01"></div>
<div><label>Pres Low (hPa)</label><input type="number" id="pL" step="0.01"></div>
</div>
<label style="margin-top:10px">Relay Mode</label>
<select id="rMode">
<option value="0">Off</option>
<option value="1">Auto (threshold)</option>
<option value="2">On</option>
</select>
<button id="saveBtn">Save Settings</button>
</div>
<div class="card">
<h2>System</h2>
<div class="row"><span>Uptime</span><span class="val" id="uptime">--</span></div>
<div class="row"><span>Free Heap</span><span class="val" id="heap">--</span></div>
<div class="row"><span>BME280</span><span class="val"><span id="bmeStatus" class="badge off">--</span></span></div>
<button class="danger" id="resetBtn">Factory Reset</button>
</div>
<div class="toast" id="toast"></div>
<script>
(function(){
var pollTimer=null,busy=false;
function $(id){return document.getElementById(id)}
function toast(msg){var t=$('toast');t.textContent=msg;t.classList.add('show');clearTimeout(t._h);t._h=setTimeout(function(){t.classList.remove('show')},2000)}
function fnum(v,d){if(v===null||v===undefined||isNaN(v))return '--';return Number(v).toFixed(d)}
function fmtUptime(s){s=Math.floor(s);var d=Math.floor(s/86400);s%=86400;var h=Math.floor(s/3600);s%=3600;var m=Math.floor(s/60);s%=60;var r='';if(d)r+=d+'d ';if(h||d)r+=h+'h ';r+=m+'m '+s+'s';return r}
function fetchStatus(){
  if(busy)return;busy=true;
  var x=new XMLHttpRequest();
  x.open('GET','/api/status',true);x.timeout=5000;
  x.onload=function(){busy=false;
    if(x.status!==200)return;
    try{
      var d=JSON.parse(x.responseText);
      $('temp').textContent=fnum(d.t/100,2)+' C';
      $('hum').textContent=fnum(d.h/100,2)+' %';
      $('pres').textContent=fnum(d.p/100,2)+' hPa';
      var rs=$('relayState');rs.textContent=d.r?'ON':'OFF';rs.className='badge '+(d.r?'on':'off');
      $('uptime').textContent=fmtUptime(d.u);
      $('heap').textContent=(d.fh/1024).toFixed(1)+' KB';
      var bs=$('bmeStatus');bs.textContent=d.bme?'OK':'FAIL';bs.className='badge '+(d.bme?'on':'off');
    }catch(e){}
  };
  x.onerror=x.ontimeout=function(){busy=false};
  x.send();
}
function loadConfig(){
  var x=new XMLHttpRequest();
  x.open('GET','/api/config',true);x.timeout=5000;
  x.onload=function(){
    if(x.status!==200)return;
    try{
      var c=JSON.parse(x.responseText);
      $('tH').value=(c.tH/100).toFixed(2);
      $('tL').value=(c.tL/100).toFixed(2);
      $('hH').value=(c.hH/100).toFixed(2);
      $('hL').value=(c.hL/100).toFixed(2);
      $('pH').value=(c.pH/100).toFixed(2);
      $('pL').value=(c.pL/100).toFixed(2);
      $('rMode').value=String(c.m);
    }catch(e){}
  };
  x.send();
}
function saveConfig(){
  var body={tH:Math.round(parseFloat($('tH').value||0)*100),
            tL:Math.round(parseFloat($('tL').value||0)*100),
            hH:Math.round(parseFloat($('hH').value||0)*100),
            hL:Math.round(parseFloat($('hL').value||0)*100),
            pH:Math.round(parseFloat($('pH').value||0)*100),
            pL:Math.round(parseFloat($('pL').value||0)*100),
            m:parseInt($('rMode').value||0,10)};
  var x=new XMLHttpRequest();
  x.open('POST','/api/config',true);x.setRequestHeader('Content-Type','application/json');x.timeout=5000;
  x.onload=function(){toast(x.status===200?'Saved':'Save failed')};
  x.onerror=x.ontimeout=function(){toast('Save failed')};
  x.send(JSON.stringify(body));
}
$('saveBtn').addEventListener('click',saveConfig);
$('resetBtn').addEventListener('click',function(){
  if(confirm('Factory reset and reboot?')){
    var x=new XMLHttpRequest();
    x.open('POST','/api/reset',true);x.timeout=5000;
    x.onload=function(){toast('Resetting...')};
    x.send();
  }
});
document.addEventListener('visibilitychange',function(){
  if(document.hidden){if(pollTimer){clearInterval(pollTimer);pollTimer=null}}
  else{if(!pollTimer){fetchStatus();pollTimer=setInterval(fetchStatus,2000)}}
});
loadConfig();fetchStatus();pollTimer=setInterval(fetchStatus,2000);
})();
</script>
</body>
</html>
)HTML";

void handleRoot() {
  server.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
  server.sendHeader("Pragma", "no-cache");
  server.sendHeader("Connection", "close");
  server.send_P(200, "text/html", PAGE_HTML);
}

void handleCaptive() {
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

void handleStatus() {
  char buf[240];
  int relay = (digitalRead(RELAY_PIN) == (RELAY_ACTIVE_LOW ? LOW : HIGH)) ? 1 : 0;
  uint64_t up = uptimeMsCache / 1000ULL;
  snprintf_P(buf, sizeof(buf),
    PSTR("{\"t\":%lld,\"h\":%lld,\"p\":%lld,\"r\":%d,\"u\":%llu,\"fh\":%u,\"bme\":%d}"),
    (long long)lastTemp, (long long)lastHum, (long long)lastPres,
    relay, (unsigned long long)up, (unsigned)ESP.getFreeHeap(), bmeOK?1:0);
  server.send(200, "application/json", buf);
}

void handleGetConfig() {
  char buf[220];
  snprintf_P(buf, sizeof(buf),
    PSTR("{\"tH\":%lld,\"tL\":%lld,\"hH\":%lld,\"hL\":%lld,\"pH\":%lld,\"pL\":%lld,\"m\":%u}"),
    (long long)cfg.tempHigh, (long long)cfg.tempLow,
    (long long)cfg.humHigh, (long long)cfg.humLow,
    (long long)cfg.presHigh, (long long)cfg.presLow,
    (unsigned)cfg.relayMode);
  server.send(200, "application/json", buf);
}

bool parseJsonInt64(const String& body, const char* key, int64_t& out) {
  char needle[24];
  int n = snprintf(needle, sizeof(needle), "\"%s\":", key);
  if (n <= 0 || n >= (int)sizeof(needle)) return false;
  const char* p = strstr(body.c_str(), needle);
  if (!p) return false;
  p += n;
  while (*p == ' ' || *p == '\t') p++;
  bool neg = false;
  if (*p == '-' || *p == '+') { neg = (*p == '-'); p++; }
  int64_t v = 0;
  int digits = 0;
  const int64_t MAX = 0x7FFFFFFFFFFFFFFFLL;
  while (*p >= '0' && *p <= '9') {
    int d = *p - '0';
    if (v > (MAX - d) / 10) return false;
    v = v * 10 + d;
    digits++;
    p++;
  }
  if (!digits) return false;
  out = neg ? -v : v;
  return true;
}

void handlePostConfig() {
  if (!server.hasArg("plain")) { server.send(400, "application/json", "{\"e\":\"no body\"}"); return; }
  String body = server.arg("plain");
  Config n = cfg;
  int64_t v;
  bool ok = false;
  if (parseJsonInt64(body, "tH", v)) { n.tempHigh = v; ok = true; }
  if (parseJsonInt64(body, "tL", v)) { n.tempLow = v; ok = true; }
  if (parseJsonInt64(body, "hH", v)) { n.humHigh = v; ok = true; }
  if (parseJsonInt64(body, "hL", v)) { n.humLow = v; ok = true; }
  if (parseJsonInt64(body, "pH", v)) { n.presHigh = v; ok = true; }
  if (parseJsonInt64(body, "pL", v)) { n.presLow = v; ok = true; }
  if (parseJsonInt64(body, "m", v)) {
    if (v < 0 || v > 2) { server.send(400, "application/json", "{\"e\":\"bad mode\"}"); return; }
    n.relayMode = (uint8_t)v; ok = true;
  }
  if (n.tempHigh < n.tempLow) n.tempHigh = n.tempLow;
  if (n.humHigh < n.humLow) n.humHigh = n.humLow;
  if (n.presHigh < n.presLow) n.presHigh = n.presLow;
  if (!ok) { server.send(400, "application/json", "{\"e\":\"bad\"}"); return; }
  cfg = n;
  bool persisted = saveConfig();
  applyRelay();
  if (persisted) {
    server.send(200, "application/json", "{\"ok\":1}");
  } else {
    server.send(500, "application/json", "{\"ok\":0,\"e\":\"eeprom\"}");
  }
}

void handleReset() {
  server.send(200, "application/json", "{\"ok\":1}");
  server.client().flush();
  server.client().stop();
  delay(100);
  factoryResetNow();
}

void handleNotFound() {
  String uri = server.uri();
  if (uri.startsWith("/api/")) {
    server.send(404, "application/json", "{\"e\":\"not found\"}");
    return;
  }
  if (server.method() == HTTP_GET) handleRoot();
  else server.send(404, "text/plain", "");
}

void setupServer() {
  server.on("/", HTTP_GET, handleRoot);
  server.on("/api/status", HTTP_GET, handleStatus);
  server.on("/api/config", HTTP_GET, handleGetConfig);
  server.on("/api/config", HTTP_POST, handlePostConfig);
  server.on("/api/reset", HTTP_POST, handleReset);
  server.on("/generate_204", HTTP_GET, handleCaptive);
  server.on("/gen_204", HTTP_GET, handleCaptive);
  server.on("/hotspot-detect.html", HTTP_GET, handleCaptive);
  server.on("/library/test/success.html", HTTP_GET, handleCaptive);
  server.on("/connecttest.txt", HTTP_GET, handleCaptive);
  server.on("/ncsi.txt", HTTP_GET, handleCaptive);
  server.on("/fwlink", HTTP_GET, handleCaptive);
  server.onNotFound(handleNotFound);
}

void setupWiFi() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  WiFi.softAPConfig(IPAddress(192,168,4,1), IPAddress(192,168,4,1), IPAddress(255,255,255,0));
  WiFi.softAP(AP_SSID, AP_PASS, 1, 0, 4);
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(DNS_PORT, "*", IPAddress(192,168,4,1));
}

void setupBME() {
  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);
  Wire.setClockStretchLimit(1500);
  bmeOK = bme.begin(0x76, &Wire);
  if (!bmeOK) bmeOK = bme.begin(0x77, &Wire);
  if (bmeOK) {
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X2,
                    Adafruit_BME280::SAMPLING_X16,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_X16,
                    Adafruit_BME280::STANDBY_MS_1000);
  }
}

void readSensors() {
  if (!bmeOK) return;
  bme.takeForcedMeasurement();
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure();
  if (isnan(t) || isnan(h) || isnan(p)) return;
  if (t < -40.0f || t > 85.0f) return;
  if (h < 0.0f || h > 100.0f) return;
  if (p < 30000.0f || p > 110000.0f) return;
  lastTemp = (int64_t)llround((double)t * 100.0);
  lastHum  = (int64_t)llround((double)h * 100.0);
  lastPres = (int64_t)llround((double)p);
  hasValidReading = true;
}

void applyRelay() {
  bool on;
  if (cfg.relayMode == 0) on = false;
  else if (cfg.relayMode == 2) on = true;
  else if (!hasValidReading) on = false;
  else {
    on = (lastTemp >= cfg.tempHigh) || (lastTemp <= cfg.tempLow)
      || (lastHum >= cfg.humHigh)   || (lastHum <= cfg.humLow)
      || (lastPres >= cfg.presHigh) || (lastPres <= cfg.presLow);
  }
  relayWrite(on);
}

void pollButton() {
  uint32_t now = millis();
  int v = digitalRead(BOOT_BUTTON_PIN);
  if (v == LOW && !buttonDown) { buttonDown = true; buttonDownTime = now; }
  else if (v == HIGH && buttonDown) { buttonDown = false; }
  else if (v == LOW && buttonDown && !factoryResetTriggered) {
    if (timeReached(now, buttonDownTime, 5000UL)) factoryResetNow();
  }
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  relayWrite(false);
  pinMode(BOOT_BUTTON_PIN, INPUT_PULLUP);
  EEPROM.begin(EEPROM_SIZE);
  loadConfig();
  setupWiFi();
  setupServer();
  setupBME();
  server.begin();
  lastRead = millis() - READ_INTERVAL;
  msPrev = millis();
  msOverflow = 0;
  uptimeMsCache = 0;
}

void loop() {
  uint32_t now = millis();
  if (now < msPrev) msOverflow += 0x100000000ULL;
  msPrev = now;
  uptimeMsCache = msOverflow + now;

  dnsServer.processNextRequest();
  server.handleClient();
  pollButton();

  if (timeReached(now, lastRead, READ_INTERVAL)) {
    lastRead = now;
    readSensors();
    applyRelay();
  }
  yield();
}
