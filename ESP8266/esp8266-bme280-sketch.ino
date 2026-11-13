#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <ESP8266HTTPUpdateServer.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>
#include <DNSServer.h>

#define RELAY_PIN D6
#define BOOT_BUTTON 0
#define SDA_PIN D2
#define SCL_PIN D1

#define EEPROM_SIZE 512
#define CONFIG_MAGIC 0xB3E28001UL
#define CONFIG_VERSION 1
#define NVS_WRITE_MIN_INTERVAL 3600000UL
#define SENSOR_READ_INTERVAL 5000UL
#define SENSOR_REINIT_INTERVAL 30000UL
#define WIFI_CHECK_INTERVAL 10000UL
#define BUTTON_CHECK_INTERVAL 50UL
#define BUTTON_HOLD_TIME 5000UL

struct __attribute__((packed)) Config {
  uint32_t magic;
  uint16_t version;
  uint16_t crc;
  char apSSID[33];
  char apPassword[65];
  uint8_t apChannel;
  uint8_t apHidden;
  uint8_t apMaxConn;
  uint32_t relayValue;
  uint32_t relayMode;
  float tempThreshold;
  float humThreshold;
  float presThreshold;
  uint32_t sampleInterval;
  uint16_t sensorAddress;
  uint8_t reserved[32];
};

static Config config;
static Config pendingConfig;
static bool configDirty = false;
static uint32_t lastConfigWrite = 0;

static Adafruit_BME280 bme;
static bool bmeReady = false;
static bool sensorValid = false;
static uint8_t sensorFailCount = 0;
static uint32_t lastSensorReinit = 0;
static float lastTemp = 0, lastHum = 0, lastPres = 0;
static uint32_t lastSensorRead = 0;
static uint32_t lastWifiCheck = 0;
static uint32_t lastButtonCheck = 0;
static uint32_t buttonPressStart = 0;
static bool buttonPressed = false;
static bool relayState = false;
static bool factoryResetRequested = false;
static bool apRunning = false;

static ESP8266WebServer server(80);
static DNSServer dnsServer;
static ESP8266HTTPUpdateServer httpUpdater;

static String cachedStatusJson;
static uint32_t lastStatusBuild = 0;
static bool statusDirty = true;

static uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= data[i];
    for (uint8_t j = 0; j < 8; j++) {
      if (crc & 1) crc = (crc >> 1) ^ 0xA001;
      else crc >>= 1;
    }
  }
  return crc;
}

static uint16_t computeConfigCrc(const Config& c) {
  Config tmp = c;
  tmp.crc = 0;
  return crc16((const uint8_t*)&tmp, sizeof(Config));
}

static void setDefaults(Config& c) {
  memset(&c, 0, sizeof(c));
  c.magic = CONFIG_MAGIC;
  c.version = CONFIG_VERSION;
  strncpy(c.apSSID, "ESP8266_BME280", sizeof(c.apSSID) - 1);
  strncpy(c.apPassword, "12345678", sizeof(c.apPassword) - 1);
  c.apSSID[32] = 0;
  c.apPassword[64] = 0;
  c.apChannel = 1;
  c.apHidden = 0;
  c.apMaxConn = 4;
  c.relayValue = 1;
  c.relayMode = 0;
  c.tempThreshold = 30.0f;
  c.humThreshold = 70.0f;
  c.presThreshold = 1000.0f;
  c.sampleInterval = SENSOR_READ_INTERVAL;
  c.sensorAddress = 0x76;
  c.crc = computeConfigCrc(c);
}

static bool loadConfig(Config& c) {
  EEPROM.get(0, c);
  if (c.magic != CONFIG_MAGIC) return false;
  if (c.version != CONFIG_VERSION) return false;
  uint16_t saved = c.crc;
  c.crc = 0;
  uint16_t calc = crc16((const uint8_t*)&c, sizeof(Config));
  c.crc = saved;
  if (saved != calc) return false;
  if (c.apSSID[0] == 0 || strlen(c.apSSID) > 32) return false;
  if (strlen(c.apPassword) > 64) return false;
  if (c.apPassword[0] != 0 && strlen(c.apPassword) < 8) return false;
  if (c.apChannel < 1 || c.apChannel > 13) c.apChannel = 1;
  if (c.apMaxConn < 1 || c.apMaxConn > 8) c.apMaxConn = 4;
  if (c.sampleInterval < 1000) c.sampleInterval = SENSOR_READ_INTERVAL;
  if (c.sampleInterval > 600000) c.sampleInterval = 600000;
  if (c.sensorAddress != 0x76 && c.sensorAddress != 0x77) c.sensorAddress = 0x76;
  if (c.relayMode > 4) c.relayMode = 0;
  if (c.relayValue > 1) c.relayValue = 1;
  if (isnan(c.tempThreshold)) c.tempThreshold = 30.0f;
  if (isnan(c.humThreshold)) c.humThreshold = 70.0f;
  if (isnan(c.presThreshold)) c.presThreshold = 1000.0f;
  if (c.tempThreshold < -40) c.tempThreshold = -40;
  if (c.tempThreshold > 85) c.tempThreshold = 85;
  if (c.humThreshold < 0) c.humThreshold = 0;
  if (c.humThreshold > 100) c.humThreshold = 100;
  if (c.presThreshold < 300) c.presThreshold = 300;
  if (c.presThreshold > 1100) c.presThreshold = 1100;
  uint16_t fixedCrc = computeConfigCrc(c);
  if (fixedCrc != c.crc) {
    c.crc = fixedCrc;
    EEPROM.put(0, c);
    EEPROM.commit();
  }
  return true;
}

static bool saveConfigNow(Config& c) {
  c.magic = CONFIG_MAGIC;
  c.version = CONFIG_VERSION;
  c.crc = computeConfigCrc(c);
  EEPROM.put(0, c);
  bool ok = EEPROM.commit();
  if (ok) {
    config = c;
    pendingConfig = c;
    configDirty = false;
    lastConfigWrite = millis();
    statusDirty = true;
  }
  return ok;
}

static void requestConfigSave(const Config& c) {
  pendingConfig = c;
  configDirty = true;
  config = c;
}

static void tickConfigSave(uint32_t now) {
  if (!configDirty) return;
  uint32_t elapsed = (uint32_t)(now - lastConfigWrite);
  if (elapsed < NVS_WRITE_MIN_INTERVAL) return;
  if (saveConfigNow(pendingConfig)) {
    configDirty = false;
    statusDirty = true;
  }
}

static void applyRelay() {
  bool target = false;
  switch (config.relayMode) {
    case 0: target = (config.relayValue != 0); break;
    case 1: target = sensorValid && (lastTemp >= config.tempThreshold); break;
    case 2: target = sensorValid && (lastHum >= config.humThreshold); break;
    case 3: target = sensorValid && (lastPres <= config.presThreshold && lastPres >= 300); break;
    case 4: target = sensorValid && ((lastTemp >= config.tempThreshold) || (lastHum >= config.humThreshold)); break;
    default: target = false;
  }
  if (target != relayState) {
    relayState = target;
    digitalWrite(RELAY_PIN, relayState ? HIGH : LOW);
    statusDirty = true;
  }
}

static void tryInitSensor(uint32_t now) {
  if (bmeReady) return;
  if (lastSensorReinit != 0 && (uint32_t)(now - lastSensorReinit) < SENSOR_REINIT_INTERVAL) return;
  lastSensorReinit = now ? now : 1;
  bool ok = bme.begin(config.sensorAddress);
  if (!ok) {
    uint8_t alt = (config.sensorAddress == 0x76) ? 0x77 : 0x76;
    ok = bme.begin(alt);
  }
  if (ok) {
    bme.setSampling(Adafruit_BME280::MODE_FORCED,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::SAMPLING_X1,
                    Adafruit_BME280::FILTER_OFF);
    bmeReady = true;
    statusDirty = true;
  }
}

static void updateSensor(uint32_t now) {
  tryInitSensor(now);
  if (!bmeReady) return;
  uint32_t interval = config.sampleInterval ? config.sampleInterval : SENSOR_READ_INTERVAL;
  if ((uint32_t)(now - lastSensorRead) < interval) return;
  lastSensorRead = now ? now : 1;
  bme.takeForcedMeasurement();
  float t = bme.readTemperature();
  float h = bme.readHumidity();
  float p = bme.readPressure() / 100.0f;
  bool ok = false;
  if (!isnan(t) && t > -100 && t < 100) { lastTemp = t; ok = true; }
  if (!isnan(h) && h >= 0 && h <= 100) { lastHum = h; ok = true; }
  if (!isnan(p) && p > 0 && p < 2000) { lastPres = p; ok = true; }
  if (ok) {
    sensorValid = true;
    sensorFailCount = 0;
  } else if (sensorFailCount < 255) {
    sensorFailCount++;
    if (sensorFailCount >= 3) {
      sensorValid = false;
      bmeReady = false;
      lastSensorReinit = 0;
    }
  }
  statusDirty = true;
}

static void checkBootButton(uint32_t now) {
  if ((uint32_t)(now - lastButtonCheck) < BUTTON_CHECK_INTERVAL) return;
  lastButtonCheck = now;
  bool pressed = (digitalRead(BOOT_BUTTON) == LOW);
  if (pressed && !buttonPressed) {
    buttonPressed = true;
    buttonPressStart = now ? now : 1;
  } else if (!pressed && buttonPressed) {
    buttonPressed = false;
    buttonPressStart = 0;
  } else if (pressed && buttonPressed) {
    if ((uint32_t)(now - buttonPressStart) >= BUTTON_HOLD_TIME) {
      factoryResetRequested = true;
      buttonPressed = false;
      buttonPressStart = 0;
    }
  }
}

static void performFactoryReset() {
  Config def;
  setDefaults(def);
  saveConfigNow(def);
  delay(100);
  ESP.restart();
}

static void setupAP() {
  if (apRunning) return;
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.setSleepMode(WIFI_NONE_SLEEP);
  IPAddress apIP(192, 168, 4, 1);
  IPAddress netMsk(255, 255, 255, 0);
  WiFi.softAPConfig(apIP, apIP, netMsk);
  const char* pw = strlen(config.apPassword) >= 8 ? config.apPassword : nullptr;
  WiFi.softAP(config.apSSID, pw, config.apChannel, config.apHidden, config.apMaxConn);
  dnsServer.stop();
  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", apIP);
  apRunning = true;
  delay(50);
}

static String htmlEscape(const String& s) {
  String out;
  out.reserve(s.length() + 16);
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    switch (c) {
      case '<': out += "&lt;"; break;
      case '>': out += "&gt;"; break;
      case '&': out += "&amp;"; break;
      case '"': out += "&quot;"; break;
      case '\'': out += "&#39;"; break;
      default: out += c;
    }
  }
  return out;
}

static const char* PAGE_HEAD =
  "<!DOCTYPE html><html><head><meta charset='utf-8'>"
  "<meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>ESP8266 BME280</title><style>"
  "body{font-family:Arial,sans-serif;margin:0;padding:16px;background:#f4f6f9;color:#222}"
  "nav{background:#223;padding:8px;border-radius:8px;margin-bottom:12px}"
  "nav a{color:#fff;text-decoration:none;margin-right:14px;font-weight:600}"
  ".card{background:#fff;border-radius:8px;padding:14px;margin-bottom:12px;box-shadow:0 1px 3px rgba(0,0,0,.15)}"
  "h1{font-size:20px;margin:0 0 12px}h2{font-size:16px;margin:0 0 8px}"
  "label{display:block;margin-top:8px;font-size:13px}"
  "input,select{width:100%;box-sizing:border-box;padding:7px;margin-top:4px;border:1px solid #bbb;border-radius:6px;font-size:14px}"
  "button{margin-top:12px;padding:10px 16px;border:0;border-radius:6px;background:#2563eb;color:#fff;font-size:14px;cursor:pointer}"
  ".row{display:flex;flex-wrap:wrap;gap:12px}.row>div{flex:1;min-width:120px}"
  ".val{font-size:22px;font-weight:700;color:#0f766e}"
  ".tag{display:inline-block;padding:3px 8px;border-radius:6px;background:#e2e8f0;font-size:12px}"
  ".on{background:#16a34a;color:#fff}.off{background:#dc2626;color:#fff}"
  ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px}"
  "</style></head><body>"
  "<nav><a href='/'>Dashboard</a><a href='/settings'>Sensor</a>"
  "<a href='/wifi'>AP</a></nav>";

static const char* PAGE_FOOT = "</body></html>";

static void sendPage(const String& body) {
  String page;
  page.reserve(strlen(PAGE_HEAD) + body.length() + strlen(PAGE_FOOT) + 8);
  page += PAGE_HEAD;
  page += body;
  page += PAGE_FOOT;
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", page);
}

static void handleRoot() {
  String b;
  b.reserve(1024);
  b += "<div class='card'><h1>BME280 Dashboard</h1><div class='grid'>";
  b += "<div><div class='tag'>Temperature</div><div class='val' id='t'>--</div></div>";
  b += "<div><div class='tag'>Humidity</div><div class='val' id='h'>--</div></div>";
  b += "<div><div class='tag'>Pressure</div><div class='val' id='p'>--</div></div>";
  b += "<div><div class='tag'>Relay</div><div class='val' id='r'>--</div></div>";
  b += "</div></div>";
  b += "<div class='card'><h2>Status</h2>";
  b += "<div><b>Uptime:</b> <span id='up'>--</span></div>";
  b += "<div><b>Heap:</b> <span id='hf'>--</span> B</div>";
  b += "<div><b>Frag:</b> <span id='fg'>--</span>%</div>";
  b += "<div><b>Clients:</b> <span id='wc'>--</span></div>";
  b += "<div><b>Sensor:</b> <span id='sr'>--</span></div>";
  b += "<div><b>Mode:</b> <span id='rm'>--</span></div>";
  b += "</div><script>"
       "function f(v){return Number(v).toFixed(2)}"
       "function upd(){fetch('/api/status',{cache:'no-store'})"
       ".then(r=>r.json()).then(d=>{"
       "var T=document.getElementById('t');T.textContent=f(d.temp)+' \\u00B0C';"
       "document.getElementById('h').textContent=f(d.hum)+' %';"
       "document.getElementById('p').textContent=f(d.pres)+' hPa';"
       "var r=document.getElementById('r');r.textContent=d.relay?'ON':'OFF';"
       "r.className='val '+(d.relay?'on':'off');"
       "document.getElementById('up').textContent=d.uptime+' s';"
       "document.getElementById('hf').textContent=d.heap;"
       "document.getElementById('fg').textContent=d.frag;"
       "document.getElementById('wc').textContent=d.clients;"
       "document.getElementById('sr').textContent=d.sensor?'OK':'FAIL';"
       "document.getElementById('rm').textContent="
       "['Manual','Temp','Hum','Pres','Temp+Hum'][d.mode]||'?';"
       "}).catch(e=>{});}"
       "upd();setInterval(upd,2000);</script>";
  sendPage(b);
}

static void handleSettings() {
  String b;
  b.reserve(1536);
  b += "<div class='card'><h1>Sensor Settings</h1>"
       "<form method='POST' action='/api/settings'>";
  b += "<div class='row'><div><label>Relay Mode</label><select name='mode'>";
  const char* modes[] = {"Manual","Temp >= Th","Hum >= Th","Pres <= Th","Temp OR Hum"};
  for (int i = 0; i < 5; i++) {
    b += "<option value='"; b += i; b += "'";
    if (config.relayMode == (uint32_t)i) b += " selected";
    b += ">"; b += modes[i]; b += "</option>";
  }
  b += "</select></div><div><label>Manual Relay (0/1)</label>"
       "<input name='relay' type='number' min='0' max='1' value='";
  b += config.relayValue;
  b += "'></div></div><div class='row'>";
  b += "<div><label>Temp Th (\u00B0C)</label>"
       "<input name='tt' type='number' step='0.1' min='-40' max='85' value='";
  b += String(config.tempThreshold, 1); b += "'></div>";
  b += "<div><label>Hum Th (%)</label>"
       "<input name='ht' type='number' step='0.1' min='0' max='100' value='";
  b += String(config.humThreshold, 1); b += "'></div>";
  b += "<div><label>Pres Th (hPa)</label>"
       "<input name='pt' type='number' step='0.1' min='300' max='1100' value='";
  b += String(config.presThreshold, 1); b += "'></div></div>";
  b += "<div class='row'>";
  b += "<div><label>Sample (ms)</label>"
       "<input name='si' type='number' min='1000' max='600000' value='";
  b += config.sampleInterval; b += "'></div>";
  b += "<div><label>I2C addr (0x76/0x77)</label><input name='sa' type='text' value='0x";
  b += String(config.sensorAddress, HEX); b += "'></div></div>";
  b += "<button type='submit'>Save</button></form></div>";
  b += "<div class='card'><h2>Factory Reset</h2>"
       "<p>Hold BOOT 5 s, or:</p>"
       "<button onclick=\"if(confirm('Factory reset?'))"
       "fetch('/api/factoryreset',{method:'POST'})\">Factory Reset</button></div>";
  sendPage(b);
}

static void handleWifi() {
  String b;
  b.reserve(1024);
  b += "<div class='card'><h1>AP Settings</h1>"
       "<form method='POST' action='/api/wifi'>";
  b += "<label>SSID</label><input name='ssid' maxlength='32' value='";
  b += htmlEscape(config.apSSID); b += "'>";
  b += "<label>Password (min 8, empty=open)</label>"
       "<input name='pass' maxlength='64' value='";
  b += htmlEscape(config.apPassword); b += "'>";
  b += "<div class='row'>";
  b += "<div><label>Channel 1-13</label>"
       "<input name='ch' type='number' min='1' max='13' value='";
  b += config.apChannel; b += "'></div>";
  b += "<div><label>Hidden</label><select name='hid'>";
  b += "<option value='0'"; if (!config.apHidden) b += " selected"; b += ">No</option>";
  b += "<option value='1'"; if (config.apHidden) b += " selected"; b += ">Yes</option>";
  b += "</select></div>";
  b += "<div><label>Max Clients 1-8</label>"
       "<input name='mc' type='number' min='1' max='8' value='";
  b += config.apMaxConn; b += "'></div></div>";
  b += "<button type='submit'>Save &amp; Reboot</button></form></div>";
  sendPage(b);
}

static void handleStatusApi() {
  uint32_t now = millis();
  if (statusDirty || (uint32_t)(now - lastStatusBuild) > 1000) {
    uint32_t heap = ESP.getFreeHeap();
    int rawFrag = ESP.getHeapFragmentation();
    uint8_t frag = (rawFrag < 0) ? 0 : (rawFrag > 100 ? 100 : (uint8_t)rawFrag);
    cachedStatusJson = "";
    cachedStatusJson.reserve(256);
    cachedStatusJson += F("{\"temp\":"); cachedStatusJson += String(lastTemp, 2);
    cachedStatusJson += F(",\"hum\":"); cachedStatusJson += String(lastHum, 2);
    cachedStatusJson += F(",\"pres\":"); cachedStatusJson += String(lastPres, 2);
    cachedStatusJson += F(",\"relay\":"); cachedStatusJson += relayState ? 1 : 0;
    cachedStatusJson += F(",\"uptime\":"); cachedStatusJson += (uint32_t)(now / 1000UL);
    cachedStatusJson += F(",\"heap\":"); cachedStatusJson += heap;
    cachedStatusJson += F(",\"frag\":"); cachedStatusJson += frag;
    cachedStatusJson += F(",\"clients\":"); cachedStatusJson += WiFi.softAPgetStationNum();
    cachedStatusJson += F(",\"sensor\":"); cachedStatusJson += (bmeReady && sensorValid) ? 1 : 0;
    cachedStatusJson += F(",\"mode\":"); cachedStatusJson += config.relayMode;
    cachedStatusJson += F("}");
    lastStatusBuild = now;
    statusDirty = false;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", cachedStatusJson);
}

static float clampF(float v, float lo, float hi, float def) {
  if (isnan(v) || isinf(v)) return def;
  if (v < lo) return lo;
  if (v > hi) return hi;
  return v;
}

static void handleSaveSettings() {
  Config c = config;
  if (server.hasArg("mode")) {
    uint32_t m = (uint32_t)server.arg("mode").toInt();
    c.relayMode = (m > 4) ? 0 : m;
  }
  if (server.hasArg("relay")) {
    c.relayValue = server.arg("relay").toInt() ? 1 : 0;
  }
  if (server.hasArg("tt")) c.tempThreshold = clampF(server.arg("tt").toFloat(), -40, 85, c.tempThreshold);
  if (server.hasArg("ht")) c.humThreshold = clampF(server.arg("ht").toFloat(), 0, 100, c.humThreshold);
  if (server.hasArg("pt")) c.presThreshold = clampF(server.arg("pt").toFloat(), 300, 1100, c.presThreshold);
  if (server.hasArg("si")) {
    uint32_t v = (uint32_t)server.arg("si").toInt();
    if (v < 1000) v = 1000;
    if (v > 600000) v = 600000;
    c.sampleInterval = v;
  }
  if (server.hasArg("sa")) {
    uint16_t addr = (uint16_t)strtoul(server.arg("sa").c_str(), nullptr, 0);
    if (addr == 0x76 || addr == 0x77) c.sensorAddress = addr;
  }
  if (c.sensorAddress != config.sensorAddress) {
    bmeReady = false;
    sensorValid = false;
    sensorFailCount = 0;
    lastSensorReinit = 0;
  }
  requestConfigSave(c);
  saveConfigNow(config);
  configDirty = false;
  pendingConfig = config;
  applyRelay();
  server.sendHeader("Location", "/settings", true);
  server.send(302, "text/plain", "");
}

static void handleSaveWifi() {
  Config c = config;
  bool restartNeeded = false;
  if (server.hasArg("ssid")) {
    String s = server.arg("ssid");
    if (s.length() >= 1 && s.length() <= 32 && strcmp(s.c_str(), c.apSSID) != 0) {
      strncpy(c.apSSID, s.c_str(), sizeof(c.apSSID) - 1);
      c.apSSID[sizeof(c.apSSID) - 1] = 0;
      restartNeeded = true;
    }
  }
  if (server.hasArg("pass")) {
    String p = server.arg("pass");
    if (p.length() == 0 || (p.length() >= 8 && p.length() <= 64)) {
      if (strcmp(p.c_str(), c.apPassword) != 0) {
        strncpy(c.apPassword, p.c_str(), sizeof(c.apPassword) - 1);
        c.apPassword[sizeof(c.apPassword) - 1] = 0;
        restartNeeded = true;
      }
    }
  }
  if (server.hasArg("ch")) {
    uint8_t ch = (uint8_t)server.arg("ch").toInt();
    if (ch >= 1 && ch <= 13 && ch != c.apChannel) { c.apChannel = ch; restartNeeded = true; }
  }
  if (server.hasArg("hid")) {
    uint8_t h = server.arg("hid").toInt() ? 1 : 0;
    if (h != c.apHidden) { c.apHidden = h; restartNeeded = true; }
  }
  if (server.hasArg("mc")) {
    uint8_t mc = (uint8_t)server.arg("mc").toInt();
    if (mc >= 1 && mc <= 8 && mc != c.apMaxConn) { c.apMaxConn = mc; restartNeeded = true; }
  }
  if (restartNeeded) {
    c.apSSID[32] = 0;
    c.apPassword[64] = 0;
    saveConfigNow(c);
  }
  server.sendHeader("Location", restartNeeded ? "/" : "/wifi", true);
  server.send(302, "text/plain", "");
  if (restartNeeded) {
    server.client().flush();
    delay(300);
    ESP.restart();
  }
}

static void handleFactoryResetApi() {
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "application/json", "{\"ok\":1}");
  server.client().flush();
  delay(100);
  performFactoryReset();
}

static void handleNotFound() {
  if (server.uri().startsWith("/update")) {
    server.send(404, "text/plain", "not found");
    return;
  }
  server.sendHeader("Location", "http://192.168.4.1/", true);
  server.send(302, "text/plain", "");
}

void setup() {
  pinMode(RELAY_PIN, OUTPUT);
  digitalWrite(RELAY_PIN, LOW);
  pinMode(BOOT_BUTTON, INPUT_PULLUP);

  EEPROM.begin(EEPROM_SIZE);
  if (!loadConfig(config)) {
    setDefaults(config);
    saveConfigNow(config);
  }
  configDirty = false;
  pendingConfig = config;

  Wire.begin(SDA_PIN, SCL_PIN);
  Wire.setClock(100000);

  tryInitSensor(millis());

  setupAP();

  httpUpdater.setup(&server, "/update");
  server.on("/", HTTP_GET, handleRoot);
  server.on("/settings", HTTP_GET, handleSettings);
  server.on("/wifi", HTTP_GET, handleWifi);
  server.on("/api/status", HTTP_GET, handleStatusApi);
  server.on("/api/settings", HTTP_POST, handleSaveSettings);
  server.on("/api/wifi", HTTP_POST, handleSaveWifi);
  server.on("/api/factoryreset", HTTP_POST, handleFactoryResetApi);
  server.onNotFound(handleNotFound);
  server.begin();

  lastConfigWrite = millis();
  applyRelay();
}

void loop() {
  uint32_t now = millis();

  dnsServer.processNextRequest();
  server.handleClient();

  checkBootButton(now);
  if (factoryResetRequested) {
    factoryResetRequested = false;
    performFactoryReset();
  }

  updateSensor(now);
  applyRelay();
  tickConfigSave(now);

  if ((uint32_t)(now - lastWifiCheck) >= WIFI_CHECK_INTERVAL) {
    lastWifiCheck = now;
    if (WiFi.getMode() != WIFI_AP) {
      apRunning = false;
      setupAP();
    }
  }

  yield();
}
