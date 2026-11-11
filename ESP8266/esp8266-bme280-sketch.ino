#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_BME280.h>
#include <EEPROM.h>

// ===== Pin definitions =====
#define PIN_SDA       4    
#define PIN_SCL       5   
#define PIN_RELAY     13
#define PIN_BOOT      0

// ===== AP credentials =====
static const char AP_SSID[] PROGMEM = "ESP8266_BME280";
static const char AP_PASS[] PROGMEM = "12345678";

// ===== NVS layout =====
#define EEPROM_SIZE          32
#define ADDR_MAGIC           0
#define ADDR_TEMP_TH         2
#define ADDR_HUM_TH          4
#define ADDR_RELAY_EN        6
#define ADDR_RELAY_STATE     7
#define ADDR_CHECKSUM        8
#define MAGIC_VALUE          0xB5

// ===== Threshold limits =====
static const int16_t  TEMP_TH_MIN = -400;
static const int16_t  TEMP_TH_MAX = 850;
static const uint16_t HUM_TH_MIN  = 0;
static const uint16_t HUM_TH_MAX  = 1000;

static const int16_t  TEMP_HYST   = 3;
static const uint16_t HUM_HYST    = 10;

ESP8266WebServer server(80);
DNSServer        dnsServer;
Adafruit_BME280  bme;

struct Settings {
  int16_t  tempTh;
  uint16_t humTh;
  uint8_t  relayEn;
  uint8_t  relayState;
} cfg;

bool     bmeOk        = false;
bool     buttonHeld   = false;
bool     buttonUsed   = false;
uint32_t tSample      = 0;
uint32_t tRelay       = 0;
uint32_t tButton      = 0;
uint32_t tDns         = 0;
uint32_t tBmeRetry    = 0;
uint32_t tApCheck     = 0;

float sTemp = NAN, sHum = NAN, sPress = NAN;

// ===== helpers =====
static int16_t clampI16(int32_t v, int16_t lo, int16_t hi) {
  if (v < lo) return lo;
  if (v > hi) return hi;
  return (int16_t)v;
}
static uint16_t clampU16(int32_t v, uint16_t lo, uint16_t hi) {
  if (v < (int32_t)lo) return lo;
  if (v > (int32_t)hi) return hi;
  return (uint16_t)v;
}

static bool isNumeric(const String &s) {
  if (s.length() == 0 || s.length() > 10) return false;
  bool dot = false;
  bool hasDigit = false;
  for (size_t i = 0; i < s.length(); i++) {
    char c = s[i];
    if (c == '-' && i == 0) continue;
    if (c == '.' && !dot) { dot = true; continue; }
    if (c < '0' || c > '9') return false;
    hasDigit = true;
  }
  return hasDigit;
}

static uint8_t calcChecksum() {
  uint8_t cs = MAGIC_VALUE;
  cs ^= (uint8_t)(cfg.tempTh & 0xFF);
  cs ^= (uint8_t)((cfg.tempTh >> 8) & 0xFF);
  cs ^= (uint8_t)(cfg.humTh & 0xFF);
  cs ^= (uint8_t)((cfg.humTh >> 8) & 0xFF);
  cs ^= cfg.relayEn;
  cs ^= cfg.relayState;
  return cs;
}

static bool saveSettings();   

static bool loadSettings() {
  if (EEPROM.read(ADDR_MAGIC) != MAGIC_VALUE) return false;

  int16_t  tt = (int16_t)(EEPROM.read(ADDR_TEMP_TH) | (EEPROM.read(ADDR_TEMP_TH + 1) << 8));
  uint16_t ht = (uint16_t)(EEPROM.read(ADDR_HUM_TH)  | (EEPROM.read(ADDR_HUM_TH + 1)  << 8));
  uint8_t  re = EEPROM.read(ADDR_RELAY_EN);
  uint8_t  rs = EEPROM.read(ADDR_RELAY_STATE);

  cfg.tempTh     = tt;
  cfg.humTh      = ht;
  cfg.relayEn    = re;
  cfg.relayState = rs;
  bool checksumOk = (EEPROM.read(ADDR_CHECKSUM) == calcChecksum());
  if (!checksumOk) return false;

  int16_t  clampedT = clampI16(tt, TEMP_TH_MIN, TEMP_TH_MAX);
  uint16_t clampedH = clampU16(ht, HUM_TH_MIN,  HUM_TH_MAX);
  uint8_t  fixedE   = (re == 0 || re == 1) ? re : 0;
  uint8_t  fixedS   = (rs == 0 || rs == 1) ? rs : 0;

  bool changed = (clampedT != tt) || (clampedH != ht) || (fixedE != re) || (fixedS != rs);

  cfg.tempTh     = clampedT;
  cfg.humTh      = clampedH;
  cfg.relayEn    = fixedE;
  cfg.relayState = fixedS;

  if (changed) saveSettings();

  return true;
}

static bool saveSettings() {
  EEPROM.write(ADDR_MAGIC, MAGIC_VALUE);
  EEPROM.write(ADDR_TEMP_TH,     (uint8_t)(cfg.tempTh & 0xFF));
  EEPROM.write(ADDR_TEMP_TH + 1, (uint8_t)((cfg.tempTh >> 8) & 0xFF));
  EEPROM.write(ADDR_HUM_TH,      (uint8_t)(cfg.humTh & 0xFF));
  EEPROM.write(ADDR_HUM_TH + 1,  (uint8_t)((cfg.humTh >> 8) & 0xFF));
  EEPROM.write(ADDR_RELAY_EN,    cfg.relayEn);
  EEPROM.write(ADDR_RELAY_STATE, cfg.relayState);
  EEPROM.write(ADDR_CHECKSUM,    calcChecksum());
  for (int i = 0; i < 3; i++) {
    if (EEPROM.commit()) return true;
  }
  return false;
}

static void applyRelayState() {
  digitalWrite(PIN_RELAY, cfg.relayState ? HIGH : LOW);
}

static void defaults() {
  cfg.tempTh     = 300;
  cfg.humTh      = 700;
  cfg.relayEn    = 1;
  cfg.relayState = 0;
}

static void factoryReset() {
  defaults();
  saveSettings();
  applyRelayState();
}

// ===== HTML =====
static const char HTML_HEAD[] PROGMEM =
  "<!DOCTYPE html><html><head><meta charset='utf-8'><meta name='viewport' content='width=device-width,initial-scale=1'>"
  "<title>ESP8266 BME280</title><style>"
  "body{font-family:Arial;margin:0;background:#f2f4f7;color:#222}"
  "header{background:#204060;color:#fff;padding:12px 16px;font-size:18px}"
  ".wrap{max-width:900px;margin:0 auto;padding:12px}"
  ".card{background:#fff;border-radius:8px;padding:16px;margin:12px 0;box-shadow:0 1px 3px rgba(0,0,0,.1)}"
  ".grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(140px,1fr));gap:12px}"
  ".val{font-size:24px;font-weight:bold;color:#204060}"
  ".lbl{font-size:12px;color:#666;text-transform:uppercase}"
  "input[type=number]{width:100%;padding:8px;box-sizing:border-box;border:1px solid #ccc;border-radius:4px}"
  "button{padding:10px 16px;border:0;border-radius:4px;background:#204060;color:#fff;font-size:14px;cursor:pointer;margin-top:8px}"
  "button.alt{background:#666}button.danger{background:#b33}"
  "table{width:100%;border-collapse:collapse}td{padding:6px 0;border-bottom:1px solid #eee}"
  "a{color:#204060;text-decoration:none;margin-right:12px}"
  "</style></head><body>";

static const char HTML_TAIL[] PROGMEM = "</div></body></html>";

static void sendPage(const String &title, const String &body) {
  String out;
  out.reserve(3800);
  out += FPSTR(HTML_HEAD);
  out += "<header>ESP8266 BME280 &middot; ";
  out += title;
  out += "</header><div class='wrap'>";
  out += "<div class='card'><a href='/'>Dashboard</a><a href='/config'>Configuration</a><a href='/wifi'>Wi-Fi</a></div>";
  out += body;
  out += FPSTR(HTML_TAIL);
  server.sendHeader("Cache-Control", "no-store");
  server.send(200, "text/html; charset=utf-8", out);
}

static String fmt(float v, int digits) {
  if (isnan(v) || isinf(v)) return String("--");
  return String(v, digits);
}

static String buildDashboard() {
  String b;
  b.reserve(2000);
  b += "<div class='card'><h3>Live Readings</h3><div class='grid'>";
  b += "<div><div class='lbl'>Temperature</div><div class='val'>"; b += fmt(sTemp, 2); b += " &deg;C</div></div>";
  b += "<div><div class='lbl'>Humidity</div><div class='val'>";    b += fmt(sHum, 2);  b += " %</div></div>";
  b += "<div><div class='lbl'>Pressure</div><div class='val'>";    b += fmt(sPress, 2);b += " hPa</div></div>";
  b += "<div><div class='lbl'>Relay</div><div class='val'>";       b += (cfg.relayState ? "ON" : "OFF"); b += "</div></div>";
  b += "</div></div>";

  b += "<div class='card'><h3>Relay Control</h3>";
  b += "<div>Auto threshold mode: <b>"; b += (cfg.relayEn ? "ON" : "OFF"); b += "</b></div>";
  b += "<div style='margin-top:6px'>";
  b += (cfg.relayEn ? "Auto state" : "Manual state");
  b += ": <b>"; b += (cfg.relayState ? "ON" : "OFF"); b += "</b></div>";
  b += "<form method='POST' action='/relay'>"
       "<button name='state' value='on' type='submit'>Relay ON</button> "
       "<button name='state' value='off' type='submit' class='alt'>Relay OFF</button> "
       "<button name='state' value='toggle' type='submit' class='alt'>Toggle</button>"
       "</form>";
  b += "<form method='POST' action='/auto'>"
       "<button name='mode' value='on' type='submit'>Auto ON</button> "
       "<button name='mode' value='off' type='submit' class='alt'>Auto OFF</button>"
       "</form></div>";

  b += "<div class='card'><h3>System</h3><table>";
  b += "<tr><td>Uptime (s)</td><td>"; b += String(millis() / 1000UL); b += "</td></tr>";
  b += "<tr><td>Free heap</td><td>";  b += String(ESP.getFreeHeap()); b += "</td></tr>";
  b += "<tr><td>Sensor</td><td>";     b += (bmeOk ? "OK" : "FAULT"); b += "</td></tr>";
  b += "<tr><td>IP</td><td>";         b += WiFi.softAPIP().toString(); b += "</td></tr>";
  b += "<tr><td>Clients</td><td>";    b += String(WiFi.softAPgetStationNum()); b += "</td></tr>";
  b += "</table></div>";
  return b;
}

static String buildConfig() {
  String b;
  b.reserve(2100);
  if (server.hasArg("err")) {
    b += "<div class='card' style='background:#fdd;color:#900'>"
         "Invalid input &mdash; value out of range or not numeric. "
         "Settings were not saved.</div>";
  }
  b += "<div class='card'><h3>Thresholds</h3>";
  b += "<form method='POST' action='/config'>";
  b += "<input type='hidden' name='cfgform' value='1'>";
  b += "<div style='margin-bottom:10px'><div class='lbl'>Temperature threshold (&deg;C, -40.0 to 85.0)</div>";
  b += "<input type='number' step='0.1' min='-40' max='85' name='temp' value='"; b += String(cfg.tempTh / 10.0f, 1); b += "'></div>";
  b += "<div style='margin-bottom:10px'><div class='lbl'>Humidity threshold (%, 0.0 to 100.0)</div>";
  b += "<input type='number' step='0.1' min='0' max='100' name='hum' value='"; b += String(cfg.humTh / 10.0f, 1); b += "'></div>";
  b += "<div style='margin-bottom:10px'><div class='lbl'>Auto relay threshold mode</div>";
  b += "<input type='checkbox' name='auto' value='1'"; b += (cfg.relayEn ? " checked" : ""); b += "></div>";
  b += "<button type='submit'>Save Settings</button></form></div>";

  b += "<div class='card'><h3>Factory Reset</h3>";
  b += "<p>Restores all settings to defaults. Wi-Fi credentials are not affected.</p>";
  b += "<form method='POST' action='/reset' onsubmit=\"return confirm('Factory reset?')\">";
  b += "<button type='submit' class='danger'>Factory Reset</button></form></div>";
  return b;
}

static String buildWifi() {
  String b;
  b.reserve(900);
  b += "<div class='card'><h3>Access Point</h3><table>";
  b += "<tr><td>SSID</td><td>";     b += FPSTR(AP_SSID); b += "</td></tr>";
  b += "<tr><td>Password</td><td>"; b += FPSTR(AP_PASS); b += "</td></tr>";
  b += "<tr><td>Status</td><td>";   b += ((WiFi.getMode() & WIFI_AP) ? "Active" : "Inactive"); b += "</td></tr>";
  b += "<tr><td>IP</td><td>";       b += WiFi.softAPIP().toString(); b += "</td></tr>";
  b += "<tr><td>Clients</td><td>";  b += String(WiFi.softAPgetStationNum()); b += "</td></tr>";
  b += "</table></div>";
  b += "<div class='card'><h3>Captive Portal</h3><p>All HTTP requests are redirected to the dashboard. "
       "Any domain resolves here and every unknown path is captured.</p></div>";
  return b;
}

// ===== handlers =====
static void handleRoot()     { sendPage("Dashboard",     buildDashboard()); }
static void handleConfigGet(){ sendPage("Configuration", buildConfig());   }
static void handleWifiGet()  { sendPage("Wi-Fi",         buildWifi());      }

static void handleConfigPost() {
  bool bad = false;
  int16_t  newTemp = cfg.tempTh;
  uint16_t newHum  = cfg.humTh;
  uint8_t  newEn   = cfg.relayEn;

  if (server.hasArg("temp")) {
    String v = server.arg("temp");
    if (isNumeric(v)) {
      float f = v.toFloat();
      if (f < (TEMP_TH_MIN / 10.0f) || f > (TEMP_TH_MAX / 10.0f)) {
        bad = true;
      } else {
        int32_t t = (int32_t)(f * 10.0f);
        newTemp = clampI16(t, TEMP_TH_MIN, TEMP_TH_MAX);
      }
    } else {
      bad = true;
    }
  }

  if (server.hasArg("hum")) {
    String v = server.arg("hum");
    if (isNumeric(v)) {
      float f = v.toFloat();
      if (f < (HUM_TH_MIN / 10.0f) || f > (HUM_TH_MAX / 10.0f)) {
        bad = true;
      } else {
        int32_t h = (int32_t)(f * 10.0f);
        newHum = clampU16(h, HUM_TH_MIN, HUM_TH_MAX);
      }
    } else {
      bad = true;
    }
  }

  if (server.hasArg("cfgform")) {
    newEn = server.hasArg("auto") ? 1 : 0;
  }

  if (!bad) {
    cfg.tempTh  = newTemp;
    cfg.humTh   = newHum;
    cfg.relayEn = newEn;
    saveSettings();
  }

  server.sendHeader("Location", bad ? "/config?err=1" : "/config", true);
  server.send(302, "text/plain", "");
}

static void handleRelayPost() {
  if (server.hasArg("state")) {
    String s = server.arg("state");
    bool valid = true;
    if      (s == "on")     cfg.relayState = 1;
    else if (s == "off")    cfg.relayState = 0;
    else if (s == "toggle") cfg.relayState = cfg.relayState ? 0 : 1;
    else valid = false;
    if (valid) {
      cfg.relayEn = 0;
      saveSettings();
      applyRelayState();
    }
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

static void handleAutoPost() {
  if (server.hasArg("mode")) {
    cfg.relayEn = (server.arg("mode") == "on") ? 1 : 0;
    saveSettings();
  }
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

static void handleResetPost() {
  factoryReset();
  server.sendHeader("Location", "/config", true);
  server.send(302, "text/plain", "");
}

static void handleNotFound() {
  server.sendHeader("Location", "/", true);
  server.send(302, "text/plain", "");
}

// ===== BME280 helpers =====
static bool initBme() {
  static const uint8_t addrs[2] = {0x76, 0x77};
  for (int a = 0; a < 2; a++) {
    if (bme.begin(addrs[a], &Wire)) {
      bme.setSampling(Adafruit_BME280::MODE_FORCED,
                      Adafruit_BME280::SAMPLING_X2,
                      Adafruit_BME280::SAMPLING_X16,
                      Adafruit_BME280::SAMPLING_X1,
                      Adafruit_BME280::FILTER_X16,
                      Adafruit_BME280::STANDBY_MS_1000);
      bme.takeForcedMeasurement();
      float t = bme.readTemperature();
      float h = bme.readHumidity();
      float p = bme.readPressure();
      if (!isnan(t) && !isnan(h) && !isnan(p) && !isinf(t) && !isinf(h) && !isinf(p)) {
        return true;
      }
      continue;
    }
  }
  return false;
}

// ===== setup =====
void setup() {
  pinMode(PIN_RELAY, OUTPUT);
  digitalWrite(PIN_RELAY, LOW);
  pinMode(PIN_BOOT, INPUT_PULLUP);

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  bmeOk = initBme();

  EEPROM.begin(EEPROM_SIZE);
  if (!loadSettings()) {
    defaults();
    saveSettings();
  }

  if (!cfg.relayEn) {
    applyRelayState();
  } else {
    digitalWrite(PIN_RELAY, LOW);
  }

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
  WiFi.softAP(FPSTR(AP_SSID), FPSTR(AP_PASS));

  dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  dnsServer.start(53, "*", WiFi.softAPIP());

  server.on("/",        HTTP_GET,  handleRoot);
  server.on("/config",  HTTP_GET,  handleConfigGet);
  server.on("/config",  HTTP_POST, handleConfigPost);
  server.on("/wifi",    HTTP_GET,  handleWifiGet);
  server.on("/relay",   HTTP_POST, handleRelayPost);
  server.on("/auto",    HTTP_POST, handleAutoPost);
  server.on("/reset",   HTTP_POST, handleResetPost);

  server.on("/generate_204",        HTTP_GET, handleNotFound);
  server.on("/gen_204",             HTTP_GET, handleNotFound);
  server.on("/hotspot-detect.html", HTTP_GET, handleNotFound);
  server.on("/connecttest.txt",     HTTP_GET, handleNotFound);
  server.on("/ncsi.txt",            HTTP_GET, handleNotFound);
  server.on("/fwlink",              HTTP_GET, handleNotFound);
  server.on("/redirect",            HTTP_GET, handleNotFound);

  server.onNotFound(handleNotFound);
  server.begin();

  uint32_t now = millis();
  tSample   = now;
  tRelay    = now;
  tButton   = now;
  tDns      = now;
  tBmeRetry = now;
  tApCheck  = now;
}

// ===== loop =====
void loop() {
  server.handleClient();
  uint32_t now = millis();

  if ((uint32_t)(now - tDns) >= 5) {
    tDns = now;
    dnsServer.processNextRequest();
  }

  if ((uint32_t)(now - tSample) >= 1000) {
    tSample = now;
    if (bmeOk) {
      bme.takeForcedMeasurement();
      float t = bme.readTemperature();
      float h = bme.readHumidity();
      float p = bme.readPressure() / 100.0f;
      if (isnan(t) || isnan(h) || isnan(p) || isinf(t) || isinf(h) || isinf(p)) {
        bmeOk = false;
        tBmeRetry = now;
      } else {
        sTemp = t; sHum = h; sPress = p;
      }
    }
  }

  if (!bmeOk && (uint32_t)(now - tBmeRetry) >= 5000) {
    tBmeRetry = now;
    Wire.begin(PIN_SDA, PIN_SCL);
    Wire.setClock(100000);
    bmeOk = initBme();
  }

  // Relay tick
  if ((uint32_t)(now - tRelay) >= 250) {
    tRelay = now;
    if (cfg.relayEn && !isnan(sTemp) && !isnan(sHum)) {
      int32_t t10 = (int32_t)(sTemp * 10.0f);
      int32_t h10 = (int32_t)(sHum  * 10.0f);
      bool want;
      if (cfg.relayState == 0) {
        want = (t10 >= (int32_t)cfg.tempTh) || (h10 >= (int32_t)cfg.humTh);
      } else {
        int32_t tOff = (int32_t)cfg.tempTh - (int32_t)TEMP_HYST;
        int32_t hOff = (int32_t)cfg.humTh  - (int32_t)HUM_HYST;
        if (tOff < (int32_t)TEMP_TH_MIN) tOff = (int32_t)TEMP_TH_MIN;
        if (hOff < (int32_t)HUM_TH_MIN)  hOff = (int32_t)HUM_TH_MIN;
        want = (t10 > tOff) || (h10 > hOff);
      }
      uint8_t target = want ? 1 : 0;
      if (target != cfg.relayState) {
        cfg.relayState = target;
      }
    }
    applyRelayState();
  }

  if ((uint32_t)(now - tApCheck) >= 30000) {
    tApCheck = now;
    IPAddress ip = WiFi.softAPIP();
    if (!(WiFi.getMode() & WIFI_AP) || ip == IPAddress(0, 0, 0, 0)) {
      WiFi.mode(WIFI_AP);
      WiFi.softAPConfig(IPAddress(192, 168, 4, 1), IPAddress(192, 168, 4, 1), IPAddress(255, 255, 255, 0));
      WiFi.softAP(FPSTR(AP_SSID), FPSTR(AP_PASS));
      dnsServer.stop();
      dnsServer.start(53, "*", WiFi.softAPIP());
    }
  }

  // Boot button factory reset (5 s hold, latched)
  int bootState = digitalRead(PIN_BOOT);
  if (bootState == LOW) {
    if (!buttonUsed) {
      if (!buttonHeld) {
        buttonHeld = true;
        tButton = now;
      } else if ((uint32_t)(now - tButton) >= 5000) {
        buttonHeld = false;
        buttonUsed = true;
        factoryReset();
      }
    }
  } else {
    buttonHeld = false;
    buttonUsed = false;
  }
}
