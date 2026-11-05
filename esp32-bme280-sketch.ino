/*
 * ESP32 BME280 Temperature Humidity Pressure Sensor
 */

#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <Preferences.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <math.h>

#define PIN_I2C_SDA   21
#define PIN_I2C_SCL   22
#define PIN_BOOT_BTN   0
#define PIN_LED        2
#define PIN_RELAY     25

#define CFG_MAGIC     0x45535034UL
#define CFG_VERSION   3
#define CFG_NS        "espcfg"
#define CFG_KEY_A     "cfgA"
#define CFG_KEY_B     "cfgB"
#define CFG_KEY_SEQ   "seq"
#define CFG_KEY_AUTH  "authPass"

#define BME_INTERVAL_MS     2000
#define BTN_DEBOUNCE_MS       50
#define FACTORY_HOLD_MS     5000
#define CFG_WRITE_MIN_MS   60000UL
#define STA_BACKOFF_MAX    30000UL
#define STA_BACKOFF_STEP    5000UL
#define JSON_BUF_SZ          896
#define REBOOT_DELAY_MS      500
#define FACTORY_DELAY_MS     800

#define RELAY_HYST_T  0.5f
#define RELAY_HYST_H  1.0f
#define RELAY_HYST_P  0.5f

struct __attribute__((packed)) Config {
  uint32_t magic;
  uint16_t version;
  uint16_t crc;

  char     apSsid[33];
  char     apPass[65];
  uint8_t  apChannel;
  uint8_t  apHidden;
  uint8_t  apMaxConn;
  int8_t   apTxPower;
  uint8_t  apAuth;

  char     staSsid[33];
  char     staPass[65];
  uint8_t  staDhcp;
  uint32_t staIp, staGw, staMask, staDns1, staDns2;
  char     staHostname[33];
  uint8_t  staUseMac;
  uint8_t  staMac[6];

  float    tMin, tMax, hMin, hMax, pMin, pMax;
  uint8_t  alertsEnabled;

  uint8_t  relayEnabled;
  uint8_t  relayInvert;
  uint8_t  relayUseT;
  uint8_t  relayUseH;
  uint8_t  relayUseP;
  uint8_t  relayLogic;
};

static Config     gCfg;
static bool       gCfgDirty          = false;
static uint32_t   gCfgLastWriteMs    = 0;
static uint8_t    gActiveSlot        = 0;
static Preferences gPrefs;

static uint32_t   gRebootAtMs        = 0;
static uint32_t   gFactoryAtMs       = 0;

static bool       gBtnArmed          = false;
static bool       gBtnPrev           = false;
static uint32_t   gBtnPressMs        = 0;

static Adafruit_BME280 gBme;
static bool       gBmeOk             = false;
static uint8_t    gBmeAddr           = 0;
static bool       gBmeWasOk          = false;
static uint32_t   gBmeLastRead       = 0;

static float      gT = NAN, gH = NAN, gP = NAN;

static bool       gRelayState        = false;

static bool       gStaWantConnect    = false;
static uint32_t   gStaLastAttempt    = 0;
static uint32_t   gStaAttemptGap     = STA_BACKOFF_STEP;
static wl_status_t gStaLastStatus    = WL_IDLE_STATUS;

static WebServer  gServer(80);
static DNSServer  gDns;
static bool       gDnsStarted        = false;

static char       gJsonBuf[JSON_BUF_SZ];
static char       gAuthPass[33] = "12345678";

static uint16_t crc16(const uint8_t* data, size_t len) {
  uint16_t crc = 0xFFFF;
  for (size_t i = 0; i < len; i++) {
    crc ^= (uint16_t)data[i] << 8;
    for (uint8_t b = 0; b < 8; b++)
      crc = (crc & 0x8000) ? (crc << 1) ^ 0x1021 : (crc << 1);
  }
  return crc;
}

static uint16_t cfgCrc(const Config& c) {
  const uint8_t* p = (const uint8_t*)&c;
  size_t off = offsetof(Config, crc) + sizeof(c.crc);
  return crc16(p + off, sizeof(Config) - off);
}

static void setDefaults(Config& c) {
  memset(&c, 0, sizeof(c));
  c.magic = CFG_MAGIC;
  c.version = CFG_VERSION;

  strncpy(c.apSsid, "ESP32-BME280", sizeof(c.apSsid) - 1);
  strncpy(c.apPass, "12345678", sizeof(c.apPass) - 1);
  c.apChannel = 6;
  c.apHidden  = 0;
  c.apMaxConn = 4;
  c.apTxPower = 78;
  c.apAuth    = 1;

  c.staSsid[0] = 0;
  c.staPass[0] = 0;
  c.staDhcp = 1;
  c.staIp = c.staGw = c.staMask = c.staDns1 = c.staDns2 = 0;
  strncpy(c.staHostname, "esp32", sizeof(c.staHostname) - 1);
  c.staUseMac = 0;
  memset(c.staMac, 0, 6);

  c.tMin = -40; c.tMax = 85;
  c.hMin = 0;   c.hMax = 100;
  c.pMin = 300; c.pMax = 1100;
  c.alertsEnabled = 1;

  c.relayEnabled = 0;
  c.relayInvert  = 0;
  c.relayUseT    = 1;
  c.relayUseH    = 1;
  c.relayUseP    = 0;
  c.relayLogic   = 0;

  c.crc = cfgCrc(c);
}

static bool cfgValid(const Config& c) {
  if (c.magic != CFG_MAGIC) return false;
  if (c.version != CFG_VERSION) return false;
  if (c.crc != cfgCrc(c)) return false;
  if (c.apChannel < 1 || c.apChannel > 13) return false;
  if (c.apMaxConn < 1 || c.apMaxConn > 8) return false;
  if (c.apTxPower < 0 || c.apTxPower > 84) return false;
  if (c.apAuth > 1) return false;
  if (c.apSsid[0] == 0) return false;
  for (size_t i = 0; i < sizeof(c.apSsid) && c.apSsid[i]; i++) {
    uint8_t ch = (uint8_t)c.apSsid[i];
    if (ch == '"' || ch == '\\' || ch < 0x20 || ch == 0x7F) return false;
  }
  for (size_t i = 0; i < sizeof(c.staSsid) && c.staSsid[i]; i++) {
    uint8_t ch = (uint8_t)c.staSsid[i];
    if (ch == '"' || ch == '\\' || ch < 0x20 || ch == 0x7F) return false;
  }
  for (size_t i = 0; i < sizeof(c.staHostname) && c.staHostname[i]; i++) {
    uint8_t ch = (uint8_t)c.staHostname[i];
    if (ch == '"' || ch == '\\' || ch < 0x20 || ch == 0x7F) return false;
  }
  if (c.apAuth == 1 && (strlen(c.apPass) < 8 || strlen(c.apPass) > 63)) return false;
  if (c.staSsid[0] != 0 && strlen(c.staPass) > 63) return false;
  if (c.tMin >= c.tMax) return false;
  if (c.hMin >= c.hMax) return false;
  if (c.pMin >= c.pMax) return false;
  if (c.relayEnabled > 1) return false;
  if (c.relayInvert  > 1) return false;
  if (c.relayUseT    > 1) return false;
  if (c.relayUseH    > 1) return false;
  if (c.relayUseP    > 1) return false;
  if (c.relayLogic   > 1) return false;
  return true;
}

static bool loadConfig() {
  if (!gPrefs.begin(CFG_NS, false)) {
    setDefaults(gCfg);
    gCfgDirty = true;
    return false;
  }
  Config a, b;
  size_t la = gPrefs.getBytes(CFG_KEY_A, &a, sizeof(a));
  size_t lb = gPrefs.getBytes(CFG_KEY_B, &b, sizeof(b));
  uint32_t seq = gPrefs.getUInt(CFG_KEY_SEQ, 0);

  bool va = (la == sizeof(a)) && cfgValid(a);
  bool vb = (lb == sizeof(b)) && cfgValid(b);

  if (va && (!vb || (seq & 1) == 0)) { gCfg = a; gActiveSlot = 0; return true; }
  if (vb)                             { gCfg = b; gActiveSlot = 1; return true; }

  setDefaults(gCfg);
  gCfgDirty = true;
  return false;
}

static bool saveConfigForce() {
  gCfg.magic = CFG_MAGIC;
  gCfg.version = CFG_VERSION;
  gCfg.crc = cfgCrc(gCfg);

  const char* key = (gActiveSlot == 0) ? CFG_KEY_B : CFG_KEY_A;
  uint8_t nextSlot = (gActiveSlot == 0) ? 1 : 0;

  size_t written = gPrefs.putBytes(key, &gCfg, sizeof(gCfg));
  if (written != sizeof(gCfg)) return false;

  uint32_t seq = gPrefs.getUInt(CFG_KEY_SEQ, 0) + 1;
  gPrefs.putUInt(CFG_KEY_SEQ, seq);

  gActiveSlot = nextSlot;
  gCfgDirty = false;
  gCfgLastWriteMs = millis();
  return true;
}

static void cfgTick(uint32_t now) {
  if (!gCfgDirty) return;
  if ((uint32_t)(now - gCfgLastWriteMs) < CFG_WRITE_MIN_MS) return;
  saveConfigForce();
}

static void cfgMarkDirty() { gCfgDirty = true; }

static void bmeInit() {
  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  if (gBme.begin(0x76, &Wire))      { gBmeOk = true; gBmeAddr = 0x76; }
  else if (gBme.begin(0x77, &Wire)) { gBmeOk = true; gBmeAddr = 0x77; }
  else                              { gBmeOk = false; gBmeAddr = 0; }

  if (gBmeOk) {
    gBme.setSampling(Adafruit_BME280::MODE_NORMAL,
                     Adafruit_BME280::SAMPLING_X2,
                     Adafruit_BME280::SAMPLING_X16,
                     Adafruit_BME280::SAMPLING_X1,
                     Adafruit_BME280::FILTER_X16,
                     Adafruit_BME280::STANDBY_MS_500);
    gBmeWasOk = true;
  }
}

static void bmeTick(uint32_t now) {
  if (!gBmeOk) {
    if ((uint32_t)(now - gBmeLastRead) >= 10000) {
      gBmeLastRead = now;
      bool ok = gBme.begin(0x76, &Wire);
      if (ok) {
        gBmeAddr = 0x76;
      } else {
        ok = gBme.begin(0x77, &Wire);
        if (ok) gBmeAddr = 0x77;
      }
      if (ok) {
        gBmeOk = true;
        gBmeWasOk = true;
        gBme.setSampling(Adafruit_BME280::MODE_NORMAL,
                         Adafruit_BME280::SAMPLING_X2,
                         Adafruit_BME280::SAMPLING_X16,
                         Adafruit_BME280::SAMPLING_X1,
                         Adafruit_BME280::FILTER_X16,
                         Adafruit_BME280::STANDBY_MS_500);
      }
    }
    return;
  }

  if ((uint32_t)(now - gBmeLastRead) < BME_INTERVAL_MS) return;
  gBmeLastRead = now;

  float t = gBme.readTemperature();
  float h = gBme.readHumidity();
  float p = gBme.readPressure() / 100.0f;

  bool gotSomething = false;
  if (!isnan(t) && t > -100 && t < 100) { gT = t; gotSomething = true; }
  else { gT = NAN; }
  if (!isnan(h) && h >= 0 && h <= 100)  { gH = h; gotSomething = true; }
  else { gH = NAN; }
  if (!isnan(p) && p > 100 && p < 2000) { gP = p; gotSomething = true; }
  else { gP = NAN; }

  if (!gotSomething && gBmeWasOk) {
    gBmeOk = false;
    gBmeWasOk = false;
    gT = NAN;
    gH = NAN;
    gP = NAN;
  }
}

static bool relayShouldBeOn() {
  if (!gCfg.relayEnabled) return false;

  const bool useT = gCfg.relayUseT;
  const bool useH = gCfg.relayUseH;
  const bool useP = gCfg.relayUseP;
  if (!useT && !useH && !useP) return false;

  int vT = 0, vH = 0, vP = 0;

  if (useT && !isnan(gT)) {
    bool rawOut = (gT < gCfg.tMin || gT > gCfg.tMax);
    bool inBand = (gT >= gCfg.tMin + RELAY_HYST_T &&
                   gT <= gCfg.tMax - RELAY_HYST_T);
    if (rawOut)      vT =  1;
    else if (inBand) vT = -1;
  }
  if (useH && !isnan(gH)) {
    bool rawOut = (gH < gCfg.hMin || gH > gCfg.hMax);
    bool inBand = (gH >= gCfg.hMin + RELAY_HYST_H &&
                   gH <= gCfg.hMax - RELAY_HYST_H);
    if (rawOut)      vH =  1;
    else if (inBand) vH = -1;
  }
  if (useP && !isnan(gP)) {
    bool rawOut = (gP < gCfg.pMin || gP > gCfg.pMax);
    bool inBand = (gP >= gCfg.pMin + RELAY_HYST_P &&
                   gP <= gCfg.pMax - RELAY_HYST_P);
    if (rawOut)      vP =  1;
    else if (inBand) vP = -1;
  }

  if (gCfg.relayLogic == 0) {
    if (vT == 1 || vH == 1 || vP == 1) return true;

    int evaluated = 0;
    bool allInside = true;
    if (useT && !isnan(gT)) { evaluated++; if (vT != -1) allInside = false; }
    if (useH && !isnan(gH)) { evaluated++; if (vH != -1) allInside = false; }
    if (useP && !isnan(gP)) { evaluated++; if (vP != -1) allInside = false; }
    if (evaluated > 0 && allInside) return false;

    return gRelayState;
  }

  bool haveAny = false, allOut = true;
  if (useT) { if (isnan(gT)) return gRelayState; haveAny = true; if (vT != 1) allOut = false; }
  if (useH) { if (isnan(gH)) return gRelayState; haveAny = true; if (vH != 1) allOut = false; }
  if (useP) { if (isnan(gP)) return gRelayState; haveAny = true; if (vP != 1) allOut = false; }
  if (!haveAny) return false;
  return allOut;
}

static void relayApplyPin() {
  bool pinHigh = gCfg.relayInvert ? !gRelayState : gRelayState;
  digitalWrite(PIN_RELAY, pinHigh ? HIGH : LOW);
}

static void relayTick() {
  bool want = relayShouldBeOn();
  if (want != gRelayState) {
    gRelayState = want;
    relayApplyPin();
  }
}

static void applyApConfig() {
  wifi_config_t wc;
  memset(&wc, 0, sizeof(wc));
  esp_wifi_get_config(WIFI_IF_AP, &wc);

  strncpy((char*)wc.ap.ssid, gCfg.apSsid, sizeof(wc.ap.ssid));
  wc.ap.ssid_len = strlen(gCfg.apSsid);
  if (gCfg.apAuth && strlen(gCfg.apPass) >= 8) {
    strncpy((char*)wc.ap.password, gCfg.apPass, sizeof(wc.ap.password) - 1);
    wc.ap.password[sizeof(wc.ap.password) - 1] = 0;
    wc.ap.authmode = WIFI_AUTH_WPA2_PSK;
  } else {
    memset(wc.ap.password, 0, sizeof(wc.ap.password));
    wc.ap.authmode = WIFI_AUTH_OPEN;
    gCfg.apPass[0] = 0;
  }
  wc.ap.channel = gCfg.apChannel;
  wc.ap.ssid_hidden = gCfg.apHidden ? 1 : 0;
  wc.ap.max_connection = gCfg.apMaxConn;
  wc.ap.beacon_interval = 100;

  esp_wifi_set_config(WIFI_IF_AP, &wc);
  esp_wifi_set_max_tx_power(gCfg.apTxPower);
}

static void applyStaStaticIp() {
  if (gCfg.staDhcp) return;
  IPAddress ip(gCfg.staIp), gw(gCfg.staGw), mask(gCfg.staMask);
  IPAddress d1(gCfg.staDns1), d2(gCfg.staDns2);
  WiFi.config(ip, gw, mask, d1, d2);
}

static void staBegin() {
  if (gCfg.staSsid[0] == 0) {
    gStaWantConnect = false;
    return;
  }
  WiFi.disconnect(true, false);
  if (gCfg.staUseMac) {
    esp_err_t macErr = esp_wifi_set_mac(WIFI_IF_STA, (uint8_t*)gCfg.staMac);
    if (macErr != ESP_OK) {
      gCfg.staUseMac = 0;
      cfgMarkDirty();
    }
  }
  if (gCfg.staHostname[0]) {
    WiFi.setHostname(gCfg.staHostname);
  }
  applyStaStaticIp();
  WiFi.begin(gCfg.staSsid, gCfg.staPass);
  gStaWantConnect = true;
  gStaLastAttempt = millis();
  gStaAttemptGap = STA_BACKOFF_STEP;
}

static void staTick(uint32_t now) {
  if (!gStaWantConnect) return;
  wl_status_t s = WiFi.status();

  if (s != gStaLastStatus) {
    gStaLastStatus = s;
    if (s == WL_CONNECTED) {
      gStaAttemptGap = STA_BACKOFF_STEP;
    }
  }

  if (s != WL_CONNECTED &&
      (uint32_t)(now - gStaLastAttempt) >= gStaAttemptGap) {
    gStaLastAttempt = now;
    if (gStaAttemptGap < STA_BACKOFF_MAX) {
      gStaAttemptGap += STA_BACKOFF_STEP;
      if (gStaAttemptGap > STA_BACKOFF_MAX) gStaAttemptGap = STA_BACKOFF_MAX;
    }
    WiFi.reconnect();
  }
}

static void factoryReset() {
  gPrefs.begin(CFG_NS, false);
  gPrefs.clear();
  gPrefs.end();
  ESP.restart();
}

static void buttonTick(uint32_t now) {
  bool pressed = (digitalRead(PIN_BOOT_BTN) == LOW);
  if (pressed && !gBtnPrev) {
    gBtnPressMs = now;
    gBtnArmed = false;
  }
  uint32_t held = now - gBtnPressMs;
  if (pressed && !gBtnArmed && held >= BTN_DEBOUNCE_MS) {
    gBtnArmed = true;
  }
  if (pressed && gBtnArmed && held >= FACTORY_HOLD_MS) {
    factoryReset();
  }
  gBtnPrev = pressed;
}

static String ipToStr(uint32_t v) {
  IPAddress ip(v);
  return ip.toString();
}

static uint32_t strToIp(const String& s) {
  IPAddress ip;
  if (!ip.fromString(s)) return 0;
  return (uint32_t)ip;
}

static bool macToStr(const uint8_t m[6], char out[18]) {
  snprintf(out, 18, "%02x:%02x:%02x:%02x:%02x:%02x",
           m[0], m[1], m[2], m[3], m[4], m[5]);
  return true;
}

static bool strToMac(const String& s, uint8_t m[6]) {
  String t = s;
  t.replace('-', ':');
  t.toLowerCase();
  int v[6];
  int consumed = 0;
  int n = sscanf(t.c_str(), "%x:%x:%x:%x:%x:%x%n",
                 &v[0], &v[1], &v[2], &v[3], &v[4], &v[5], &consumed);
  if (n != 6) return false;
  if (consumed != (int)t.length()) return false;
  for (int i = 0; i < 6; i++) {
    if (v[i] < 0 || v[i] > 255) return false;
    m[i] = (uint8_t)v[i];
  }
  bool all0 = true, allF = true;
  for (int i = 0; i < 6; i++) {
    if (m[i]) all0 = false;
    if (m[i] != 0xFF) allF = false;
  }
  if (all0 || allF) return false;
  return true;
}

static const char PAGE_HTML[] PROGMEM = R"HTML(
<!doctype html><html><head><meta charset="utf-8">
<meta name="viewport" content="width=device-width,initial-scale=1">
<meta name="color-scheme" content="dark">
<title>ESP32 Dashboard</title>
<style>
:root{--bg:#0f172a;--card:#1e293b;--fg:#e2e8f0;--ac:#38bdf8;--er:#f87171;--ok:#4ade80;--wn:#fbbf24}
*{box-sizing:border-box}
body{margin:0;font-family:system-ui,-apple-system,Segoe UI,Roboto,sans-serif;background:var(--bg);color:var(--fg)}
header{padding:12px 16px;background:#111827;display:flex;justify-content:space-between;align-items:center;flex-wrap:wrap;gap:8px}
header h1{margin:0;font-size:18px}
nav{display:flex;gap:4px;flex-wrap:wrap}
nav button{background:transparent;color:var(--fg);border:1px solid #334155;padding:6px 10px;border-radius:6px;cursor:pointer;font-size:13px}
nav button.active{background:var(--ac);color:#0f172a;border-color:var(--ac)}
main{padding:16px;max-width:1000px;margin:0 auto}
.card{background:var(--card);border-radius:10px;padding:16px;margin-bottom:16px}
.card h2{margin:0 0 12px;font-size:15px;color:var(--ac)}
.grid{display:grid;grid-template-columns:repeat(auto-fit,minmax(160px,1fr));gap:12px}
.metric{background:#0b1220;border-radius:8px;padding:12px;text-align:center}
.metric .v{font-size:24px;font-weight:600}
.metric .l{font-size:12px;opacity:.7;margin-top:4px}
.metric.warn{border:1px solid var(--wn)}
.metric.er{border:1px solid var(--er)}
.row{display:flex;gap:8px;flex-wrap:wrap;margin-bottom:8px;align-items:center}
.row label{min-width:140px;font-size:13px;opacity:.85}
.row input,.row select{background:#0b1220;color:var(--fg);border:1px solid #334155;border-radius:6px;padding:8px;flex:1;min-width:120px;font-size:14px}
button.act{background:var(--ac);color:#0f172a;border:none;padding:10px 16px;border-radius:6px;cursor:pointer;font-weight:600}
button.act:hover{filter:brightness(1.1)}
button.danger{background:var(--er);color:#fff}
.status{font-size:12px;opacity:.85;padding:4px 0}
.ok{color:var(--ok)} .er{color:var(--er)} .wn{color:var(--wn)}
table{width:100%;border-collapse:collapse;font-size:13px}
td{padding:6px 4px;border-bottom:1px solid #334155;word-break:break-all}
.hide{display:none}
.banner{background:#7c2d12;color:#fff;padding:8px 16px;font-size:13px;display:none}
.banner.show{display:block}
.overlay{position:fixed;top:50%;left:50%;transform:translate(-50%,-50%);background:#1e293b;padding:24px;border-radius:10px;z-index:999;color:#e2e8f0;font-size:15px;box-shadow:0 10px 40px rgba(0,0,0,.5)}
@media(max-width:600px){.row label{min-width:100%}}
</style></head><body>
<div class="banner" id="rebootBanner">Configuration changed. <b>Reboot required</b> to take effect.
<button class="act" id="rebootNowBtn" style="margin-left:8px;padding:4px 10px">Reboot Now</button></div>
<header><h1>ESP32 Dashboard</h1>
<nav>
<button data-p="dash" class="active">Dashboard</button>
<button data-p="thr">Thresholds</button>
<button data-p="ap">WiFi AP</button>
<button data-p="sta">WiFi STA</button>
<button data-p="sys">System</button>
</nav></header>
<main>

<section id="p-dash">
<div class="card"><h2>Environment</h2>
<div class="grid">
<div class="metric" id="cardT"><div class="v" id="mT">--</div><div class="l">Temperature °C</div></div>
<div class="metric" id="cardH"><div class="v" id="mH">--</div><div class="l">Humidity %</div></div>
<div class="metric" id="cardP"><div class="v" id="mP">--</div><div class="l">Pressure hPa</div></div>
</div>
<div class="status" id="sensorStatus"></div>
<div class="status" id="alertStatus"></div>
<div class="status" id="relayStatus"></div>
</div>
<div class="card"><h2>Network</h2>
<table>
<tr><td>AP SSID</td><td id="nApSsid">-</td></tr>
<tr><td>AP IP</td><td id="nApIp">-</td></tr>
<tr><td>AP Clients</td><td id="nApClients">-</td></tr>
<tr><td>STA Status</td><td id="nStaStatus">-</td></tr>
<tr><td>STA IP</td><td id="nStaIp">-</td></tr>
<tr><td>STA RSSI</td><td id="nStaRssi">-</td></tr>
<tr><td>Hostname</td><td id="nHost">-</td></tr>
</table>
</div>
</section>

<section id="p-thr" class="hide">
<div class="card"><h2>BME280 Thresholds</h2>
<div class="row"><label>Alerts</label><select id="thrEn"><option value="1">Enabled</option><option value="0">Disabled</option></select></div>
<div class="row"><label>Temp min (°C)</label><input id="thrTmin" type="number" step="0.1" min="-40" max="85"></div>
<div class="row"><label>Temp max (°C)</label><input id="thrTmax" type="number" step="0.1" min="-40" max="85"></div>
<div class="row"><label>Humidity min (%)</label><input id="thrHmin" type="number" step="0.1" min="0" max="100"></div>
<div class="row"><label>Humidity max (%)</label><input id="thrHmax" type="number" step="0.1" min="0" max="100"></div>
<div class="row"><label>Pressure min (hPa)</label><input id="thrPmin" type="number" step="0.1" min="300" max="1100"></div>
<div class="row"><label>Pressure max (hPa)</label><input id="thrPmax" type="number" step="0.1" min="300" max="1100"></div>
<button class="act" id="thrSave">Save Thresholds</button>
<span class="status" id="thrMsg"></span>
</div>
<div class="card"><h2>Relay (GPIO 25)</h2>
<div class="row"><label>Relay enabled</label><select id="relayEn"><option value="0">No</option><option value="1">Yes</option></select></div>
<div class="row"><label>Logic</label><select id="relayLogic"><option value="0">OR (any alert)</option><option value="1">AND (all alerts)</option></select></div>
<div class="row"><label>React to temperature</label><select id="relayUseT"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="row"><label>React to humidity</label><select id="relayUseH"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="row"><label>React to pressure</label><select id="relayUseP"><option value="1">Yes</option><option value="0">No</option></select></div>
<div class="row"><label>Active-low relay</label><select id="relayInv"><option value="0">No</option><option value="1">Yes</option></select></div>
<button class="act" id="relaySave">Save Relay Settings</button>
<span class="status" id="relayMsg"></span>
</div>
</section>

<section id="p-ap" class="hide">
<div class="card"><h2>WiFi AP Settings</h2>
<div class="row"><label>SSID</label><input id="apSsid" maxlength="32"></div>
<div class="row"><label>Password</label><input id="apPass" maxlength="64" type="password"></div>
<div class="row"><label>Authentication</label><select id="apAuth"><option value="1">WPA2</option><option value="0">Open</option></select></div>
<div class="row"><label>Channel</label><select id="apCh"></select></div>
<div class="row"><label>Hidden SSID</label><select id="apHidden"><option value="0">No</option><option value="1">Yes</option></select></div>
<div class="row"><label>Max clients</label><input id="apMax" type="number" min="1" max="8"></div>
<div class="row"><label>TX power (0-84)</label><input id="apTx" type="number" min="0" max="84"></div>
<button class="act" id="apSave">Apply AP</button>
<span class="status" id="apMsg"></span>
</div>
</section>

<section id="p-sta" class="hide">
<div class="card"><h2>WiFi STA Settings</h2>
<div class="row"><label>SSID</label><input id="staSsid" maxlength="32"></div>
<div class="row"><label>Password</label><input id="staPass" maxlength="64" type="password"></div>
<div class="row"><label>Mode</label><select id="staDhcp"><option value="1">DHCP</option><option value="0">Static</option></select></div>
<div class="row"><label>IP</label><input id="staIp"></div>
<div class="row"><label>Gateway</label><input id="staGw"></div>
<div class="row"><label>Subnet mask</label><input id="staMask"></div>
<div class="row"><label>DNS1</label><input id="staDns1"></div>
<div class="row"><label>DNS2</label><input id="staDns2"></div>
<div class="row"><label>Hostname</label><input id="staHost" maxlength="32"></div>
<div class="row"><label>Custom MAC</label><select id="staUseMac"><option value="0">Default</option><option value="1">Custom</option></select></div>
<div class="row"><label>MAC (aa:bb:cc:dd:ee:ff)</label><input id="staMac"></div>
<button class="act" id="staSave">Apply STA</button>
<span class="status" id="staMsg"></span>
</div>
</section>

<section id="p-sys" class="hide">
<div class="card"><h2>System</h2>
<table>
<tr><td>Firmware</td><td>ESP32 Unified v3</td></tr>
<tr><td>Uptime</td><td id="sUp">-</td></tr>
<tr><td>Free heap</td><td id="sHeap">-</td></tr>
<tr><td>Min free heap</td><td id="sHeapMin">-</td></tr>
<tr><td>Reset reason</td><td id="sRst">-</td></tr>
</table>
</div>
<div class="card"><h2>Change Admin Password</h2>
<div class="row"><label>New password</label><input id="newAuth" type="password" maxlength="32"></div>
<button class="act" id="btnAuth">Change Password</button>
<span class="status" id="authMsg"></span>
</div>
<div class="card"><h2>Actions</h2>
<div class="row">
<button class="act" id="btnReboot">Reboot</button>
<button class="act danger" id="btnFactory">Factory Reset</button>
</div>
<span class="status" id="sysMsg"></span>
</div>
</section>

</main>
<script>
var $=function(id){return document.getElementById(id);};
var page='dash';
var rebootNeeded=false;
var inflight=0;
var busyTimers=[];
function busy(){
  inflight++;
  var t=setTimeout(function(){
    while(busyTimers.length){clearTimeout(busyTimers.shift());}
    inflight=0;
  },10000);
  busyTimers.push(t);
}
function idle(){
  while(busyTimers.length){clearTimeout(busyTimers.shift());}
  inflight=0;
}
function api(path,opts){return fetch(path,opts).then(function(r){return r.json();});}
function showBanner(){rebootNeeded=true;$('rebootBanner').classList.add('show');}
function overlay(msg,color){
  var btns=document.querySelectorAll('button');
  for(var i=0;i<btns.length;i++){btns[i].disabled=true;}
  document.body.style.opacity='0.5';
  var d=document.createElement('div');
  d.className='overlay';
  if(color){d.style.background=color;}
  d.textContent=msg;
  d.id='__overlay';
  document.body.appendChild(d);
}
function doReboot(){
  overlay('Rebooting, please wait...',null);
  api('/api/reboot',{method:'POST'}).catch(function(){}).then(function(){
    setTimeout(function(){location.reload();},6000);
  });
}
var failCount=0;
function activateTab(p){
  document.querySelectorAll('nav button').forEach(function(x){
    x.classList.toggle('active',x.dataset.p===p);
  });
  page=p;
  ['dash','thr','ap','sta','sys'].forEach(function(q){$('p-'+q).classList.toggle('hide',q!==page);});
  if(page==='thr')loadThr();
  if(page==='ap')loadAp();
  if(page==='sta')loadSta();
  if(page==='sys')loadSys();
}
window.addEventListener('hashchange',function(){
  var h=location.hash.replace('#','');
  if(['dash','thr','ap','sta','sys'].indexOf(h)>=0&&inflight===0){activateTab(h);}
});
document.querySelectorAll('nav button').forEach(function(b){
  b.onclick=function(){
    var newPage=b.dataset.p;
    if(newPage===page){return;}
    if(inflight>0){return;}
    location.hash=newPage;
  };
});
(function(){var s=$('apCh');for(var i=1;i<=13;i++){var o=document.createElement('option');o.value=i;o.textContent=i;s.appendChild(o);}})();
function num2(v){return (typeof v==='number')?v.toFixed(1):'--';}
function refreshDash(){
  api('/api/status').then(function(d){
    failCount=0;
    $('sensorStatus').classList.remove('er');
    $('mT').textContent=num2(d.t);
    $('mH').textContent=num2(d.h);
    $('mP').textContent=num2(d.p);
    var alertsOn=d.alertsEn?1:0;
    $('cardT').className='metric'+((alertsOn&&d.tAlert)?' warn':'');
    $('cardH').className='metric'+((alertsOn&&d.hAlert)?' warn':'');
    $('cardP').className='metric'+((alertsOn&&d.pAlert)?' warn':'');
    if(d.sensorOk){$('sensorStatus').innerHTML='<span class="ok">Sensor OK at 0x'+d.sensorAddr.toString(16).toUpperCase()+'</span>';}
    else{$('sensorStatus').innerHTML='<span class="er">Sensor not detected</span>';}
    var al=[];
    if(alertsOn&&d.tAlert)al.push('Temperature out of range');
    if(alertsOn&&d.hAlert)al.push('Humidity out of range');
    if(alertsOn&&d.pAlert)al.push('Pressure out of range');
    if(al.length){$('alertStatus').innerHTML='<span class="wn">'+al.join(' · ')+'</span>';}
    else{$('alertStatus').textContent='';}
    if(d.relayEn){
      $('relayStatus').innerHTML=d.relayOn
        ?'<span class="wn">Relay: ON (GPIO 25)</span>'
        :'<span class="ok">Relay: OFF (GPIO 25)</span>';
    }else{
      $('relayStatus').innerHTML='<span style="opacity:.6">Relay: disabled</span>';
    }
    $('nApSsid').textContent=d.apSsid;
    $('nApIp').textContent=d.apIp;
    $('nApClients').textContent=d.apClients;
    $('nStaStatus').textContent=d.staStatus;
    $('nStaIp').textContent=d.staIp;
    $('nStaRssi').textContent=(d.staStatus==='connected')?(d.staRssi+' dBm'):'—';
    $('nHost').textContent=d.hostname;
  }).catch(function(){
    failCount++;
    if(failCount>=3){
      $('sensorStatus').innerHTML='<span class="er">Connection lost — values stale</span>';
    }
  });
}
setInterval(function(){if(page==='dash')refreshDash();},2000);
refreshDash();
function loadThr(){
  $('thrMsg').textContent='';
  $('relayMsg').textContent='';
  api('/api/thresholds').then(function(d){
    $('thrEn').value=d.en;$('thrTmin').value=d.tMin;$('thrTmax').value=d.tMax;
    $('thrHmin').value=d.hMin;$('thrHmax').value=d.hMax;
    $('thrPmin').value=d.pMin;$('thrPmax').value=d.pMax;
    $('relayEn').value=d.relayEn;
    $('relayLogic').value=d.relayLogic;
    $('relayUseT').value=d.relayUseT;
    $('relayUseH').value=d.relayUseH;
    $('relayUseP').value=d.relayUseP;
    $('relayInv').value=d.relayInv;
  });
}
function parseField(id,name,min,max){
  var el=$(id);
  var raw=el.value.trim();
  if(raw===''){return {err:name+' is required'};}
  var v=Number(raw);
  if(isNaN(v)||!isFinite(v)){return {err:name+' must be a number'};}
  if(v<min||v>max){return {err:name+' must be between '+min+' and '+max};}
  return {v:v};
}
$('thrSave').onclick=function(){
  if(inflight>0){$('thrMsg').innerHTML='<span class="wn">Please wait...</span>';return;}
  $('thrMsg').textContent='';
  var tMin=parseField('thrTmin','Temp min',-40,85);   if(tMin.err){$('thrMsg').innerHTML='<span class="er">'+tMin.err+'</span>';return;}
  var tMax=parseField('thrTmax','Temp max',-40,85);   if(tMax.err){$('thrMsg').innerHTML='<span class="er">'+tMax.err+'</span>';return;}
  var hMin=parseField('thrHmin','Humidity min',0,100);if(hMin.err){$('thrMsg').innerHTML='<span class="er">'+hMin.err+'</span>';return;}
  var hMax=parseField('thrHmax','Humidity max',0,100);if(hMax.err){$('thrMsg').innerHTML='<span class="er">'+hMax.err+'</span>';return;}
  var pMin=parseField('thrPmin','Pressure min',300,1100);if(pMin.err){$('thrMsg').innerHTML='<span class="er">'+pMin.err+'</span>';return;}
  var pMax=parseField('thrPmax','Pressure max',300,1100);if(pMax.err){$('thrMsg').innerHTML='<span class="er">'+pMax.err+'</span>';return;}
  if(tMin.v>=tMax.v){$('thrMsg').innerHTML='<span class="er">Temp min must be less than Temp max</span>';return;}
  if(hMin.v>=hMax.v){$('thrMsg').innerHTML='<span class="er">Humidity min must be less than Humidity max</span>';return;}
  if(pMin.v>=pMax.v){$('thrMsg').innerHTML='<span class="er">Pressure min must be less than Pressure max</span>';return;}
  var body=JSON.stringify({en:+$('thrEn').value,tMin:tMin.v,tMax:tMax.v,hMin:hMin.v,hMax:hMax.v,pMin:pMin.v,pMax:pMax.v});
  $('thrMsg').innerHTML='<span>Saving...</span>';
  busy();
  api('/api/thresholds',{method:'POST',headers:{'Content-Type':'application/json'},body:body}).then(function(d){
    if(d.ok){$('thrMsg').innerHTML='<span class="ok">Saved.</span>';}
    else{$('thrMsg').innerHTML='<span class="er">Error: '+d.err+'</span>';}
  }).catch(function(){$('thrMsg').innerHTML='<span class="er">Network error</span>';}).then(idle);
};
$('relaySave').onclick=function(){
  if(inflight>0){$('relayMsg').innerHTML='<span class="wn">Please wait...</span>';return;}
  $('relayMsg').textContent='';
  var rEn=+$('relayEn').value;
  var rT=+$('relayUseT').value,rH=+$('relayUseH').value,rP=+$('relayUseP').value;
  if(rEn===1&&rT===0&&rH===0&&rP===0){
    $('relayMsg').innerHTML='<span class="er">Select at least one metric</span>';return;
  }
  var body=JSON.stringify({
    relayEn:rEn,relayLogic:+$('relayLogic').value,
    relayUseT:rT,relayUseH:rH,relayUseP:rP,relayInv:+$('relayInv').value
  });
  $('relayMsg').innerHTML='<span>Saving...</span>';
  busy();
  api('/api/relay',{method:'POST',headers:{'Content-Type':'application/json'},body:body}).then(function(d){
    if(d.ok){$('relayMsg').innerHTML='<span class="ok">Saved.</span>';}
    else{$('relayMsg').innerHTML='<span class="er">Error: '+d.err+'</span>';}
  }).catch(function(){$('relayMsg').innerHTML='<span class="er">Network error</span>';}).then(idle);
};
function loadAp(){
  $('apMsg').textContent='';
  api('/api/ap').then(function(d){
    $('apSsid').value=d.ssid;
    $('apPass').value='';
    $('apPass').placeholder=d.auth?'(unchanged if empty)':'(open)';
    $('apAuth').value=d.auth;$('apCh').value=d.ch;
    $('apHidden').value=d.hidden;$('apMax').value=d.max;
    $('apTx').value=d.tx;
  });
}
$('apSave').onclick=function(){
  if(inflight>0){$('apMsg').innerHTML='<span class="wn">Please wait...</span>';return;}
  var body=JSON.stringify({ssid:$('apSsid').value,pass:$('apPass').value,auth:+$('apAuth').value,ch:+$('apCh').value,hidden:+$('apHidden').value,max:+$('apMax').value,tx:+$('apTx').value});
  busy();
  api('/api/ap',{method:'POST',headers:{'Content-Type':'application/json'},body:body}).then(function(d){
    if(d.ok){$('apMsg').innerHTML='<span class="ok">Applied.</span>';$('apPass').value='';}
    else{$('apMsg').innerHTML='<span class="er">Error: '+d.err+'</span>';}
  }).catch(function(){$('apMsg').innerHTML='<span class="er">Network error</span>';}).then(idle);
};
function emptyIfZero(s){return (s==='0.0.0.0')?'':s;}
function loadSta(){
  $('staMsg').textContent='';
  api('/api/sta').then(function(d){
    $('staSsid').value=d.ssid;
    $('staPass').value='';
    $('staPass').placeholder=d.ssid?'(unchanged if empty)':'(none)';
    $('staDhcp').value=d.dhcp;
    $('staIp').value=emptyIfZero(d.ip);
    $('staGw').value=emptyIfZero(d.gw);
    $('staMask').value=emptyIfZero(d.mask);
    $('staDns1').value=emptyIfZero(d.dns1);
    $('staDns2').value=emptyIfZero(d.dns2);
    $('staHost').value=d.host;
    $('staUseMac').value=d.useMac;
    $('staMac').value=d.mac;
  });
}
$('staSave').onclick=function(){
  if(inflight>0){$('staMsg').innerHTML='<span class="wn">Please wait...</span>';return;}
  if(+$('staDhcp').value===0){
    if(!$('staIp').value.trim()){$('staMsg').innerHTML='<span class="er">IP is required for static mode</span>';return;}
    if(!$('staGw').value.trim()){$('staMsg').innerHTML='<span class="er">Gateway is required for static mode</span>';return;}
    if(!$('staMask').value.trim()){$('staMsg').innerHTML='<span class="er">Subnet mask is required for static mode</span>';return;}
  }
  if(+$('staUseMac').value===1){
    if(!$('staMac').value.trim()){$('staMsg').innerHTML='<span class="er">MAC address is required</span>';return;}
  }
  var body=JSON.stringify({ssid:$('staSsid').value,pass:$('staPass').value,dhcp:+$('staDhcp').value,ip:$('staIp').value,gw:$('staGw').value,mask:$('staMask').value,dns1:$('staDns1').value,dns2:$('staDns2').value,host:$('staHost').value,useMac:+$('staUseMac').value,mac:$('staMac').value});
  busy();
  api('/api/sta',{method:'POST',headers:{'Content-Type':'application/json'},body:body}).then(function(d){
    if(d.ok){
      $('staMsg').innerHTML='<span class="ok">Saved.</span>';
      $('staPass').value='';
      if(d.rebootRequired){showBanner();}
    } else {
      $('staMsg').innerHTML='<span class="er">Error: '+d.err+'</span>';
    }
  }).catch(function(){$('staMsg').innerHTML='<span class="er">Network error</span>';}).then(idle);
};
function loadSys(){
  api('/api/sys').then(function(d){
    var s=d.up,total=s,d2=Math.floor(s/86400),h=Math.floor((s%86400)/3600),m=Math.floor((s%3600)/60),sec=s%60;
    var parts=[];
    if(d2>0)parts.push(d2+'d');
    if(h>0)parts.push(h+'h');
    if(m>0)parts.push(m+'m');
    parts.push(sec+'s');
    $('sUp').textContent=parts.join(' ');
    $('sHeap').textContent=d.heap+' bytes';
    $('sHeapMin').textContent=d.heapMin+' bytes';
    $('sRst').textContent=d.rst;
  });
}
$('btnAuth').onclick=function(){
  var p=$('newAuth').value;
  if(p.length<8){$('authMsg').innerHTML='<span class="er">Min 8 chars</span>';return;}
  $('authMsg').textContent='';
  api('/api/auth',{method:'POST',headers:{'Content-Type':'application/json'},body:JSON.stringify({pass:p})}).then(function(d){
    if(d.ok){$('authMsg').innerHTML='<span class="ok">Changed. Reload page to log in.</span>';$('newAuth').value='';}
    else{$('authMsg').innerHTML='<span class="er">Failed</span>';}
  }).catch(function(){$('authMsg').innerHTML='<span class="er">Network error</span>';});
};
$('btnReboot').onclick=function(){if(confirm('Reboot?'))doReboot();};
$('btnFactory').onclick=function(){
  var a=prompt('Type RESET to erase ALL settings and reboot:');
  if(a!=='RESET'){return;}
  overlay('Factory reset in progress...','#7c2d12');
  api('/api/factory',{method:'POST'}).catch(function(){}).then(function(){
    setTimeout(function(){
      var host=location.hostname;
      if(!host||host==='0.0.0.0'){host='192.168.4.1';}
      location.href='http://'+host+'/';
    },10000);
  });
};
document.addEventListener('DOMContentLoaded',function(){
  var b=document.getElementById('rebootNowBtn');
  if(b){b.onclick=function(){b.disabled=true;b.textContent='Rebooting...';doReboot();};}
  var h=location.hash.replace('#','');
  if(['dash','thr','ap','sta','sys'].indexOf(h)>=0){activateTab(h);}
});
</script>
</body></html>
)HTML";

static bool checkAuth() {
  if (gServer.authenticate("admin", gAuthPass)) return true;
  gServer.requestAuthentication();
  return false;
}

static void sendHtml() {
  gServer.sendHeader("Cache-Control", "no-store");
  gServer.send_P(200, "text/html", PAGE_HTML);
}

static void sendJsonOk() {
  gServer.send(200, "application/json", "{\"ok\":true}");
}

static void sendJsonErr(const char* e) {
  snprintf(gJsonBuf, sizeof(gJsonBuf), "{\"ok\":false,\"err\":\"%s\"}", e);
  gServer.send(400, "application/json", gJsonBuf);
}

static void handleStatus() {
  if (!checkAuth()) return;
  bool tAlert = !isnan(gT) && (gT < gCfg.tMin || gT > gCfg.tMax);
  bool hAlert = !isnan(gH) && (gH < gCfg.hMin || gH > gCfg.hMax);
  bool pAlert = !isnan(gP) && (gP < gCfg.pMin || gP > gCfg.pMax);

  char tB[12], hB[12], pB[12];
  if (isnan(gT)) strcpy(tB, "null"); else snprintf(tB, sizeof(tB), "%.2f", gT);
  if (isnan(gH)) strcpy(hB, "null"); else snprintf(hB, sizeof(hB), "%.2f", gH);
  if (isnan(gP)) strcpy(pB, "null"); else snprintf(pB, sizeof(pB), "%.2f", gP);

  snprintf(gJsonBuf, sizeof(gJsonBuf),
    "{\"t\":%s,\"h\":%s,\"p\":%s,\"tAlert\":%s,\"hAlert\":%s,\"pAlert\":%s,"
    "\"alertsEn\":%u,\"sensorOk\":%s,\"sensorAddr\":%u,"
    "\"apSsid\":\"%s\",\"apIp\":\"%s\",\"apClients\":%d,"
    "\"staStatus\":\"%s\",\"staIp\":\"%s\",\"staRssi\":%d,\"hostname\":\"%s\","
    "\"relayOn\":%s,\"relayEn\":%u}",
    tB, hB, pB,
    tAlert ? "true" : "false",
    hAlert ? "true" : "false",
    pAlert ? "true" : "false",
    (unsigned)gCfg.alertsEnabled,
    gBmeOk ? "true" : "false",
    (unsigned)gBmeAddr,
    gCfg.apSsid,
    WiFi.softAPIP().toString().c_str(),
    WiFi.softAPgetStationNum(),
    WiFi.status() == WL_CONNECTED ? "connected" : "disconnected",
    WiFi.status() == WL_CONNECTED ? WiFi.localIP().toString().c_str() : "-",
    WiFi.status() == WL_CONNECTED ? WiFi.RSSI() : 0,
    gCfg.staHostname,
    gRelayState ? "true" : "false",
    (unsigned)gCfg.relayEnabled
  );
  gServer.send(200, "application/json", gJsonBuf);
}

static bool jsonGetFloat(const String& body, const char* key, float& out) {
  String k = String("\"") + key + "\":";
  int i = body.indexOf(k);
  if (i < 0) return false;
  i += k.length();
  while (i < (int)body.length() && (body[i] == ' ' || body[i] == '\t')) i++;
  int j = i;
  int dots = 0;
  while (j < (int)body.length() && (body[j] == '-' || body[j] == '+' || body[j] == '.' || (body[j] >= '0' && body[j] <= '9'))) {
    if (body[j] == '.') dots++;
    j++;
  }
  if (j == i || dots > 1) return false;
  out = body.substring(i, j).toFloat();
  return true;
}

static bool jsonGetUInt(const String& body, const char* key, uint32_t& out) {
  String k = String("\"") + key + "\":";
  int i = body.indexOf(k);
  if (i < 0) return false;
  i += k.length();
  while (i < (int)body.length() && (body[i] == ' ' || body[i] == '\t')) i++;
  int j = i;
  while (j < (int)body.length() && body[j] >= '0' && body[j] <= '9') j++;
  if (j == i) return false;
  out = (uint32_t)body.substring(i, j).toInt();
  return true;
}

static bool jsonGetStr(const String& body, const char* key, String& out) {
  String k = String("\"") + key + "\":\"";
  int i = body.indexOf(k);
  if (i < 0) return false;
  i += k.length();
  String res;
  while (i < (int)body.length()) {
    char c = body[i];
    if (c == '"') { out = res; return true; }
    if (c == '\\' && i + 1 < (int)body.length()) {
      char n = body[i + 1];
      if (n == '"' || n == '\\' || n == '/') { res += n; i += 2; continue; }
      if (n == 'n') { res += '\n'; i += 2; continue; }
      if (n == 't') { res += '\t'; i += 2; continue; }
      return false;
    }
    res += c;
    i++;
  }
  return false;
}

static void handleGetThr() {
  if (!checkAuth()) return;
  snprintf(gJsonBuf, sizeof(gJsonBuf),
    "{\"en\":%u,\"tMin\":%.2f,\"tMax\":%.2f,\"hMin\":%.2f,\"hMax\":%.2f,\"pMin\":%.2f,\"pMax\":%.2f,"
    "\"relayEn\":%u,\"relayInv\":%u,\"relayUseT\":%u,\"relayUseH\":%u,\"relayUseP\":%u,\"relayLogic\":%u}",
    (unsigned)gCfg.alertsEnabled, gCfg.tMin, gCfg.tMax, gCfg.hMin, gCfg.hMax, gCfg.pMin, gCfg.pMax,
    (unsigned)gCfg.relayEnabled, (unsigned)gCfg.relayInvert,
    (unsigned)gCfg.relayUseT, (unsigned)gCfg.relayUseH,
    (unsigned)gCfg.relayUseP, (unsigned)gCfg.relayLogic);
  gServer.send(200, "application/json", gJsonBuf);
}

static void handlePostThr() {
  if (!checkAuth()) return;
  String body = gServer.arg("plain");
  uint32_t en = 0;
  float tMin, tMax, hMin, hMax, pMin, pMax;
  if (!jsonGetUInt(body, "en", en) || !jsonGetFloat(body, "tMin", tMin) ||
      !jsonGetFloat(body, "tMax", tMax) || !jsonGetFloat(body, "hMin", hMin) ||
      !jsonGetFloat(body, "hMax", hMax) || !jsonGetFloat(body, "pMin", pMin) ||
      !jsonGetFloat(body, "pMax", pMax)) {
    sendJsonErr("bad json"); return;
  }
  if (tMin >= tMax) { sendJsonErr("tMin>=tMax"); return; }
  if (hMin >= hMax) { sendJsonErr("hMin>=hMax"); return; }
  if (pMin >= pMax) { sendJsonErr("pMin>=pMax"); return; }
  if (tMin < -40 || tMax > 85) { sendJsonErr("t range"); return; }
  if (hMin < 0 || hMax > 100) { sendJsonErr("h range"); return; }
  if (pMin < 300 || pMax > 1100) { sendJsonErr("p range"); return; }
  if (en > 1) { sendJsonErr("en"); return; }

  gCfg.alertsEnabled = (uint8_t)en;
  gCfg.tMin = tMin; gCfg.tMax = tMax;
  gCfg.hMin = hMin; gCfg.hMax = hMax;
  gCfg.pMin = pMin; gCfg.pMax = pMax;
  cfgMarkDirty();
  saveConfigForce();
  relayTick();
  sendJsonOk();
}

static void handlePostRelay() {
  if (!checkAuth()) return;
  String body = gServer.arg("plain");
  uint32_t en = 0, inv = 0, useT = 0, useH = 0, useP = 0, logic = 0;
  if (!jsonGetUInt(body, "relayEn", en) || !jsonGetUInt(body, "relayInv", inv) ||
      !jsonGetUInt(body, "relayUseT", useT) || !jsonGetUInt(body, "relayUseH", useH) ||
      !jsonGetUInt(body, "relayUseP", useP) || !jsonGetUInt(body, "relayLogic", logic)) {
    sendJsonErr("bad json"); return;
  }
  if (en > 1 || inv > 1 || useT > 1 || useH > 1 || useP > 1 || logic > 1) {
    sendJsonErr("relay bool"); return;
  }
  if (en == 1 && useT == 0 && useH == 0 && useP == 0) {
    sendJsonErr("relay: no metric selected"); return;
  }

  gCfg.relayEnabled = (uint8_t)en;
  gCfg.relayInvert  = (uint8_t)inv;
  gCfg.relayUseT    = (uint8_t)useT;
  gCfg.relayUseH    = (uint8_t)useH;
  gCfg.relayUseP    = (uint8_t)useP;
  gCfg.relayLogic   = (uint8_t)logic;
  cfgMarkDirty();
  saveConfigForce();
  relayTick();
  sendJsonOk();
}

static void handleGetAp() {
  if (!checkAuth()) return;
  snprintf(gJsonBuf, sizeof(gJsonBuf),
    "{\"ssid\":\"%s\",\"pass\":\"\",\"auth\":%u,\"ch\":%u,\"hidden\":%u,\"max\":%u,\"tx\":%d}",
    gCfg.apSsid, gCfg.apAuth, gCfg.apChannel,
    gCfg.apHidden, gCfg.apMaxConn, gCfg.apTxPower);
  gServer.send(200, "application/json", gJsonBuf);
}

static void handlePostAp() {
  if (!checkAuth()) return;
  String body = gServer.arg("plain");
  String ssid, pass;
  uint32_t auth = 0, ch = 0, hidden = 0, maxc = 0, tx = 0;
  if (!jsonGetStr(body, "ssid", ssid) || !jsonGetStr(body, "pass", pass) ||
      !jsonGetUInt(body, "auth", auth) || !jsonGetUInt(body, "ch", ch) ||
      !jsonGetUInt(body, "hidden", hidden) || !jsonGetUInt(body, "max", maxc) ||
      !jsonGetUInt(body, "tx", tx)) {
    sendJsonErr("bad json"); return;
  }
  if (ssid.length() == 0 || ssid.length() > 32) { sendJsonErr("bad ssid"); return; }
  for (size_t i = 0; i < ssid.length(); i++) {
    uint8_t c = (uint8_t)ssid[i];
    if (c == '"' || c == '\\' || c < 0x20 || c == 0x7F) { sendJsonErr("ssid charset"); return; }
  }
  if (auth > 1) { sendJsonErr("bad auth"); return; }
  if (auth == 1 && pass.length() > 0 && (pass.length() < 8 || pass.length() > 63)) { sendJsonErr("bad pass"); return; }
  if (auth == 1 && pass.length() == 0 && strlen(gCfg.apPass) < 8) { sendJsonErr("bad pass"); return; }
  if (ch < 1 || ch > 13) { sendJsonErr("bad channel"); return; }
  if (hidden > 1) { sendJsonErr("bad hidden"); return; }
  if (maxc < 1 || maxc > 8) { sendJsonErr("bad max"); return; }
  if (tx > 84) { sendJsonErr("bad tx"); return; }

  strncpy(gCfg.apSsid, ssid.c_str(), sizeof(gCfg.apSsid) - 1);
  gCfg.apSsid[32] = 0;
  if (pass.length() > 0) {
    strncpy(gCfg.apPass, pass.c_str(), sizeof(gCfg.apPass) - 1);
    gCfg.apPass[64] = 0;
  }
  gCfg.apAuth = (uint8_t)auth;
  gCfg.apChannel = (uint8_t)ch;
  gCfg.apHidden = (uint8_t)hidden;
  gCfg.apMaxConn = (uint8_t)maxc;
  gCfg.apTxPower = (int8_t)tx;
  cfgMarkDirty();
  saveConfigForce();
  applyApConfig();
  sendJsonOk();
}

static void handleGetSta() {
  if (!checkAuth()) return;
  char mac[18] = "";
  if (gCfg.staUseMac) macToStr(gCfg.staMac, mac);
  snprintf(gJsonBuf, sizeof(gJsonBuf),
    "{\"ssid\":\"%s\",\"pass\":\"\",\"dhcp\":%u,"
    "\"ip\":\"%s\",\"gw\":\"%s\",\"mask\":\"%s\",\"dns1\":\"%s\",\"dns2\":\"%s\","
    "\"host\":\"%s\",\"useMac\":%u,\"mac\":\"%s\"}",
    gCfg.staSsid, gCfg.staDhcp,
    ipToStr(gCfg.staIp).c_str(), ipToStr(gCfg.staGw).c_str(),
    ipToStr(gCfg.staMask).c_str(), ipToStr(gCfg.staDns1).c_str(),
    ipToStr(gCfg.staDns2).c_str(), gCfg.staHostname,
    gCfg.staUseMac, mac);
  gServer.send(200, "application/json", gJsonBuf);
}

static void handlePostSta() {
  if (!checkAuth()) return;
  String body = gServer.arg("plain");
  String ssid, pass, ip, gw, mask, dns1, dns2, host, mac;
  uint32_t dhcp = 1, useMac = 0;
  if (!jsonGetStr(body, "ssid", ssid) || !jsonGetStr(body, "pass", pass) ||
      !jsonGetUInt(body, "dhcp", dhcp) || !jsonGetStr(body, "host", host) ||
      !jsonGetUInt(body, "useMac", useMac)) {
    sendJsonErr("bad json"); return;
  }
  jsonGetStr(body, "ip", ip);
  jsonGetStr(body, "gw", gw);
  jsonGetStr(body, "mask", mask);
  jsonGetStr(body, "dns1", dns1);
  jsonGetStr(body, "dns2", dns2);
  jsonGetStr(body, "mac", mac);

  if (ssid.length() > 32) { sendJsonErr("bad ssid"); return; }
  for (size_t i = 0; i < ssid.length(); i++) {
    uint8_t c = (uint8_t)ssid[i];
    if (c == '"' || c == '\\' || c < 0x20 || c == 0x7F) { sendJsonErr("ssid charset"); return; }
  }
  if (pass.length() > 63) { sendJsonErr("bad pass"); return; }
  if (host.length() > 32) { sendJsonErr("bad host"); return; }
  for (size_t i = 0; i < host.length(); i++) {
    uint8_t c = (uint8_t)host[i];
    if (c == '"' || c == '\\' || c < 0x20 || c == 0x7F) { sendJsonErr("host charset"); return; }
  }
  if (dhcp > 1) { sendJsonErr("bad dhcp"); return; }
  if (useMac > 1) { sendJsonErr("bad useMac"); return; }

  if (dhcp == 0) {
    if (strToIp(ip) == 0 || strToIp(gw) == 0 || strToIp(mask) == 0) {
      sendJsonErr("bad static ip"); return;
    }
    uint32_t uip = strToIp(ip), ugw = strToIp(gw), umask = strToIp(mask);
    if ((uip & umask) == 0) { sendJsonErr("ip/mask"); return; }
    if ((ugw & umask) != (uip & umask)) { sendJsonErr("gw subnet"); return; }
  }

  bool macChanged = false;
  uint8_t newMac[6];
  if (useMac) {
    if (!strToMac(mac, newMac)) { sendJsonErr("bad mac"); return; }
    if (memcmp(newMac, gCfg.staMac, 6) != 0) macChanged = true;
  } else {
    if (gCfg.staUseMac) macChanged = true;
  }

  bool ipChanged =
    (gCfg.staDhcp != (uint8_t)dhcp) ||
    (!dhcp && (gCfg.staIp != strToIp(ip) || gCfg.staGw != strToIp(gw) ||
               gCfg.staMask != strToIp(mask) || gCfg.staDns1 != strToIp(dns1) ||
               gCfg.staDns2 != strToIp(dns2)));

  bool credChanged =
    (strcmp(gCfg.staSsid, ssid.c_str()) != 0) ||
    (pass.length() > 0 && strcmp(gCfg.staPass, pass.c_str()) != 0);

  strncpy(gCfg.staSsid, ssid.c_str(), sizeof(gCfg.staSsid) - 1);
  gCfg.staSsid[32] = 0;
  if (pass.length() > 0) {
    strncpy(gCfg.staPass, pass.c_str(), sizeof(gCfg.staPass) - 1);
    gCfg.staPass[64] = 0;
  }
  gCfg.staDhcp = (uint8_t)dhcp;
  gCfg.staIp = strToIp(ip);
  gCfg.staGw = strToIp(gw);
  gCfg.staMask = strToIp(mask);
  gCfg.staDns1 = strToIp(dns1);
  gCfg.staDns2 = strToIp(dns2);
  strncpy(gCfg.staHostname, host.c_str(), sizeof(gCfg.staHostname) - 1);
  gCfg.staHostname[32] = 0;
  gCfg.staUseMac = (uint8_t)useMac;
  if (useMac) memcpy(gCfg.staMac, newMac, 6);
  cfgMarkDirty();
  saveConfigForce();

  bool rebootRequired = macChanged || ipChanged || credChanged;

  snprintf(gJsonBuf, sizeof(gJsonBuf),
    "{\"ok\":true,\"rebootRequired\":%s}",
    rebootRequired ? "true" : "false");
  gServer.send(200, "application/json", gJsonBuf);
}

static void handleSys() {
  if (!checkAuth()) return;
  const char* rst = "unknown";
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  rst = "poweron"; break;
    case ESP_RST_EXT:      rst = "external"; break;
    case ESP_RST_SW:       rst = "software"; break;
    case ESP_RST_PANIC:    rst = "panic"; break;
    case ESP_RST_INT_WDT:  rst = "int_wdt"; break;
    case ESP_RST_TASK_WDT: rst = "task_wdt"; break;
    case ESP_RST_WDT:      rst = "wdt"; break;
    case ESP_RST_BROWNOUT: rst = "brownout"; break;
    case ESP_RST_SDIO:     rst = "sdio"; break;
    default: break;
  }
  snprintf(gJsonBuf, sizeof(gJsonBuf),
    "{\"up\":%lu,\"heap\":%u,\"heapMin\":%u,\"rst\":\"%s\"}",
    (unsigned long)(millis() / 1000),
    (unsigned)ESP.getFreeHeap(),
    (unsigned)ESP.getMinFreeHeap(),
    rst);
  gServer.send(200, "application/json", gJsonBuf);
}

static void handleReboot() {
  if (!checkAuth()) return;
  if (gCfgDirty) saveConfigForce();
  sendJsonOk();
  gRebootAtMs = millis() + REBOOT_DELAY_MS;
  if (gRebootAtMs == 0) gRebootAtMs = 1;
}

static void handleFactory() {
  if (!checkAuth()) return;
  sendJsonOk();
  gFactoryAtMs = millis() + FACTORY_DELAY_MS;
  if (gFactoryAtMs == 0) gFactoryAtMs = 1;
}

static void handleAuth() {
  if (!checkAuth()) return;
  String body = gServer.arg("plain");
  String pass;
  if (!jsonGetStr(body, "pass", pass) || pass.length() < 8 || pass.length() > 32) {
    sendJsonErr("bad pass"); return;
  }

  char oldPass[33];
  strncpy(oldPass, gAuthPass, sizeof(oldPass) - 1);
  oldPass[sizeof(oldPass) - 1] = 0;

  char newPass[33];
  strncpy(newPass, pass.c_str(), sizeof(newPass) - 1);
  newPass[sizeof(newPass) - 1] = 0;

  gPrefs.begin(CFG_NS, false);
  size_t written = gPrefs.putString(CFG_KEY_AUTH, newPass);
  gPrefs.end();

  if (written != strlen(newPass)) {
    strncpy(gAuthPass, oldPass, sizeof(gAuthPass) - 1);
    gAuthPass[sizeof(gAuthPass) - 1] = 0;
    sendJsonErr("nvs write failed");
    return;
  }

  strncpy(gAuthPass, newPass, sizeof(gAuthPass) - 1);
  gAuthPass[sizeof(gAuthPass) - 1] = 0;
  sendJsonOk();
}

static void handleCaptive() {
  IPAddress ap = WiFi.softAPIP();
  if (ap == IPAddress(0, 0, 0, 0)) ap = IPAddress(192, 168, 4, 1);
  gServer.sendHeader("Location", String("http://") + ap.toString() + "/", true);
  gServer.send(302, "text/plain", "");
}

static void handleNotFound() {
  IPAddress apAddr = WiFi.softAPIP();
  if (apAddr == IPAddress(0, 0, 0, 0)) apAddr = IPAddress(192, 168, 4, 1);

  String host  = gServer.hostHeader();
  String apIp  = apAddr.toString();
  String staIp = (WiFi.status() == WL_CONNECTED)
                   ? WiFi.localIP().toString() : "";

  bool hostIsAp  = (host == apIp);
  bool hostIsSta = (staIp.length() && host == staIp);

  if (hostIsAp || hostIsSta) {
    if (gServer.uri().startsWith("/api/")) {
      sendJsonErr("not found");
      return;
    }
    sendHtml();
    return;
  }
  handleCaptive();
}

void setup() {
  pinMode(PIN_BOOT_BTN, INPUT_PULLUP);
  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);

  loadConfig();
  gCfgLastWriteMs = millis();

  pinMode(PIN_RELAY, OUTPUT);
  gRelayState = false;
  relayApplyPin();

  WiFi.persistent(false);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);

  WiFi.softAPsetHostname("esp32-ap");
  if (gCfg.apAuth && strlen(gCfg.apPass) >= 8) {
    WiFi.softAP(gCfg.apSsid, gCfg.apPass, gCfg.apChannel,
                gCfg.apHidden, gCfg.apMaxConn);
  } else {
    WiFi.softAP(gCfg.apSsid, NULL, gCfg.apChannel,
                gCfg.apHidden, gCfg.apMaxConn);
  }
  applyApConfig();

  staBegin();

  gDns.setErrorReplyCode(DNSReplyCode::NoError);
  gDns.start(53, "*", WiFi.softAPIP());
  gDnsStarted = true;

  bmeInit();
  gBmeLastRead = millis();

  gPrefs.begin(CFG_NS, false);
  String ap = gPrefs.getString(CFG_KEY_AUTH, "");
  if (ap.length() >= 8) {
    strncpy(gAuthPass, ap.c_str(), sizeof(gAuthPass) - 1);
    gAuthPass[32] = 0;
  } else {
    gPrefs.putString(CFG_KEY_AUTH, gAuthPass);
  }

  gServer.on("/", HTTP_GET, sendHtml);
  gServer.on("/api/status", HTTP_GET, handleStatus);
  gServer.on("/api/thresholds", HTTP_GET, handleGetThr);
  gServer.on("/api/thresholds", HTTP_POST, handlePostThr);
  gServer.on("/api/relay", HTTP_POST, handlePostRelay);
  gServer.on("/api/ap", HTTP_GET, handleGetAp);
  gServer.on("/api/ap", HTTP_POST, handlePostAp);
  gServer.on("/api/sta", HTTP_GET, handleGetSta);
  gServer.on("/api/sta", HTTP_POST, handlePostSta);
  gServer.on("/api/sys", HTTP_GET, handleSys);
  gServer.on("/api/reboot", HTTP_POST, handleReboot);
  gServer.on("/api/factory", HTTP_POST, handleFactory);
  gServer.on("/api/auth", HTTP_POST, handleAuth);

  static const char* const kProbePaths[] = {
    "/generate_204",
    "/gen_204",
    "/hotspot-detect.html",
    "/connecttest.txt",
    "/redirect",
    "/canonical.html",
    "/success.txt",
    "/ncsi.txt",
    "/library/test/success.html",
    "/manifest.json",
    "/connectivity-check.html",
    "/captiveportal",
    "/portal",
  };
  for (size_t i = 0; i < sizeof(kProbePaths) / sizeof(kProbePaths[0]); i++) {
    gServer.on(kProbePaths[i], HTTP_ANY, handleCaptive);
  }

  gServer.onNotFound(handleNotFound);
  gServer.begin();
}

void loop() {
  uint32_t now = millis();

  buttonTick(now);
  cfgTick(now);
  bmeTick(now);
  staTick(now);
  relayTick();

  gServer.handleClient();
  if (gDnsStarted) gDns.processNextRequest();

  if (gRebootAtMs && (int32_t)(now - gRebootAtMs) >= 0) {
    ESP.restart();
  }
  if (gFactoryAtMs && (int32_t)(now - gFactoryAtMs) >= 0) {
    factoryReset();
  }

  static uint32_t ledLast = 0;
  if ((uint32_t)(now - ledLast) >= 1000) {
    ledLast = now;
    static bool ledState = false;
    ledState = !ledState;
    digitalWrite(PIN_LED, ledState ? HIGH : LOW);
  }
}
