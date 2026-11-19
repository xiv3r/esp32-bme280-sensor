#include <Arduino.h>
#include <ESP8266WiFi.h>
#include <ESPAsyncTCP.h>
#include <ESPAsyncWebServer.h>
#include <DNSServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <LittleFS.h>
#include <stddef.h>
#include <stdint.h>
#include <math.h>
#include <stdlib.h>
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

static const uint8_t PIN_BME_SDA = 5;       // D1 / GPIO5
static const uint8_t PIN_BME_SCL = 4;       // D2 / GPIO4
static const uint8_t PIN_RELAY_TEMP = 14;   // D5 / GPIO14
static const uint8_t PIN_RELAY_HUMID = 12;  // D6 / GPIO12
static const uint8_t PIN_RELAY_PRESS = 13;  // D7 / GPIO13
static const uint8_t PIN_FACTORY_BUTTON = 0; // D3 / GPIO0

static const uint16_t DNS_PORT = 53;
static const uint8_t AP_MAX_CLIENTS = 4;
static const uint32_t SENSOR_RETRY_MS = 5000UL;
static const uint32_t BUTTON_DEBOUNCE_MS = 35UL;
static const uint32_t FACTORY_HOLD_MS = 5000UL;
static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_GATEWAY(192, 168, 4, 1);
static const IPAddress AP_SUBNET(255, 255, 255, 0);

static const uint32_t CONFIG_MAGIC = 0x42323830UL;
static const uint16_t CONFIG_VERSION = 1;
static const char DEFAULT_AP_SSID[] = "ESP8266_BME280";
static const char DEFAULT_AP_PASSWORD[] = "12345678";

enum SensorIndex : uint8_t {
  SENSOR_TEMPERATURE = 0,
  SENSOR_HUMIDITY = 1,
  SENSOR_PRESSURE = 2,
  SENSOR_COUNT = 3
};

enum RelayMode : uint8_t {
  RELAY_MODE_UNIFIED = 0,
  RELAY_MODE_INDIVIDUAL = 1
};

enum SaveState : uint8_t {
  SAVE_STATE_DEFAULTS = 0,
  SAVE_STATE_PENDING = 1,
  SAVE_STATE_SAVED = 2,
  SAVE_STATE_ERROR = 3
};

struct MetricConfig {
  uint8_t enabled;
  uint8_t reserved[7];
  int64_t low;
  int64_t high;
  int64_t hysteresis;
};

struct DeviceConfig {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint64_t sequence;
  char apSsid[33];
  char apPassword[64];
  uint8_t apChannel;
  uint8_t relayMode;
  uint8_t relayActiveLow;
  uint8_t sensorFaultOn;
  uint32_t sampleIntervalMs;
  uint32_t reserved;
  MetricConfig metrics[SENSOR_COUNT];
  uint32_t crc;
};

class FixedJson {
 public:
  FixedJson(char *buffer, size_t capacity) : data_(buffer), capacity_(capacity), length_(0), good_(true) {
    if (capacity_ > 0) data_[0] = '\0';
  }

  void raw(const char *text) {
    if (!good_ || text == NULL) return;
    const size_t n = strlen(text);
    if (n >= capacity_ - length_) {
      good_ = false;
      return;
    }
    memcpy(data_ + length_, text, n);
    length_ += n;
    data_[length_] = '\0';
  }

  void character(char c) {
    char value[2] = {c, '\0'};
    raw(value);
  }

  void format(const char *formatString, ...) {
    if (!good_ || length_ >= capacity_) return;
    va_list args;
    va_start(args, formatString);
    const int count = vsnprintf(data_ + length_, capacity_ - length_, formatString, args);
    va_end(args);
    if (count < 0 || static_cast<size_t>(count) >= capacity_ - length_) {
      good_ = false;
      data_[capacity_ - 1] = '\0';
      return;
    }
    length_ += static_cast<size_t>(count);
  }

  void quoted(const char *text) {
    raw("\"");
    if (text != NULL) {
      const unsigned char *p = reinterpret_cast<const unsigned char *>(text);
      while (*p != 0 && good_) {
        switch (*p) {
          case '"': raw("\\\""); break;
          case '\\': raw("\\\\"); break;
          case '\b': raw("\\b"); break;
          case '\f': raw("\\f"); break;
          case '\n': raw("\\n"); break;
          case '\r': raw("\\r"); break;
          case '\t': raw("\\t"); break;
          default:
            if (*p < 0x20) format("\\u%04X", static_cast<unsigned int>(*p));
            else character(static_cast<char>(*p));
            break;
        }
        ++p;
      }
    }
    raw("\"");
  }

  bool good() const { return good_; }
  const char *c_str() const { return data_; }

 private:
  char *data_;
  size_t capacity_;
  size_t length_;
  bool good_;
};

static AsyncWebServer webServer(80);
static DNSServer dnsServer;
static Adafruit_BME280 bme;
static DeviceConfig config;
static DeviceConfig queuedConfig;

static bool filesystemReady = false;
static bool recoveredDefaults = false;
static bool bmeReady = false;
static bool sensorValid = false;
static bool hasGoodSample = false;
static bool apReady = false;
static bool dnsReady = false;
static bool relayPinsInitialized = false;
static volatile bool configBusy = false;
static volatile bool configSavePending = false;
static volatile bool rescanRequested = false;
static bool queuedApRestart = false;
static bool queuedFactoryReset = false;
static volatile bool factoryResetRequested = false;
static SaveState saveState = SAVE_STATE_DEFAULTS;

static uint8_t bmeAddress = 0;
static uint8_t apRestartPhase = 0;
static uint8_t apClientCount = 0;
static uint32_t apRestartDeadline = 0;
static uint32_t nextSensorReadAt = 0;
static uint32_t nextBmeProbeAt = 0;
static uint32_t nextClientCountAt = 0;
static uint32_t lastClockMillis = 0;
static uint32_t resetRawChangedAt = 0;
static uint32_t resetPressStartedAt = 0;
static bool resetRawPressed = false;
static bool resetStablePressed = false;
static bool resetTriggered = false;

static uint64_t uptimeMs = 0;
static uint64_t lastGoodSampleUptimeMs = 0;
static int64_t temperatureCenti = 0;
static int64_t humidityCenti = 0;
static int64_t pressurePa = 0;
static int8_t alarmSide[SENSOR_COUNT] = {0, 0, 0};
static bool relayLogicalState[SENSOR_COUNT] = {false, false, false};
static uint8_t relayPhysicalLevel[SENSOR_COUNT] = {HIGH, HIGH, HIGH};
static const uint8_t relayPins[SENSOR_COUNT] = {PIN_RELAY_TEMP, PIN_RELAY_HUMID, PIN_RELAY_PRESS};

static const char PAGE_HTML[] PROGMEM = R"APPHTML(<!doctype html>
<html lang="en">
<head>
  <meta charset="utf-8">
  <meta name="viewport" content="width=device-width, initial-scale=1, viewport-fit=cover">
  <meta name="theme-color" content="#f3f6fb">
  <title>Atmos — BME280</title>
  <link rel="stylesheet" href="/app.css">
</head>
<body>
  <header class="topbar">
    <a class="brand" href="/" aria-label="Atmos BME280 dashboard">
      <span class="brand-mark" aria-hidden="true"><svg viewBox="0 0 32 32"><path d="M16 3.5C12.8 8.4 7.1 14 7.1 20.1a8.9 8.9 0 0 0 17.8 0C24.9 14 19.2 8.4 16 3.5Z"/><path d="M11.1 20.4a5 5 0 0 0 4.9 5"/></svg></span>
      <span class="brand-copy"><strong>Atmos</strong><small>ENVIRONMENT CONTROL</small></span>
    </a>
    <div class="topbar-right">
      <span id="connectionBadge" class="connection-badge pending"><span class="status-dot"></span><span>Connecting</span></span>
      <span class="device-label">ESP8266 · BME280</span>
    </div>
  </header>
  <div class="layout">
    <nav class="sidebar" aria-label="Primary navigation">
      <p class="nav-heading">WORKSPACE</p>
      <a class="nav-link" data-nav="/" href="/"><span class="nav-icon" aria-hidden="true">◫</span><span>Overview</span></a>
      <a class="nav-link" data-nav="/sensors" href="/sensors"><span class="nav-icon" aria-hidden="true">⌁</span><span>Sensor &amp; relay</span></a>
      <a class="nav-link" data-nav="/wifi" href="/wifi"><span class="nav-icon" aria-hidden="true">⌁</span><span>Wi-Fi access point</span></a>
      <div class="sidebar-spacer"></div>
      <div class="sidebar-note"><span class="mini-led"></span><span>Local device<br><strong>192.168.4.1</strong></span></div>
    </nav>
    <main id="app" class="main">
      <div class="loading-card"><span class="spinner" aria-hidden="true"></span><span>Loading device…</span></div>
    </main>
  </div>
  <footer class="footer"><span>Atmos BME280 controller</span><span>Local-only · No cloud connection</span></footer>
  <script defer src="/app.js"></script>
</body>
</html>)APPHTML";

static const char PAGE_CSS[] PROGMEM = R"APPCSS(:root{color-scheme:light;--ink:#172638;--muted:#6b7b8d;--line:#e4eaf1;--paper:#f3f6fb;--card:#fff;--blue:#2066df;--blue-dark:#174db0;--teal:#0e9c8c;--green:#18845d;--red:#b84242;--amber:#a96913;--shadow:0 10px 32px rgba(27,49,76,.06);font-family:Inter,ui-sans-serif,system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}*{box-sizing:border-box}body{margin:0;background:var(--paper);color:var(--ink);min-height:100vh}.topbar{height:72px;position:sticky;top:0;z-index:5;display:flex;align-items:center;justify-content:space-between;padding:0 clamp(18px,4vw,48px);background:rgba(255,255,255,.94);border-bottom:1px solid var(--line);backdrop-filter:blur(12px)}.brand{display:flex;align-items:center;gap:12px;color:inherit;text-decoration:none}.brand-mark{width:38px;height:38px;border-radius:13px;background:#e9f4f2;display:grid;place-items:center}.brand-mark svg{width:25px;height:25px;fill:none;stroke:var(--teal);stroke-width:2;stroke-linecap:round;stroke-linejoin:round}.brand-copy{display:flex;flex-direction:column;gap:2px}.brand-copy strong{font-size:16px;letter-spacing:.01em}.brand-copy small{font-size:9px;letter-spacing:.14em;color:var(--muted);font-weight:750}.topbar-right{display:flex;align-items:center;gap:17px}.connection-badge{display:inline-flex;align-items:center;gap:8px;padding:7px 11px;border-radius:999px;font-size:12px;font-weight:700;background:#f1f4f8;color:var(--muted)}.connection-badge.good{background:#e8f6ef;color:var(--green)}.connection-badge.warn{background:#fff3df;color:var(--amber)}.status-dot,.mini-led{width:8px;height:8px;border-radius:50%;background:#a4afbd}.good .status-dot,.mini-led{background:#1cae7a;box-shadow:0 0 0 3px rgba(28,174,122,.12)}.warn .status-dot{background:#d38a20;box-shadow:0 0 0 3px rgba(211,138,32,.13)}.device-label{font-size:12px;color:var(--muted)}.layout{max-width:1440px;margin:0 auto;min-height:calc(100vh - 120px);display:grid;grid-template-columns:228px minmax(0,1fr);gap:28px;padding:28px clamp(18px,4vw,48px) 42px}.sidebar{display:flex;flex-direction:column;gap:8px;padding:8px 0;position:sticky;top:98px;height:calc(100vh - 130px);min-height:360px}.nav-heading{font-size:10px;font-weight:800;letter-spacing:.14em;color:#93a0ae;margin:6px 12px 12px}.nav-link{display:flex;align-items:center;gap:12px;padding:12px 13px;text-decoration:none;color:#607183;font-size:13px;font-weight:650;border-radius:11px;transition:background .16s,color .16s}.nav-link:hover{background:#e9eef5;color:var(--ink)}.nav-link.active{background:#e6efff;color:var(--blue)}.nav-icon{width:20px;text-align:center;font-size:20px;line-height:1}.sidebar-spacer{flex:1}.sidebar-note{display:flex;align-items:center;gap:11px;padding:14px 12px;border-top:1px solid var(--line);font-size:11px;line-height:1.6;color:var(--muted)}.sidebar-note strong{color:var(--ink);font-size:12px}.main{min-width:0}.footer{display:flex;justify-content:space-between;max-width:1440px;margin:0 auto;padding:0 clamp(18px,4vw,48px) 22px;color:#93a0ae;font-size:11px}.page-head{display:flex;align-items:flex-end;justify-content:space-between;gap:20px;margin:3px 0 23px}.eyebrow{font-size:10px;letter-spacing:.16em;color:var(--blue);font-weight:800;margin:0 0 8px}.page-head h1{font-size:clamp(25px,3vw,34px);line-height:1.14;margin:0;letter-spacing:-.04em}.page-subtitle{font-size:13px;color:var(--muted);margin:8px 0 0;line-height:1.55}.head-actions{display:flex;align-items:center;gap:10px}.grid{display:grid;gap:16px}.metric-grid{grid-template-columns:repeat(3,minmax(0,1fr))}.two-grid{grid-template-columns:repeat(2,minmax(0,1fr));margin-top:16px}.three-grid{grid-template-columns:repeat(3,minmax(0,1fr))}.card{background:var(--card);border:1px solid var(--line);border-radius:16px;padding:20px;box-shadow:var(--shadow);min-width:0}.metric-card{position:relative;overflow:hidden;min-height:208px}.metric-top{display:flex;align-items:center;justify-content:space-between;color:#6a7a8c;font-size:12px;font-weight:700}.metric-icon{width:31px;height:31px;border-radius:10px;background:#f0f5ff;color:var(--blue);display:grid;place-items:center;font-size:16px}.metric-card:nth-child(2) .metric-icon{background:#e9f7f3;color:var(--teal)}.metric-card:nth-child(3) .metric-icon{background:#fff4e7;color:#be7b22}.metric-value{display:flex;align-items:baseline;gap:8px;margin-top:17px;font-size:clamp(31px,4vw,43px);font-weight:760;letter-spacing:-.055em;line-height:1}.metric-value span{font-size:15px;letter-spacing:0;color:#738397;font-weight:600}.metric-meta{display:flex;align-items:center;justify-content:space-between;gap:8px;margin-top:14px;font-size:11px;color:var(--muted)}.tag{display:inline-flex;align-items:center;gap:5px;padding:5px 8px;border-radius:999px;background:#f1f4f7;color:#758396;font-weight:750;font-size:10px;white-space:nowrap}.tag.active{background:#fff0eb;color:#b64d36}.tag.ok{background:#eaf6ef;color:var(--green)}.sparkline{width:100%;height:38px;margin-top:8px;overflow:visible}.sparkline path{fill:none;stroke:var(--blue);stroke-width:2.2;stroke-linecap:round;stroke-linejoin:round}.metric-card:nth-child(2) .sparkline path{stroke:var(--teal)}.metric-card:nth-child(3) .sparkline path{stroke:#be7b22}.notice{padding:13px 15px;border-radius:12px;display:flex;align-items:flex-start;gap:10px;font-size:12px;line-height:1.5;border:1px solid transparent}.notice.success{background:#ebf7f0;border-color:#d4edde;color:#257354}.notice.warning{background:#fff5e6;border-color:#f5e6c7;color:#8a5b13}.notice.error{background:#fff0ef;border-color:#f2d5d1;color:#a83c36}.notice.info{background:#edf4ff;border-color:#dce8fb;color:#335d9e}.notice strong{font-weight:800}.notice + .grid{margin-top:16px}.section-head{display:flex;align-items:flex-start;justify-content:space-between;gap:14px;margin-bottom:16px}.section-head h2,.card h2{font-size:15px;margin:0;letter-spacing:-.02em}.section-head p,.card-description{font-size:12px;line-height:1.5;color:var(--muted);margin:6px 0 0}.section-label{font-size:10px;letter-spacing:.12em;color:#8b99a8;font-weight:800;text-transform:uppercase;margin:0 0 9px}.detail-list{display:grid;gap:0}.detail-row{display:flex;justify-content:space-between;gap:14px;padding:11px 0;border-bottom:1px solid #edf1f5;font-size:12px}.detail-row:last-child{border-bottom:0;padding-bottom:0}.detail-row span:first-child{color:var(--muted)}.detail-row strong{font-size:12px;text-align:right;overflow-wrap:anywhere}.relay-list{display:grid;gap:10px}.relay-row{display:flex;align-items:center;justify-content:space-between;padding:12px 13px;background:#f7f9fc;border:1px solid #edf1f5;border-radius:12px}.relay-label{display:flex;flex-direction:column;gap:4px}.relay-label strong{font-size:12px}.relay-label span{font-size:10px;color:var(--muted)}.relay-state{display:inline-flex;align-items:center;gap:6px;font-size:10px;font-weight:800;color:var(--muted)}.relay-state::before{content:"";width:7px;height:7px;background:#aab4c1;border-radius:50%}.relay-state.on{color:var(--green)}.relay-state.on::before{background:#1cae7a;box-shadow:0 0 0 3px rgba(28,174,122,.12)}.small-note{font-size:11px;color:var(--muted);line-height:1.55;margin:13px 0 0}.button{appearance:none;border:1px solid transparent;border-radius:10px;min-height:40px;padding:0 15px;display:inline-flex;align-items:center;justify-content:center;gap:8px;background:var(--blue);color:#fff;font:inherit;font-size:12px;font-weight:750;text-decoration:none;cursor:pointer;transition:background .16s,transform .16s,box-shadow .16s}.button:hover{background:var(--blue-dark);box-shadow:0 5px 15px rgba(32,102,223,.18)}.button:active{transform:translateY(1px)}.button:disabled{opacity:.55;cursor:wait;box-shadow:none}.button.secondary{background:#fff;border-color:var(--line);color:#40536a}.button.secondary:hover{background:#f5f8fc}.button.ghost{background:transparent;color:var(--blue);padding:0 8px;min-height:34px}.button.ghost:hover{background:#edf4ff;box-shadow:none}.form-layout{display:grid;gap:16px}.form-card{padding:22px}.form-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:15px}.field{display:flex;flex-direction:column;gap:7px;min-width:0}.field label,.field-label{font-size:11px;font-weight:750;color:#43566c}.field input,.field select{width:100%;height:42px;padding:0 11px;border:1px solid #dbe3ec;border-radius:9px;background:#fff;color:var(--ink);font:inherit;font-size:13px;outline:none}.field input:focus,.field select:focus{border-color:#78a5f4;box-shadow:0 0 0 3px rgba(32,102,223,.11)}.field input:disabled{background:#f4f6f9;color:#8d99a8}.field small,.field-hint{font-size:10px;line-height:1.45;color:#8593a2}.form-section{padding-top:18px;margin-top:18px;border-top:1px solid var(--line)}.form-section:first-child{padding-top:0;margin-top:0;border:0}.form-section-title{display:flex;justify-content:space-between;align-items:flex-start;gap:15px;margin-bottom:14px}.form-section-title h2{font-size:14px;margin:0}.form-section-title p{font-size:11px;color:var(--muted);margin:5px 0 0;line-height:1.5}.switch-row{display:flex;align-items:center;justify-content:space-between;gap:12px}.switch{position:relative;display:inline-flex;align-items:center;gap:9px;cursor:pointer;font-size:11px;color:#506176;font-weight:700}.switch input{position:absolute;opacity:0;width:1px;height:1px}.switch-track{width:36px;height:21px;background:#c5ced8;border-radius:999px;transition:background .15s;position:relative}.switch-track::after{content:"";position:absolute;top:3px;left:3px;width:15px;height:15px;border-radius:50%;background:white;box-shadow:0 1px 4px rgba(0,0,0,.18);transition:transform .15s}.switch input:checked + .switch-track{background:var(--blue)}.switch input:checked + .switch-track::after{transform:translateX(15px)}.switch input:focus-visible + .switch-track{outline:3px solid rgba(32,102,223,.2)}.threshold-grid{display:grid;grid-template-columns:repeat(3,minmax(0,1fr));gap:12px}.sensor-block{border:1px solid var(--line);border-radius:13px;padding:16px;background:#fff}.sensor-block-head{display:flex;align-items:center;justify-content:space-between;gap:10px;margin-bottom:13px}.sensor-name{display:flex;align-items:center;gap:9px}.sensor-name .metric-icon{width:29px;height:29px}.sensor-name strong{font-size:12px}.sensor-unit{font-size:10px;color:var(--muted);margin-top:2px}.sensor-block.disabled{background:#fafbfd}.sensor-block.disabled .threshold-grid{opacity:.48}.savebar{display:flex;align-items:center;justify-content:space-between;gap:12px;padding-top:15px;margin-top:17px;border-top:1px solid var(--line)}.form-message{min-height:16px;font-size:11px;color:var(--muted);line-height:1.4}.form-message.success{color:var(--green)}.form-message.error{color:var(--red)}.wifi-banner{display:flex;align-items:center;gap:13px;padding:16px 18px;border:1px solid #dce8fb;background:#edf4ff;border-radius:14px;margin-bottom:16px}.wifi-symbol{width:37px;height:37px;border-radius:11px;background:#fff;display:grid;place-items:center;color:var(--blue);font-size:18px}.wifi-banner strong{font-size:12px}.wifi-banner p{margin:4px 0 0;font-size:11px;color:#5f7391}.password-wrap{position:relative}.password-wrap input{padding-right:68px}.password-toggle{position:absolute;right:7px;top:6px;border:0;background:transparent;color:var(--blue);font-size:10px;font-weight:750;padding:7px;cursor:pointer}.loading-card{display:flex;align-items:center;gap:12px;background:#fff;border:1px solid var(--line);border-radius:14px;padding:20px;color:var(--muted);font-size:13px}.spinner{width:16px;height:16px;border-radius:50%;border:2px solid #dce5f0;border-top-color:var(--blue);animation:spin .8s linear infinite}@keyframes spin{to{transform:rotate(360deg)}}.empty-state{padding:26px;border:1px dashed #cbd6e2;border-radius:14px;background:#fff;color:var(--muted);font-size:12px;text-align:center}.mono{font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace}.sr-only{position:absolute;width:1px;height:1px;padding:0;margin:-1px;overflow:hidden;clip:rect(0,0,0,0);white-space:nowrap;border:0}
@media(max-width:980px){.layout{grid-template-columns:190px minmax(0,1fr);gap:20px}.metric-grid{gap:12px}.metric-card{padding:16px}.threshold-grid{grid-template-columns:1fr}.sensor-block{padding:14px}.three-grid{grid-template-columns:1fr}.device-label{display:none}}
@media(max-width:700px){.topbar{height:64px;padding:0 16px}.topbar-right{gap:8px}.connection-badge{padding:7px 9px;font-size:11px}.layout{display:block;padding:16px 14px 30px}.sidebar{height:auto;min-height:0;position:sticky;top:64px;z-index:4;margin:0 -14px 17px;padding:8px 14px;background:rgba(243,246,251,.96);border-bottom:1px solid var(--line);flex-direction:row;overflow-x:auto;gap:6px}.nav-heading,.sidebar-spacer,.sidebar-note{display:none}.nav-link{white-space:nowrap;flex:0 0 auto;padding:10px 12px;font-size:11px}.nav-icon{font-size:17px}.footer{padding:0 14px 17px;font-size:10px;gap:12px}.page-head{align-items:flex-start;flex-direction:column;margin:7px 0 17px;gap:13px}.head-actions{width:100%}.head-actions .button{width:100%}.metric-grid{grid-template-columns:1fr;gap:11px}.metric-card{min-height:174px}.metric-value{font-size:38px;margin-top:13px}.two-grid{grid-template-columns:1fr;gap:12px;margin-top:12px}.card{padding:16px;border-radius:14px}.notice + .grid{margin-top:12px}.form-grid{grid-template-columns:1fr;gap:12px}.form-card{padding:17px}.savebar{align-items:stretch;flex-direction:column-reverse}.savebar .button{width:100%}.page-head h1{font-size:27px}.footer span:last-child{text-align:right}}
@media(prefers-reduced-motion:reduce){*,*::before,*::after{scroll-behavior:auto!important;animation-duration:.01ms!important;animation-iteration-count:1!important;transition-duration:.01ms!important}}.top-gap-8{margin-top:8px}.top-gap-9{margin-top:9px}.top-gap-15{margin-top:15px}.full-span{grid-column:1/-1}
)APPCSS";

static const char PAGE_JS[] PROGMEM = R"APPJS('use strict';
(function(){
  const app = document.getElementById('app');
  const route = location.pathname.replace(/\/+$/, '') || '/';
  const page = route === '/sensors' ? 'sensors' : (route === '/wifi' ? 'wifi' : 'dashboard');
  const badge = document.getElementById('connectionBadge');
  const history = {temperature: [], humidity: [], pressure: []};
  let lastSampleToken = null;
  let statusSnapshot = null;
  let pollBusy = false;

  document.querySelectorAll('[data-nav]').forEach(function(link){
    if (link.getAttribute('data-nav') === route) link.classList.add('active');
  });

  function escapeHtml(value){
    return String(value).replace(/[&<>"']/g, function(ch){
      return {'&':'&amp;','<':'&lt;','>':'&gt;','"':'&quot;',"'":'&#39;'}[ch];
    });
  }
  function sleep(ms){ return new Promise(function(resolve){ setTimeout(resolve, ms); }); }
  async function requestJson(url, options){
    const controller = new AbortController();
    const timer = setTimeout(function(){ controller.abort(); }, 5000);
    try{
      const response = await fetch(url, Object.assign({cache:'no-store', credentials:'same-origin', signal:controller.signal}, options || {}));
      let data = {};
      try { data = await response.json(); } catch (_) { data = {}; }
      if (!response.ok) throw new Error(data.error || ('Request failed (' + response.status + ')'));
      return {response:response, data:data};
    } finally { clearTimeout(timer); }
  }
  function badgeSet(text, kind){
    if (!badge) return;
    badge.className = 'connection-badge ' + kind;
    badge.innerHTML = '<span class="status-dot"></span><span>' + escapeHtml(text) + '</span>';
  }
  function showMessage(element, text, kind){
    if (!element) return;
    element.textContent = text;
    element.className = 'form-message' + (kind ? ' ' + kind : '');
  }
  function fixed(value, digits){
    const number = Number(value);
    return Number.isFinite(number) ? number.toFixed(digits) : '—';
  }
  function duration(ms){
    let seconds = Math.max(0, Math.floor(Number(ms || 0) / 1000));
    const days = Math.floor(seconds / 86400); seconds %= 86400;
    const hours = Math.floor(seconds / 3600); seconds %= 3600;
    const minutes = Math.floor(seconds / 60);
    if (days) return days + 'd ' + hours + 'h';
    if (hours) return hours + 'h ' + minutes + 'm';
    return minutes + 'm ' + (seconds % 60) + 's';
  }
  function rememberSample(s){
    if (!s || !s.valid || !s.hasSample) return;
    const token = Number(s.uptimeMs) - Number(s.sampleAgeMs || 0);
    if (token === lastSampleToken) return;
    lastSampleToken = token;
    history.temperature.push(Number(s.temperatureCenti) / 100);
    history.humidity.push(Number(s.humidityCenti) / 100);
    history.pressure.push(Number(s.pressurePa) / 100);
    Object.keys(history).forEach(function(key){ if (history[key].length > 36) history[key].shift(); });
  }
  function spark(values){
    if (!values || values.length < 2) return '';
    const low = Math.min.apply(null, values), high = Math.max.apply(null, values);
    const spread = high - low || 1;
    return values.map(function(value, index){
      const x = 2 + index * 96 / (values.length - 1);
      const y = 32 - ((value - low) / spread) * 25;
      return (index ? 'L' : 'M') + x.toFixed(1) + ' ' + y.toFixed(1);
    }).join(' ');
  }
  function renderDashboard(s){
    const data = s || null;
    const has = !!(data && data.hasSample);
    const online = !!(data && data.valid);
    const values = has ? [fixed(data.temperatureCenti / 100, 1), fixed(data.humidityCenti / 100, 1), fixed(data.pressurePa / 100, 1)] : ['—','—','—'];
    const units = ['°C','%RH','hPa'];
    const names = ['Temperature','Humidity','Pressure'];
    const symbols = ['↗','◌','⌁'];
    const alarms = data && data.alarms ? data.alarms : [false,false,false];
    const metricKeys = ['temperature','humidity','pressure'];
    const metricCards = names.map(function(name, i){
      const active = !!alarms[i];
      const path = spark(history[metricKeys[i]]);
      return '<article class="card metric-card"><div class="metric-top"><span>' + name + '</span><span class="metric-icon" aria-hidden="true">' + symbols[i] + '</span></div>' +
        '<div class="metric-value">' + values[i] + '<span>' + units[i] + '</span></div>' +
        '<div class="metric-meta"><span>' + (has ? (online ? 'Latest reading' : 'Last good reading') : 'Waiting for a reading') + '</span><span class="tag ' + (active ? 'active' : (online ? 'ok' : '')) + '">' + (active ? 'OUT OF RANGE' : (online ? 'IN RANGE' : 'NO DATA')) + '</span></div>' +
        '<svg class="sparkline" viewBox="0 0 100 36" preserveAspectRatio="none" aria-label="' + name + ' recent trend"><path d="' + path + '"></path></svg></article>';
    }).join('');
    let bannerClass = 'warning', bannerText = 'BME280 is not reporting valid data. Check 3.3 V power, I²C wiring, and the sensor address. Relay fault behavior follows the configured setting.';
    if (online) { bannerClass = 'success'; bannerText = 'BME280 is online and providing fresh readings. Values and relay decisions are updated locally.'; }
    else if (has) { bannerText = 'The latest BME280 sample is invalid or stale. Displayed values are the last good sample; relay behavior follows the configured fault setting.'; }
    let relayRows = '';
    const mode = data && data.relayMode === 1 ? 1 : 0;
    const relayLabels = mode ? [
      ['D5 · GPIO14','Temperature relay'],['D6 · GPIO12','Humidity relay'],['D7 · GPIO13','Pressure relay']
    ] : [
      ['D5 · GPIO14','Unified relay · any enabled alarm'],['D6 · GPIO12','Unused in unified mode'],['D7 · GPIO13','Unused in unified mode']
    ];
    const outputs = data && data.relayOutputs ? data.relayOutputs : [false,false,false];
    relayRows = relayLabels.map(function(item, i){
      const available = mode === 1 || i === 0;
      const on = available && !!outputs[i];
      return '<div class="relay-row"><div class="relay-label"><strong>' + item[0] + '</strong><span>' + item[1] + '</span></div><span class="relay-state ' + (on ? 'on' : '') + '">' + (on ? 'ENERGIZED' : 'OFF') + '</span></div>';
    }).join('');
    const sampleAge = has ? duration(data.sampleAgeMs) + ' ago' : 'No successful sample';
    app.innerHTML = '<div class="page-head"><div><p class="eyebrow">ENVIRONMENT · LIVE</p><h1>Overview</h1><p class="page-subtitle">Live BME280 conditions and relay activity for this device.</p></div><div class="head-actions"><button class="button secondary" id="rescanButton" type="button">↻ &nbsp; Re-scan sensor</button></div></div>' +
      '<div class="notice ' + bannerClass + '" role="status"><strong>' + (online ? 'Sensor online' : 'Sensor needs attention') + '</strong><span>' + bannerText + '</span></div>' +
      '<div class="grid metric-grid">' + metricCards + '</div>' +
      '<div class="grid two-grid"><section class="card"><div class="section-head"><div><p class="section-label">OUTPUTS</p><h2>Relay status</h2><p>Logical relay state; electrical level depends on selected relay polarity.</p></div><span class="tag">' + (mode ? 'INDIVIDUAL' : 'UNIFIED') + '</span></div><div class="relay-list">' + relayRows + '</div><p class="small-note">Sensor alarms use threshold bands with hysteresis. A sensor fault uses the selected fault-output behavior.</p></section>' +
      '<section class="card"><p class="section-label">DEVICE HEALTH</p><h2>Controller status</h2><div class="detail-list top-gap-8">' +
      '<div class="detail-row"><span>BME280</span><strong>' + (data && data.bmeReady ? ('Detected · 0x' + Number(data.bmeAddress).toString(16).toUpperCase()) : 'Not detected') + '</strong></div>' +
      '<div class="detail-row"><span>Last good sample</span><strong>' + sampleAge + '</strong></div>' +
      '<div class="detail-row"><span>Sampling interval</span><strong>' + (data ? (Number(data.sampleIntervalMs) / 1000).toFixed(0) + ' seconds' : '—') + '</strong></div>' +
      '<div class="detail-row"><span>Access point</span><strong>' + (data ? escapeHtml(data.apSsid) : '—') + '</strong></div>' +
      '<div class="detail-row"><span>Local address</span><strong class="mono">192.168.4.1</strong></div>' +
      '<div class="detail-row"><span>Connected clients</span><strong>' + (data ? Number(data.clients) : '—') + ' / 4</strong></div>' +
      '<div class="detail-row"><span>Uptime</span><strong>' + (data ? duration(data.uptimeMs) : '—') + '</strong></div>' +
      '<div class="detail-row"><span>Settings storage</span><strong>' + (data ? (data.filesystemReady ? (data.saveState === 'error' ? 'Write error' : 'LittleFS ready') : 'LittleFS unavailable') : '—') + '</strong></div>' +
      '</div></section></div>';
    const rescan = document.getElementById('rescanButton');
    rescan.addEventListener('click', async function(){
      rescan.disabled = true;
      rescan.textContent = 'Requesting scan…';
      try { await requestJson('/api/rescan', {method:'POST'}); }
      catch (error) { window.alert(error.message || 'Could not request a sensor scan.'); }
      await sleep(500);
      rescan.disabled = false;
      rescan.innerHTML = '↻ &nbsp; Re-scan sensor';
    });
  }
  function sensorSettingsMarkup(){
    return '<div class="page-head"><div><p class="eyebrow">CONTROL · SENSOR RULES</p><h1>Sensor &amp; relay</h1><p class="page-subtitle">Choose which BME280 measurements can trigger relays and set their alarm bands.</p></div></div>' +
      '<form id="sensorForm" class="form-layout" novalidate>' +
      '<section class="card form-card"><div class="form-section"><div class="form-section-title"><div><h2>Relay routing</h2><p>Unified mode combines enabled alarms on D5. Individual mode maps each sensor to its own output.</p></div></div>' +
      '<div class="form-grid"><div class="field"><label for="relayMode">Control mode</label><select id="relayMode"><option value="0">Unified · D5 for any enabled alarm</option><option value="1">Individual · one output per sensor</option></select><small>In unified mode D6 and D7 remain off.</small></div>' +
      '<div class="field"><label for="relayPolarity">Relay input polarity</label><select id="relayPolarity"><option value="1">Active LOW · common relay modules</option><option value="0">Active HIGH</option></select><small>Choose the electrical input level required by your relay board.</small></div>' +
      '<div class="field"><label for="faultBehavior">When BME280 data is unavailable</label><select id="faultBehavior"><option value="0">De-energize configured relays</option><option value="1">Energize configured relays</option></select><small>Not a safety-rated fail-safe; select based on your external circuit.</small></div>' +
      '<div class="field"><label for="sampleSeconds">Sensor sampling interval</label><input id="sampleSeconds" type="number" min="1" max="60" step="1" required><small>1–60 seconds. Live dashboard polling is independent.</small></div></div></div>' +
      '<div class="form-section"><div class="form-section-title"><div><h2>Alarm bands</h2><p>An enabled sensor alarms at or outside its low/high trip points. Hysteresis is the return margin used to clear an alarm and reduce relay chatter.</p></div></div>' +
      '<div class="threshold-grid">' +
      '<section class="sensor-block" id="tempBlock"><div class="sensor-block-head"><div class="sensor-name"><span class="metric-icon">↗</span><div><strong>Temperature</strong><div class="sensor-unit">degrees Celsius</div></div></div><label class="switch"><span class="sr-only">Enable temperature relay rule</span><input id="tempEnabled" type="checkbox"><span class="switch-track"></span><span>Enabled</span></label></div><div class="threshold-grid"><div class="field"><label for="tempLow">Low trip · °C</label><input id="tempLow" type="number" min="-40" max="85" step="0.1" required></div><div class="field"><label for="tempHigh">High trip · °C</label><input id="tempHigh" type="number" min="-40" max="85" step="0.1" required></div><div class="field"><label for="tempHyst">Hysteresis · °C</label><input id="tempHyst" type="number" min="0" max="62.5" step="0.1" required></div></div></section>' +
      '<section class="sensor-block" id="humidityBlock"><div class="sensor-block-head"><div class="sensor-name"><span class="metric-icon">◌</span><div><strong>Humidity</strong><div class="sensor-unit">relative humidity · %</div></div></div><label class="switch"><span class="sr-only">Enable humidity relay rule</span><input id="humidityEnabled" type="checkbox"><span class="switch-track"></span><span>Enabled</span></label></div><div class="threshold-grid"><div class="field"><label for="humidityLow">Low trip · %RH</label><input id="humidityLow" type="number" min="0" max="100" step="0.1" required></div><div class="field"><label for="humidityHigh">High trip · %RH</label><input id="humidityHigh" type="number" min="0" max="100" step="0.1" required></div><div class="field"><label for="humidityHyst">Hysteresis · %RH</label><input id="humidityHyst" type="number" min="0" max="50" step="0.1" required></div></div></section>' +
      '<section class="sensor-block" id="pressureBlock"><div class="sensor-block-head"><div class="sensor-name"><span class="metric-icon">⌁</span><div><strong>Pressure</strong><div class="sensor-unit">station pressure · hPa</div></div></div><label class="switch"><span class="sr-only">Enable pressure relay rule</span><input id="pressureEnabled" type="checkbox"><span class="switch-track"></span><span>Enabled</span></label></div><div class="threshold-grid"><div class="field"><label for="pressureLow">Low trip · hPa</label><input id="pressureLow" type="number" min="300" max="1100" step="0.1" required></div><div class="field"><label for="pressureHigh">High trip · hPa</label><input id="pressureHigh" type="number" min="300" max="1100" step="0.1" required></div><div class="field"><label for="pressureHyst">Hysteresis · hPa</label><input id="pressureHyst" type="number" min="0" max="400" step="0.1" required></div></div></section>' +
      '</div><div class="savebar"><span id="sensorMessage" class="form-message" role="status">Loading saved settings…</span><button class="button" id="saveSensors" type="submit">Save sensor settings</button></div></div></section>' +
      '<div class="notice info"><strong>Wiring map</strong><span>Unified: any enabled alarm controls D5 / GPIO14. Individual: temperature → D5 / GPIO14, humidity → D6 / GPIO12, pressure → D7 / GPIO13. Outputs are logical relay states; use a suitable driver board.</span></div></form>';
  }
  function renderSensorPage(){
    app.innerHTML = sensorSettingsMarkup();
    const form = document.getElementById('sensorForm');
    const msg = document.getElementById('sensorMessage');
    function setInput(id, value, digits){ document.getElementById(id).value = Number(value).toFixed(digits); }
    function fillConfig(c){
      document.getElementById('relayMode').value = String(c.relayMode);
      document.getElementById('relayPolarity').value = c.relayActiveLow ? '1' : '0';
      document.getElementById('faultBehavior').value = c.sensorFaultOn ? '1' : '0';
      document.getElementById('sampleSeconds').value = String(Math.round(c.sampleIntervalMs / 1000));
      const m = c.metrics;
      document.getElementById('tempEnabled').checked = !!m[0].enabled;
      setInput('tempLow', m[0].low / 100, 1); setInput('tempHigh', m[0].high / 100, 1); setInput('tempHyst', m[0].hysteresis / 100, 1);
      document.getElementById('humidityEnabled').checked = !!m[1].enabled;
      setInput('humidityLow', m[1].low / 100, 1); setInput('humidityHigh', m[1].high / 100, 1); setInput('humidityHyst', m[1].hysteresis / 100, 1);
      document.getElementById('pressureEnabled').checked = !!m[2].enabled;
      setInput('pressureLow', m[2].low / 100, 1); setInput('pressureHigh', m[2].high / 100, 1); setInput('pressureHyst', m[2].hysteresis / 100, 1);
      updateMetricBlocks();
    }
    function updateMetricBlocks(){
      [['tempEnabled','tempBlock'],['humidityEnabled','humidityBlock'],['pressureEnabled','pressureBlock']].forEach(function(pair){
        const enabled = document.getElementById(pair[0]).checked;
        document.getElementById(pair[1]).classList.toggle('disabled', !enabled);
      });
    }
    ['tempEnabled','humidityEnabled','pressureEnabled'].forEach(function(id){ document.getElementById(id).addEventListener('change', updateMetricBlocks); });
    function scaled(id, multiplier){
      const value = Number(document.getElementById(id).value);
      if (!Number.isFinite(value)) throw new Error('Enter a valid number for every threshold.');
      return String(Math.round(value * multiplier));
    }
    async function waitForCommit(){
      for (let i = 0; i < 20; i++){
        await sleep(300);
        try{
          const result = await requestJson('/api/status');
          if (result.data.saveState === 'saved') return true;
          if (result.data.saveState === 'error') throw new Error('Settings could not be written to LittleFS. Check the filesystem partition and retry.');
        }catch(error){ if (error.message && error.message.indexOf('could not be written') >= 0) throw error; }
      }
      throw new Error('The save is still pending. Keep this page open and check the device status.');
    }
    form.addEventListener('submit', async function(event){
      event.preventDefault();
      if (!form.reportValidity()) return;
      const button = document.getElementById('saveSensors');
      button.disabled = true;
      showMessage(msg, 'Validating and saving settings…', '');
      try{
        const body = new URLSearchParams();
        body.set('mode', document.getElementById('relayMode').value);
        body.set('activeLow', document.getElementById('relayPolarity').value);
        body.set('faultOn', document.getElementById('faultBehavior').value);
        body.set('sampleSec', document.getElementById('sampleSeconds').value);
        body.set('tempEnabled', document.getElementById('tempEnabled').checked ? '1' : '0');
        body.set('tempLow', scaled('tempLow', 100)); body.set('tempHigh', scaled('tempHigh', 100)); body.set('tempHyst', scaled('tempHyst', 100));
        body.set('humidityEnabled', document.getElementById('humidityEnabled').checked ? '1' : '0');
        body.set('humidityLow', scaled('humidityLow', 100)); body.set('humidityHigh', scaled('humidityHigh', 100)); body.set('humidityHyst', scaled('humidityHyst', 100));
        body.set('pressureEnabled', document.getElementById('pressureEnabled').checked ? '1' : '0');
        body.set('pressureLow', scaled('pressureLow', 100)); body.set('pressureHigh', scaled('pressureHigh', 100)); body.set('pressureHyst', scaled('pressureHyst', 100));
        const result = await requestJson('/api/config', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8'}, body:body.toString()});
        if (result.response.status === 202){
          await waitForCommit();
          showMessage(msg, 'Settings saved and applied.', 'success');
        }else showMessage(msg, result.data.changed === false ? 'No changes to save.' : 'Settings are up to date.', 'success');
      }catch(error){ showMessage(msg, error.message || 'Could not save settings.', 'error'); }
      finally{ button.disabled = false; }
    });
    requestJson('/api/config').then(function(result){ fillConfig(result.data); showMessage(msg, 'Loaded from device.', ''); }).catch(function(error){ showMessage(msg, error.message || 'Could not load settings.', 'error'); });
  }
  function renderWifiPage(){
    app.innerHTML = '<div class="page-head"><div><p class="eyebrow">NETWORK · LOCAL ACCESS</p><h1>Wi-Fi access point</h1><p class="page-subtitle">Manage the device network and captive portal entry point.</p></div></div>' +
      '<div class="wifi-banner"><span class="wifi-symbol" aria-hidden="true">⌁</span><div><strong>Device address · 192.168.4.1</strong><p>HTTP requests are redirected to the local dashboard. HTTPS pages cannot be transparently redirected.</p></div></div>' +
      '<div class="grid two-grid"><section class="card form-card"><div class="section-head"><div><p class="section-label">ACCESS POINT</p><h2>Network settings</h2><p>Changes restart the access point; connected devices will need to rejoin.</p></div></div>' +
      '<form id="wifiForm" novalidate><div class="form-grid"><div class="field"><label for="apSsid">Network name (SSID)</label><input id="apSsid" type="text" minlength="1" maxlength="32" required autocomplete="off"><small>1–32 printable ASCII characters.</small></div>' +
      '<div class="field"><label for="apChannel">2.4 GHz channel</label><select id="apChannel"><option value="1">Channel 1</option><option value="2">Channel 2</option><option value="3">Channel 3</option><option value="4">Channel 4</option><option value="5">Channel 5</option><option value="6">Channel 6</option><option value="7">Channel 7</option><option value="8">Channel 8</option><option value="9">Channel 9</option><option value="10">Channel 10</option><option value="11">Channel 11</option><option value="12">Channel 12</option><option value="13">Channel 13</option></select><small>Choose a channel permitted in your region.</small></div>' +
      '<div class="field full-span"><label for="apPassword">New WPA2 password</label><div class="password-wrap"><input id="apPassword" type="password" minlength="8" maxlength="63" autocomplete="new-password" placeholder="Leave blank to keep the current password"><button id="togglePassword" class="password-toggle" type="button" aria-pressed="false">Show</button></div><small>8–63 printable ASCII characters. Password is never returned by the device API.</small></div></div>' +
      '<div class="savebar"><span id="wifiMessage" class="form-message" role="status">Loading access point settings…</span><button class="button" id="saveWifi" type="submit">Save &amp; restart AP</button></div></form></section>' +
      '<section class="card"><p class="section-label">ACCESS &amp; RECOVERY</p><h2>Portal and safety notes</h2><div class="detail-list top-gap-9"><div class="detail-row"><span>Default SSID</span><strong>ESP8266_BME280</strong></div><div class="detail-row"><span>Default password</span><strong>12345678</strong></div><div class="detail-row"><span>Portal address</span><strong class="mono">192.168.4.1</strong></div><div class="detail-row"><span>Maximum clients</span><strong>4</strong></div></div><div class="notice warning top-gap-15"><strong>Change the default password</strong><span>The factory password is shared and is not suitable for an unattended or untrusted installation.</span></div><p class="small-note">Factory reset: while the device is running, hold the onboard D3 / GPIO0 button continuously for 5 seconds. GPIO0 must be HIGH during reset and boot for normal startup; do not hold the button while powering on.</p></section></div>';
    const form = document.getElementById('wifiForm');
    const msg = document.getElementById('wifiMessage');
    requestJson('/api/config').then(function(result){
      document.getElementById('apSsid').value = result.data.apSsid;
      document.getElementById('apChannel').value = String(result.data.apChannel);
      showMessage(msg, 'Loaded from device. Password is not disclosed.', '');
    }).catch(function(error){ showMessage(msg, error.message || 'Could not load access point settings.', 'error'); });
    document.getElementById('togglePassword').addEventListener('click', function(){
      const input = document.getElementById('apPassword');
      const showing = input.type === 'text';
      input.type = showing ? 'password' : 'text';
      this.textContent = showing ? 'Show' : 'Hide';
      this.setAttribute('aria-pressed', showing ? 'false' : 'true');
    });
    form.addEventListener('submit', async function(event){
      event.preventDefault();
      if (!form.reportValidity()) return;
      const button = document.getElementById('saveWifi');
      button.disabled = true;
      showMessage(msg, 'Saving network settings…', '');
      try{
        const body = new URLSearchParams();
        body.set('ssid', document.getElementById('apSsid').value);
        body.set('channel', document.getElementById('apChannel').value);
        body.set('password', document.getElementById('apPassword').value);
        const result = await requestJson('/api/wifi', {method:'POST', headers:{'Content-Type':'application/x-www-form-urlencoded;charset=UTF-8'}, body:body.toString()});
        if (result.response.status === 202){
          let saved = false;
          for (let i = 0; i < 15; i++){
            await sleep(200);
            try{
              const state = await requestJson('/api/status');
              if (state.data.saveState === 'saved'){ saved = true; break; }
              if (state.data.saveState === 'error') throw new Error('Settings could not be written to LittleFS. The access point was not changed.');
            }catch(error){ if (error.message && error.message.indexOf('could not be written') >= 0) throw error; }
          }
          if (!saved) throw new Error('The save is still pending. Keep this page open and check device status.');
          showMessage(msg, 'Saved. The access point is restarting; reconnect to the new network and open 192.168.4.1.', 'success');
          document.getElementById('apPassword').value = '';
        }else{
          showMessage(msg, result.data.changed === false ? 'No changes to save.' : 'Access point settings are up to date.', 'success');
          document.getElementById('apPassword').value = '';
        }
      }catch(error){ showMessage(msg, error.message || 'Could not save access point settings.', 'error'); }
      finally{ button.disabled = false; }
    });
  }
  async function poll(){
    if (pollBusy) return;
    pollBusy = true;
    try{
      const result = await requestJson('/api/status');
      statusSnapshot = result.data;
      rememberSample(statusSnapshot);
      if (statusSnapshot.valid) badgeSet('Sensor live', 'good');
      else if (statusSnapshot.apReady) badgeSet('Sensor attention', 'warn');
      else badgeSet('Device offline', 'pending');
      if (page === 'dashboard') renderDashboard(statusSnapshot);
    }catch(_){
      badgeSet('Reconnecting', 'pending');
    }finally{ pollBusy = false; }
  }
  if (page === 'sensors') renderSensorPage();
  else if (page === 'wifi') renderWifiPage();
  else renderDashboard(null);
  poll();
  setInterval(poll, 2500);
})();
)APPJS";

static size_t boundedLength(const char *text, size_t capacity) {
  size_t length = 0;
  while (length < capacity && text[length] != '\0') ++length;
  return length;
}

static bool printableAscii(const char *text, size_t length) {
  for (size_t i = 0; i < length; ++i) {
    const unsigned char c = static_cast<unsigned char>(text[i]);
    if (c < 0x20 || c > 0x7E) return false;
  }
  return true;
}

static uint32_t crc32Bytes(const uint8_t *data, size_t length) {
  uint32_t crc = 0xFFFFFFFFUL;
  for (size_t i = 0; i < length; ++i) {
    crc ^= data[i];
    for (uint8_t bit = 0; bit < 8; ++bit) {
      crc = (crc & 1U) ? ((crc >> 1) ^ 0xEDB88320UL) : (crc >> 1);
    }
  }
  return ~crc;
}

static void makeDefaultConfig(DeviceConfig &value) {
  memset(&value, 0, sizeof(value));
  value.magic = CONFIG_MAGIC;
  value.version = CONFIG_VERSION;
  value.size = static_cast<uint16_t>(sizeof(DeviceConfig));
  value.sequence = 0;
  memcpy(value.apSsid, DEFAULT_AP_SSID, sizeof(DEFAULT_AP_SSID));
  memcpy(value.apPassword, DEFAULT_AP_PASSWORD, sizeof(DEFAULT_AP_PASSWORD));
  value.apChannel = 6;
  value.relayMode = RELAY_MODE_UNIFIED;
  value.relayActiveLow = 1;
  value.sensorFaultOn = 0;
  value.sampleIntervalMs = 2000UL;

  value.metrics[SENSOR_TEMPERATURE].enabled = 1;
  value.metrics[SENSOR_TEMPERATURE].low = 1800;
  value.metrics[SENSOR_TEMPERATURE].high = 3000;
  value.metrics[SENSOR_TEMPERATURE].hysteresis = 100;

  value.metrics[SENSOR_HUMIDITY].enabled = 1;
  value.metrics[SENSOR_HUMIDITY].low = 3000;
  value.metrics[SENSOR_HUMIDITY].high = 7000;
  value.metrics[SENSOR_HUMIDITY].hysteresis = 300;

  value.metrics[SENSOR_PRESSURE].enabled = 1;
  value.metrics[SENSOR_PRESSURE].low = 98000;
  value.metrics[SENSOR_PRESSURE].high = 103000;
  value.metrics[SENSOR_PRESSURE].hysteresis = 500;
}

static bool validMetric(const MetricConfig &metric, int64_t minimum, int64_t maximum) {
  if (metric.enabled > 1) return false;
  if (metric.low < minimum || metric.high > maximum || metric.low >= metric.high) return false;
  if (metric.hysteresis < 0) return false;
  const uint64_t width = static_cast<uint64_t>(metric.high - metric.low);
  const uint64_t hysteresisTwice = static_cast<uint64_t>(metric.hysteresis) * 2ULL;
  return hysteresisTwice <= width;
}

static bool validSettings(const DeviceConfig &value) {
  const size_t ssidLength = boundedLength(value.apSsid, sizeof(value.apSsid));
  const size_t passwordLength = boundedLength(value.apPassword, sizeof(value.apPassword));
  if (ssidLength == 0 || ssidLength > 32 || !printableAscii(value.apSsid, ssidLength)) return false;
  if (passwordLength < 8 || passwordLength > 63 || !printableAscii(value.apPassword, passwordLength)) return false;
  if (value.apChannel < 1 || value.apChannel > 13) return false;
  if (value.relayMode > RELAY_MODE_INDIVIDUAL || value.relayActiveLow > 1 || value.sensorFaultOn > 1) return false;
  if (value.sampleIntervalMs < 1000UL || value.sampleIntervalMs > 60000UL || (value.sampleIntervalMs % 1000UL) != 0) return false;
  if (!validMetric(value.metrics[SENSOR_TEMPERATURE], -4000, 8500)) return false;
  if (!validMetric(value.metrics[SENSOR_HUMIDITY], 0, 10000)) return false;
  if (!validMetric(value.metrics[SENSOR_PRESSURE], 30000, 110000)) return false;
  return true;
}

static void sealConfig(DeviceConfig &value) {
  value.magic = CONFIG_MAGIC;
  value.version = CONFIG_VERSION;
  value.size = static_cast<uint16_t>(sizeof(DeviceConfig));
  value.crc = crc32Bytes(reinterpret_cast<const uint8_t *>(&value), offsetof(DeviceConfig, crc));
}

static bool validRecord(const DeviceConfig &value) {
  if (value.magic != CONFIG_MAGIC || value.version != CONFIG_VERSION || value.size != sizeof(DeviceConfig)) return false;
  const uint32_t expected = crc32Bytes(reinterpret_cast<const uint8_t *>(&value), offsetof(DeviceConfig, crc));
  return expected == value.crc && validSettings(value);
}

static bool readConfigRecord(const char *path, DeviceConfig &result) {
  if (!filesystemReady || !LittleFS.exists(path)) return false;
  File file = LittleFS.open(path, "r");
  if (!file) return false;
  if (file.size() != sizeof(DeviceConfig)) {
    file.close();
    return false;
  }
  DeviceConfig candidate;
  const size_t count = file.read(reinterpret_cast<uint8_t *>(&candidate), sizeof(candidate));
  file.close();
  if (count != sizeof(candidate) || !validRecord(candidate)) return false;
  result = candidate;
  return true;
}

static bool sequenceIsNewer(uint64_t left, uint64_t right) {
  const uint64_t difference = left - right;
  return difference != 0 && difference < (UINT64_C(1) << 63);
}

static void loadConfig() {
  DeviceConfig slot0;
  DeviceConfig slot1;
  const bool valid0 = readConfigRecord("/settings0.bin", slot0);
  const bool valid1 = readConfigRecord("/settings1.bin", slot1);
  if (valid0 && valid1) config = sequenceIsNewer(slot1.sequence, slot0.sequence) ? slot1 : slot0;
  else if (valid0) config = slot0;
  else if (valid1) config = slot1;
  else {
    makeDefaultConfig(config);
    recoveredDefaults = true;
    saveState = filesystemReady ? SAVE_STATE_DEFAULTS : SAVE_STATE_ERROR;
    return;
  }
  recoveredDefaults = false;
  saveState = SAVE_STATE_SAVED;
}

static bool writeConfigRecord(DeviceConfig &record) {
  if (!filesystemReady) return false;
  sealConfig(record);
  if (!validSettings(record)) return false;

  const bool odd = (record.sequence & 1ULL) != 0;
  const char *target = odd ? "/settings1.bin" : "/settings0.bin";
  const char *temporary = odd ? "/settings1.tmp" : "/settings0.tmp";
  if (LittleFS.exists(temporary) && !LittleFS.remove(temporary)) return false;

  File file = LittleFS.open(temporary, "w");
  if (!file) return false;
  const size_t written = file.write(reinterpret_cast<const uint8_t *>(&record), sizeof(record));
  file.flush();
  file.close();
  if (written != sizeof(record)) {
    if (LittleFS.exists(temporary)) LittleFS.remove(temporary);
    return false;
  }

  DeviceConfig verification;
  if (!readConfigRecord(temporary, verification) || verification.sequence != record.sequence || verification.crc != record.crc) {
    if (LittleFS.exists(temporary)) LittleFS.remove(temporary);
    return false;
  }

  if (LittleFS.exists(target) && !LittleFS.remove(target)) {
    if (LittleFS.exists(temporary)) LittleFS.remove(temporary);
    return false;
  }
  if (!LittleFS.rename(temporary, target)) return false;
  if (!readConfigRecord(target, verification)) return false;
  return verification.sequence == record.sequence && verification.crc == record.crc;
}

static bool sameSettings(const DeviceConfig &left, const DeviceConfig &right) {
  if (strcmp(left.apSsid, right.apSsid) != 0 || strcmp(left.apPassword, right.apPassword) != 0) return false;
  if (left.apChannel != right.apChannel || left.relayMode != right.relayMode || left.relayActiveLow != right.relayActiveLow || left.sensorFaultOn != right.sensorFaultOn || left.sampleIntervalMs != right.sampleIntervalMs) return false;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (left.metrics[i].enabled != right.metrics[i].enabled || left.metrics[i].low != right.metrics[i].low || left.metrics[i].high != right.metrics[i].high || left.metrics[i].hysteresis != right.metrics[i].hysteresis) return false;
  }
  return true;
}

static bool queueConfigSave(const DeviceConfig &candidate, bool restartAp, bool factoryReset) {
  if (!filesystemReady || configBusy || configSavePending) return false;
  queuedConfig = candidate;
  queuedConfig.sequence = config.sequence + 1ULL;
  queuedConfig.crc = 0;
  queuedApRestart = restartAp;
  queuedFactoryReset = factoryReset;
  saveState = SAVE_STATE_PENDING;
  configSavePending = true;
  return true;
}

static const char *saveStateName() {
  switch (saveState) {
    case SAVE_STATE_PENDING: return "pending";
    case SAVE_STATE_SAVED: return "saved";
    case SAVE_STATE_ERROR: return "error";
    default: return "defaults";
  }
}

static bool timeReached(uint32_t now, uint32_t deadline) {
  return static_cast<int32_t>(now - deadline) >= 0;
}

static void clearAlarmStates() {
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) alarmSide[i] = 0;
}

static void scheduleApRestart() {
  apRestartPhase = 1;
  apRestartDeadline = millis() + 1000UL;
}

static bool startAccessPoint() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  if (!WiFi.softAPConfig(AP_IP, AP_GATEWAY, AP_SUBNET)) return false;
  return WiFi.softAP(config.apSsid, config.apPassword, config.apChannel, false, AP_MAX_CLIENTS);
}

static void startCaptiveDns() {
  dnsServer.start(DNS_PORT, "*", AP_IP);
  dnsReady = true;
}

static void serviceApRestart(uint32_t now) {
  if (apRestartPhase == 1 && timeReached(now, apRestartDeadline)) {
    if (dnsReady) {
      dnsServer.stop();
      dnsReady = false;
    }
    WiFi.softAPdisconnect(false);
    apReady = false;
    apRestartPhase = 2;
    apRestartDeadline = now + 250UL;
  } else if (apRestartPhase == 2 && timeReached(now, apRestartDeadline)) {
    apReady = startAccessPoint();
    if (apReady) {
      startCaptiveDns();
      apRestartPhase = 0;
    } else {
      apRestartPhase = 3;
      apRestartDeadline = now + SENSOR_RETRY_MS;
    }
  } else if (apRestartPhase == 3 && timeReached(now, apRestartDeadline)) {
    apReady = startAccessPoint();
    if (apReady) {
      startCaptiveDns();
      apRestartPhase = 0;
    } else {
      apRestartDeadline = now + SENSOR_RETRY_MS;
    }
  }
}

static void initializeRelayPins() {
  const uint8_t offLevel = config.relayActiveLow ? HIGH : LOW;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    digitalWrite(relayPins[i], offLevel);
    pinMode(relayPins[i], OUTPUT);
    digitalWrite(relayPins[i], offLevel);
    relayPhysicalLevel[i] = offLevel;
    relayLogicalState[i] = false;
  }
  relayPinsInitialized = true;
}

static void applyRelayOutputs() {
  bool desired[SENSOR_COUNT] = {false, false, false};
  bool anyEnabled = false;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) anyEnabled = anyEnabled || (config.metrics[i].enabled != 0);

  if (config.relayMode == RELAY_MODE_UNIFIED) {
    bool alarm = false;
    if (sensorValid) {
      for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
        if (config.metrics[i].enabled && alarmSide[i] != 0) alarm = true;
      }
    } else {
      alarm = anyEnabled && config.sensorFaultOn;
    }
    desired[0] = alarm;
  } else {
    for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
      if (!config.metrics[i].enabled) desired[i] = false;
      else if (!sensorValid) desired[i] = config.sensorFaultOn != 0;
      else desired[i] = alarmSide[i] != 0;
    }
  }

  const uint8_t offLevel = config.relayActiveLow ? HIGH : LOW;
  const uint8_t onLevel = config.relayActiveLow ? LOW : HIGH;
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const uint8_t level = desired[i] ? onLevel : offLevel;
    if (!relayPinsInitialized || level != relayPhysicalLevel[i]) digitalWrite(relayPins[i], level);
    relayPhysicalLevel[i] = level;
    relayLogicalState[i] = desired[i];
  }
  relayPinsInitialized = true;
}

static bool startBme280() {
  bmeReady = false;
  bmeAddress = 0;
  if (bme.begin(0x76, &Wire)) bmeAddress = 0x76;
  else if (bme.begin(0x77, &Wire)) bmeAddress = 0x77;
  if (bmeAddress == 0) return false;

  bme.setSampling(Adafruit_BME280::MODE_NORMAL,
                  Adafruit_BME280::SAMPLING_X2,
                  Adafruit_BME280::SAMPLING_X4,
                  Adafruit_BME280::SAMPLING_X4,
                  Adafruit_BME280::FILTER_X4,
                  Adafruit_BME280::STANDBY_MS_500);
  bmeReady = true;
  return true;
}

static int64_t getMetricValue(uint8_t index) {
  if (index == SENSOR_TEMPERATURE) return temperatureCenti;
  if (index == SENSOR_HUMIDITY) return humidityCenti;
  return pressurePa;
}

static void updateAlarmStates() {
  if (!sensorValid) {
    clearAlarmStates();
    return;
  }
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    const MetricConfig &metric = config.metrics[i];
    if (!metric.enabled) {
      alarmSide[i] = 0;
      continue;
    }
    const int64_t value = getMetricValue(i);
    const int64_t clearLow = metric.low + metric.hysteresis;
    const int64_t clearHigh = metric.high - metric.hysteresis;
    if (alarmSide[i] == 0) {
      if (value <= metric.low) alarmSide[i] = -1;
      else if (value >= metric.high) alarmSide[i] = 1;
    } else if (alarmSide[i] < 0) {
      if (value >= metric.high) alarmSide[i] = 1;
      else if (value >= clearLow) alarmSide[i] = 0;
    } else {
      if (value <= metric.low) alarmSide[i] = -1;
      else if (value <= clearHigh) alarmSide[i] = 0;
    }
  }
}

static void sampleBme280(uint32_t now) {
  const float temperature = bme.readTemperature();
  const float humidity = bme.readHumidity();
  const float pressure = bme.readPressure();
  const bool plausible = temperature >= -40.0f && temperature <= 85.0f &&
                         humidity >= 0.0f && humidity <= 100.0f &&
                         pressure >= 30000.0f && pressure <= 110000.0f;
  if (!plausible) {
    sensorValid = false;
    bmeReady = false;
    nextBmeProbeAt = now + SENSOR_RETRY_MS;
    clearAlarmStates();
    return;
  }

  temperatureCenti = static_cast<int64_t>(llroundf(temperature * 100.0f));
  humidityCenti = static_cast<int64_t>(llroundf(humidity * 100.0f));
  pressurePa = static_cast<int64_t>(llroundf(pressure));
  hasGoodSample = true;
  sensorValid = true;
  lastGoodSampleUptimeMs = uptimeMs;
  updateAlarmStates();
}

static void serviceSensor(uint32_t now) {
  if (rescanRequested) {
    rescanRequested = false;
    bmeReady = false;
    sensorValid = false;
    clearAlarmStates();
    nextBmeProbeAt = now;
  }

  if (!bmeReady && timeReached(now, nextBmeProbeAt)) {
    if (startBme280()) nextSensorReadAt = now;
    else nextBmeProbeAt = now + SENSOR_RETRY_MS;
  }
  if (!bmeReady) {
    sensorValid = false;
    clearAlarmStates();
    return;
  }
  if (!timeReached(now, nextSensorReadAt)) return;
  nextSensorReadAt = now + config.sampleIntervalMs;
  sampleBme280(now);
}

static void serviceButton(uint32_t now) {
  const bool rawPressed = digitalRead(PIN_FACTORY_BUTTON) == LOW;
  if (rawPressed != resetRawPressed) {
    resetRawPressed = rawPressed;
    resetRawChangedAt = now;
  }
  if (static_cast<uint32_t>(now - resetRawChangedAt) >= BUTTON_DEBOUNCE_MS && resetStablePressed != resetRawPressed) {
    resetStablePressed = resetRawPressed;
    if (resetStablePressed) resetPressStartedAt = now;
  }
  if (resetStablePressed && !resetTriggered && static_cast<uint32_t>(now - resetPressStartedAt) >= FACTORY_HOLD_MS) {
    resetTriggered = true;
    factoryResetRequested = true;
  }
  if (!resetStablePressed) resetTriggered = false;
}

static void servicePendingSave() {
  if (!configSavePending || configBusy) return;
  configBusy = true;
  DeviceConfig work = queuedConfig;
  const bool restartAp = queuedApRestart;
  const bool factoryReset = queuedFactoryReset;
  configSavePending = false;

  const bool saved = writeConfigRecord(work);
  if (saved) {
    config = work;
    recoveredDefaults = false;
    saveState = SAVE_STATE_SAVED;
    clearAlarmStates();
    nextSensorReadAt = millis();
  } else {
    saveState = SAVE_STATE_ERROR;
    if (factoryReset) {
      config = work;
      clearAlarmStates();
      nextSensorReadAt = millis();
    }
  }
  if (restartAp && (saved || factoryReset)) scheduleApRestart();
  configBusy = false;
}

static void serviceFactoryReset() {
  if (!factoryResetRequested || configSavePending || configBusy) return;
  factoryResetRequested = false;
  DeviceConfig defaults;
  makeDefaultConfig(defaults);
  if (filesystemReady) {
    if (!queueConfigSave(defaults, true, true)) factoryResetRequested = true;
  } else {
    defaults.sequence = config.sequence + 1ULL;
    config = defaults;
    saveState = SAVE_STATE_ERROR;
    recoveredDefaults = true;
    clearAlarmStates();
    nextSensorReadAt = millis();
    scheduleApRestart();
  }
}

static void serviceClientCount(uint32_t now) {
  if (timeReached(now, nextClientCountAt)) {
    apClientCount = apReady ? WiFi.softAPgetStationNum() : 0;
    nextClientCountAt = now + 1000UL;
  }
}

static void sendJson(AsyncWebServerRequest *request, int code, const char *payload) {
  AsyncWebServerResponse *response = request->beginResponse(code, "application/json; charset=utf-8", String(payload));
  response->addHeader("Cache-Control", "no-store, max-age=0");
  response->addHeader("X-Content-Type-Options", "nosniff");
  request->send(response);
}

static void sendStatic(AsyncWebServerRequest *request, const char *contentType, const char *programMemoryContent) {
  AsyncWebServerResponse *response = request->beginResponse_P(200, contentType, programMemoryContent);
  response->addHeader("Cache-Control", "no-store");
  response->addHeader("X-Content-Type-Options", "nosniff");
  response->addHeader("Referrer-Policy", "no-referrer");
  response->addHeader("Content-Security-Policy", "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; img-src 'self' data:; object-src 'none'; base-uri 'none'; frame-ancestors 'none'; form-action 'self'");
  request->send(response);
}

static void handlePage(AsyncWebServerRequest *request) {
  sendStatic(request, "text/html; charset=utf-8", PAGE_HTML);
}

static void writeMetricJson(FixedJson &json, const MetricConfig &metric) {
  json.format("{\"enabled\":%s,\"low\":%lld,\"high\":%lld,\"hysteresis\":%lld}",
              metric.enabled ? "true" : "false",
              static_cast<long long>(metric.low),
              static_cast<long long>(metric.high),
              static_cast<long long>(metric.hysteresis));
}

static void handleConfigGet(AsyncWebServerRequest *request) {
  char output[768];
  FixedJson json(output, sizeof(output));
  json.raw("{\"sequence\":");
  json.format("%llu", static_cast<unsigned long long>(config.sequence));
  json.raw(",\"apSsid\":");
  json.quoted(config.apSsid);
  json.format(",\"apChannel\":%u,\"relayMode\":%u,\"relayActiveLow\":%s,\"sensorFaultOn\":%s,\"sampleIntervalMs\":%lu,\"metrics\":[",
              static_cast<unsigned int>(config.apChannel),
              static_cast<unsigned int>(config.relayMode),
              config.relayActiveLow ? "true" : "false",
              config.sensorFaultOn ? "true" : "false",
              static_cast<unsigned long>(config.sampleIntervalMs));
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i) json.raw(",");
    writeMetricJson(json, config.metrics[i]);
  }
  json.raw("]}");
  if (!json.good()) sendJson(request, 500, "{\"ok\":false,\"error\":\"response_overflow\"}");
  else sendJson(request, 200, json.c_str());
}

static void handleStatusGet(AsyncWebServerRequest *request) {
  char output[1024];
  FixedJson json(output, sizeof(output));
  json.raw("{\"valid\":");
  json.raw(sensorValid ? "true" : "false");
  json.raw(",\"bmeReady\":");
  json.raw(bmeReady ? "true" : "false");
  json.format(",\"bmeAddress\":%u,\"hasSample\":%s,\"temperatureCenti\":",
              static_cast<unsigned int>(bmeAddress), hasGoodSample ? "true" : "false");
  if (hasGoodSample) json.format("%lld", static_cast<long long>(temperatureCenti)); else json.raw("null");
  json.raw(",\"humidityCenti\":");
  if (hasGoodSample) json.format("%lld", static_cast<long long>(humidityCenti)); else json.raw("null");
  json.raw(",\"pressurePa\":");
  if (hasGoodSample) json.format("%lld", static_cast<long long>(pressurePa)); else json.raw("null");
  json.raw(",\"sampleAgeMs\":");
  if (hasGoodSample) json.format("%llu", static_cast<unsigned long long>(uptimeMs - lastGoodSampleUptimeMs)); else json.raw("null");
  json.format(",\"uptimeMs\":%llu,\"sampleIntervalMs\":%lu,\"relayMode\":%u,\"relayActiveLow\":%s,\"sensorFaultOn\":%s,\"alarms\":[",
              static_cast<unsigned long long>(uptimeMs),
              static_cast<unsigned long>(config.sampleIntervalMs),
              static_cast<unsigned int>(config.relayMode),
              config.relayActiveLow ? "true" : "false",
              config.sensorFaultOn ? "true" : "false");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i) json.raw(",");
    json.raw(alarmSide[i] != 0 ? "true" : "false");
  }
  json.raw("],\"relayOutputs\":[");
  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (i) json.raw(",");
    json.raw(relayLogicalState[i] ? "true" : "false");
  }
  json.raw("],\"filesystemReady\":");
  json.raw(filesystemReady ? "true" : "false");
  json.raw(",\"recoveredDefaults\":");
  json.raw(recoveredDefaults ? "true" : "false");
  json.raw(",\"saveState\":");
  json.quoted(saveStateName());
  json.raw(",\"apReady\":");
  json.raw(apReady ? "true" : "false");
  json.raw(",\"apRestarting\":");
  json.raw(apRestartPhase != 0 ? "true" : "false");
  json.raw(",\"apSsid\":");
  json.quoted(config.apSsid);
  json.format(",\"clients\":%u,\"ip\":\"192.168.4.1\"}", static_cast<unsigned int>(apClientCount));
  if (!json.good()) sendJson(request, 500, "{\"ok\":false,\"error\":\"response_overflow\"}");
  else sendJson(request, 200, json.c_str());
}

static bool parseIntegerParameter(AsyncWebServerRequest *request, const char *name, int64_t minimum, int64_t maximum, int64_t &result) {
  if (!request->hasParam(name, true)) return false;
  const AsyncWebParameter *parameter = request->getParam(name, true);
  if (parameter == NULL) return false;
  const String value = parameter->value();
  if (value.length() == 0 || value.length() > 20) return false;
  const char *text = value.c_str();
  const char *cursor = text;
  if (*cursor == '-') ++cursor;
  if (*cursor == '\0') return false;
  for (; *cursor != '\0'; ++cursor) if (*cursor < '0' || *cursor > '9') return false;
  char *end = NULL;
  const long long parsed = strtoll(text, &end, 10);
  if (end == text || *end != '\0' || parsed < minimum || parsed > maximum) return false;
  result = static_cast<int64_t>(parsed);
  return true;
}

static bool parseTextParameter(AsyncWebServerRequest *request, const char *name, char *destination, size_t capacity, bool allowEmpty) {
  if (!request->hasParam(name, true) || capacity == 0) return false;
  const AsyncWebParameter *parameter = request->getParam(name, true);
  if (parameter == NULL) return false;
  const String value = parameter->value();
  const size_t length = value.length();
  if ((!allowEmpty && length == 0) || length >= capacity) return false;
  if (!printableAscii(value.c_str(), length)) return false;
  memset(destination, 0, capacity);
  memcpy(destination, value.c_str(), length);
  destination[length] = '\0';
  return true;
}

static void handleConfigPost(AsyncWebServerRequest *request) {
  if (!filesystemReady) {
    sendJson(request, 503, "{\"ok\":false,\"error\":\"filesystem_unavailable\"}");
    return;
  }
  if (configBusy || configSavePending) {
    sendJson(request, 409, "{\"ok\":false,\"error\":\"save_busy\"}");
    return;
  }

  DeviceConfig candidate = config;
  int64_t value = 0;
  bool valid = true;
  if (parseIntegerParameter(request, "mode", 0, 1, value)) candidate.relayMode = static_cast<uint8_t>(value); else valid = false;
  if (parseIntegerParameter(request, "activeLow", 0, 1, value)) candidate.relayActiveLow = static_cast<uint8_t>(value); else valid = false;
  if (parseIntegerParameter(request, "faultOn", 0, 1, value)) candidate.sensorFaultOn = static_cast<uint8_t>(value); else valid = false;
  if (parseIntegerParameter(request, "sampleSec", 1, 60, value)) candidate.sampleIntervalMs = static_cast<uint32_t>(value * 1000LL); else valid = false;

  static const char *enableNames[SENSOR_COUNT] = {"tempEnabled", "humidityEnabled", "pressureEnabled"};
  static const char *lowNames[SENSOR_COUNT] = {"tempLow", "humidityLow", "pressureLow"};
  static const char *highNames[SENSOR_COUNT] = {"tempHigh", "humidityHigh", "pressureHigh"};
  static const char *hysteresisNames[SENSOR_COUNT] = {"tempHyst", "humidityHyst", "pressureHyst"};
  static const int64_t minimums[SENSOR_COUNT] = {-4000, 0, 30000};
  static const int64_t maximums[SENSOR_COUNT] = {8500, 10000, 110000};

  for (uint8_t i = 0; i < SENSOR_COUNT; ++i) {
    if (parseIntegerParameter(request, enableNames[i], 0, 1, value)) candidate.metrics[i].enabled = static_cast<uint8_t>(value); else valid = false;
    if (parseIntegerParameter(request, lowNames[i], minimums[i], maximums[i], value)) candidate.metrics[i].low = value; else valid = false;
    if (parseIntegerParameter(request, highNames[i], minimums[i], maximums[i], value)) candidate.metrics[i].high = value; else valid = false;
    if (parseIntegerParameter(request, hysteresisNames[i], 0, maximums[i] - minimums[i], value)) candidate.metrics[i].hysteresis = value; else valid = false;
  }

  if (!valid || !validSettings(candidate)) {
    sendJson(request, 400, "{\"ok\":false,\"error\":\"invalid_configuration\"}");
    return;
  }
  if (sameSettings(candidate, config)) {
    sendJson(request, 200, "{\"ok\":true,\"changed\":false}");
    return;
  }
  if (!queueConfigSave(candidate, false, false)) {
    sendJson(request, 409, "{\"ok\":false,\"error\":\"save_busy\"}");
    return;
  }
  sendJson(request, 202, "{\"ok\":true,\"queued\":true}");
}

static void handleWifiPost(AsyncWebServerRequest *request) {
  if (!filesystemReady) {
    sendJson(request, 503, "{\"ok\":false,\"error\":\"filesystem_unavailable\"}");
    return;
  }
  if (configBusy || configSavePending) {
    sendJson(request, 409, "{\"ok\":false,\"error\":\"save_busy\"}");
    return;
  }

  DeviceConfig candidate = config;
  char newSsid[sizeof(candidate.apSsid)];
  char newPassword[sizeof(candidate.apPassword)];
  int64_t channel = 0;
  if (!parseTextParameter(request, "ssid", newSsid, sizeof(newSsid), false) ||
      !parseTextParameter(request, "password", newPassword, sizeof(newPassword), true) ||
      !parseIntegerParameter(request, "channel", 1, 13, channel)) {
    sendJson(request, 400, "{\"ok\":false,\"error\":\"invalid_network_settings\"}");
    return;
  }
  memcpy(candidate.apSsid, newSsid, sizeof(candidate.apSsid));
  if (newPassword[0] != '\0') memcpy(candidate.apPassword, newPassword, sizeof(candidate.apPassword));
  candidate.apChannel = static_cast<uint8_t>(channel);
  if (!validSettings(candidate)) {
    sendJson(request, 400, "{\"ok\":false,\"error\":\"invalid_network_settings\"}");
    return;
  }
  if (sameSettings(candidate, config)) {
    sendJson(request, 200, "{\"ok\":true,\"changed\":false}");
    return;
  }
  if (!queueConfigSave(candidate, true, false)) {
    sendJson(request, 409, "{\"ok\":false,\"error\":\"save_busy\"}");
    return;
  }
  sendJson(request, 202, "{\"ok\":true,\"queued\":true}");
}

static void handleRescanPost(AsyncWebServerRequest *request) {
  rescanRequested = true;
  sendJson(request, 202, "{\"ok\":true,\"queued\":true}");
}

static void handleNotFound(AsyncWebServerRequest *request) {
  request->redirect("http://192.168.4.1/");
}

static void setupWebServer() {
  webServer.on("/", HTTP_GET, handlePage);
  webServer.on("/sensors", HTTP_GET, handlePage);
  webServer.on("/wifi", HTTP_GET, handlePage);
  webServer.on("/app.css", HTTP_GET, [](AsyncWebServerRequest *request) { sendStatic(request, "text/css; charset=utf-8", PAGE_CSS); });
  webServer.on("/app.js", HTTP_GET, [](AsyncWebServerRequest *request) { sendStatic(request, "application/javascript; charset=utf-8", PAGE_JS); });
  webServer.on("/api/status", HTTP_GET, handleStatusGet);
  webServer.on("/api/config", HTTP_GET, handleConfigGet);
  webServer.on("/api/config", HTTP_POST, handleConfigPost);
  webServer.on("/api/wifi", HTTP_POST, handleWifiPost);
  webServer.on("/api/rescan", HTTP_POST, handleRescanPost);
  webServer.onNotFound(handleNotFound);
  webServer.begin();
}

void setup() {
  lastClockMillis = millis();
  pinMode(PIN_FACTORY_BUTTON, INPUT_PULLUP);
  resetRawPressed = digitalRead(PIN_FACTORY_BUTTON) == LOW;
  resetStablePressed = false;
  resetRawChangedAt = millis();

  filesystemReady = LittleFS.begin();
  if (filesystemReady) loadConfig();
  else {
    makeDefaultConfig(config);
    recoveredDefaults = true;
    saveState = SAVE_STATE_ERROR;
  }

  initializeRelayPins();

  Wire.begin(PIN_BME_SDA, PIN_BME_SCL);
  Wire.setClock(100000UL);
  Wire.setClockStretchLimit(10000UL);
  if (startBme280()) nextSensorReadAt = millis();
  else nextBmeProbeAt = millis() + SENSOR_RETRY_MS;

  WiFi.persistent(false);
  apReady = startAccessPoint();
  if (apReady) startCaptiveDns();
  else {
    apRestartPhase = 3;
    apRestartDeadline = millis() + SENSOR_RETRY_MS;
  }

  setupWebServer();
  nextClientCountAt = millis();
}

void loop() {
  const uint32_t now = millis();
  uptimeMs += static_cast<uint32_t>(now - lastClockMillis);
  lastClockMillis = now;

  serviceButton(now);
  serviceSensor(now);
  servicePendingSave();
  serviceFactoryReset();
  serviceApRestart(millis());
  serviceClientCount(millis());
  if (dnsReady) dnsServer.processNextRequest();
  applyRelayOutputs();
  yield();
}
