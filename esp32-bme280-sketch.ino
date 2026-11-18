#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <WiFi.h>
#include <DNSServer.h>
#include <WebServer.h>
#include <Preferences.h>
#include <ArduinoJson.h>
#include <esp_timer.h>
#include <esp_task_wdt.h>
#include <esp_wifi.h>
#include <esp_log.h>
#include <esp_idf_version.h>
#include <math.h>
#include <string.h>
#include <stdio.h>

static constexpr uint64_t PIN_I2C_SDA           = 21ULL;
static constexpr uint64_t PIN_I2C_SCL           = 22ULL;
static constexpr uint64_t PIN_RELAY_TEMP_UNIF   = 25ULL;
static constexpr uint64_t PIN_RELAY_HUMIDITY    = 26ULL;
static constexpr uint64_t PIN_RELAY_PRESSURE    = 27ULL;
static constexpr uint64_t PIN_FACTORY_RESET_BTN = 0ULL;

static constexpr uint64_t CFG_MAGIC             = 0x424D453238305634ULL;
static constexpr uint64_t CFG_VERSION           = 4ULL;
static constexpr uint64_t FACTORY_RESET_HOLD_MS = 5000ULL;
static constexpr uint64_t BTN_DEBOUNCE_MS       = 40ULL;
static constexpr uint64_t BME_RECOVERY_RETRY_MS = 5000ULL;
static constexpr uint64_t BME_WARMUP_DELAY_MS   = 35ULL;
static constexpr uint64_t NVS_COMMIT_DELAY_MS   = 600ULL;
static constexpr uint64_t NVS_MIN_WRITE_GAP_MS  = 2000ULL;
static constexpr uint64_t ACTION_DELAY_MS       = 800ULL;
static constexpr uint64_t DNS_PORT              = 53ULL;
static constexpr uint64_t HTTP_PORT             = 80ULL;
static constexpr uint64_t WDT_TIMEOUT_SEC       = 15ULL;

static constexpr uint64_t RELAY_MODE_UNIFIED    = 0ULL;
static constexpr uint64_t RELAY_MODE_INDIVIDUAL = 1ULL;

static constexpr uint64_t COND_ABOVE_HIGH       = 0ULL;
static constexpr uint64_t COND_BELOW_LOW        = 1ULL;
static constexpr uint64_t COND_OUTSIDE_RANGE    = 2ULL;
static constexpr uint64_t COND_INSIDE_RANGE     = 3ULL;

static constexpr uint64_t FAILSAFE_FORCE_OFF    = 0ULL;
static constexpr uint64_t FAILSAFE_HOLD_LAST    = 1ULL;
static constexpr uint64_t FAILSAFE_FORCE_ON     = 2ULL;

struct alignas(8) SystemConfig {
  uint64_t magic;
  uint64_t version;
  uint64_t sequence;

  uint64_t relayMode;
  uint64_t relayActiveLow;
  uint64_t failsafeMode;
  uint64_t minRelaySwitchMs;
  uint64_t sensorPollMs;

  uint64_t tempEnable;
  uint64_t tempCondition;
  double   tempHigh;
  double   tempLow;
  double   tempHyst;
  double   tempOffset;

  uint64_t humEnable;
  uint64_t humCondition;
  double   humHigh;
  double   humLow;
  double   humHyst;
  double   humOffset;

  uint64_t presEnable;
  uint64_t presCondition;
  double   presHigh;
  double   presLow;
  double   presHyst;
  double   presOffset;

  uint64_t wifiChannel;
  uint64_t wifiHidden;
  uint64_t wifiMaxConn;
  char     wifiSsid[64];
  char     wifiPass[64];

  uint64_t checksum;
};

struct alignas(8) RuntimeState {
  uint64_t i2cInitialized;
  uint64_t sensorHealthy;
  uint64_t hasValidSample;
  uint64_t activeI2CAddress;
  uint64_t sensorWarmupUntilMs;
  uint64_t lastSensorPollMs;
  uint64_t lastSensorInitTryMs;
  uint64_t consecutiveSensorErrors;
  uint64_t totalSensorReads;
  uint64_t totalSensorFaults;

  double   rawTempC;
  double   rawHumPct;
  double   rawPresHpa;

  double   temperatureC;
  double   temperatureF;
  double   humidityPct;
  double   pressureHpa;
  double   dewPointC;
  double   altitudeM;

  uint64_t tempTriggered;
  uint64_t humTriggered;
  uint64_t presTriggered;

  uint64_t relayLogicalState[3];
  uint64_t relayDesiredState[3];
  uint64_t relayLastSwitchMs[3];
  uint64_t relaySwitchCount[3];

  uint64_t btnRawState;
  uint64_t btnStableState;
  uint64_t btnLastChangeMs;
  uint64_t btnPressStartMs;
  uint64_t btnResetTriggered;

  uint64_t nvsDirty;
  uint64_t nvsStageMs;
  uint64_t nvsLastWriteMs;
  uint64_t nvsWriteCount;
  uint64_t nvsBankHealth;

  uint64_t pendingWifiReload;
  uint64_t wifiReloadAtMs;
  uint64_t pendingReboot;
  uint64_t rebootAtMs;
};

static SystemConfig     g_config;
static RuntimeState     g_state;
static Adafruit_BME280  g_bme;
static DNSServer        g_dnsServer;
static WebServer        g_webServer((int)HTTP_PORT);
static Preferences      g_prefs;
static IPAddress        g_apIP(192, 168, 4, 1);
static IPAddress        g_netMask(255, 255, 255, 0);

static char             g_jsonBuf[3072];

static const char INDEX_HTML[] PROGMEM = R"rawliteral(<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width,initial-scale=1.0,maximum-scale=5.0">
<title>ESP32 BME280 Industrial Controller</title>
<style>
:root{
  --bg:#0b1120;
  --panel:#131c31;
  --panel-alt:#1a2642;
  --border:#28375c;
  --text:#f1f5f9;
  --muted:#94a3b8;
  --accent:#38bdf8;
  --accent-hover:#0284c7;
  --ok:#10b981;
  --warn:#f59e0b;
  --danger:#ef4444;
  --radius:14px;
  --shadow:0 10px 28px rgba(0,0,0,0.38);
}
*{box-sizing:border-box;margin:0;padding:0;-webkit-tap-highlight-color:transparent;}
body{
  font-family:-apple-system,BlinkMacSystemFont,"Segoe UI",Roboto,Helvetica,Arial,sans-serif;
  background:var(--bg);
  color:var(--text);
  line-height:1.5;
  min-height:100vh;
  padding-bottom:84px;
}
.header{
  position:sticky;
  top:0;
  z-index:100;
  background:rgba(11,17,32,0.92);
  backdrop-filter:blur(12px);
  border-bottom:1px solid var(--border);
  padding:14px 20px;
}
.header-inner{
  max-width:1280px;
  margin:0 auto;
  display:flex;
  flex-wrap:wrap;
  align-items:center;
  justify-content:space-between;
  gap:12px;
}
.brand{
  display:flex;
  align-items:center;
  gap:12px;
}
.brand-icon{
  width:38px;
  height:38px;
  border-radius:10px;
  background:linear-gradient(135deg,#0284c7,#38bdf8);
  display:flex;
  align-items:center;
  justify-content:center;
  font-weight:800;
  color:#fff;
  font-size:15px;
  box-shadow:0 4px 12px rgba(56,189,248,0.3);
}
.brand-title{font-size:1.1rem;font-weight:700;letter-spacing:-0.01em;}
.brand-sub{font-size:0.76rem;color:var(--muted);}
.status-bar{
  display:flex;
  flex-wrap:wrap;
  align-items:center;
  gap:8px;
}
.pill{
  display:inline-flex;
  align-items:center;
  gap:6px;
  padding:5px 11px;
  border-radius:999px;
  font-size:0.75rem;
  font-weight:600;
  background:var(--panel-alt);
  border:1px solid var(--border);
  color:var(--text);
}
.dot{
  width:8px;
  height:8px;
  border-radius:50%;
  background:var(--muted);
}
.dot.ok{background:var(--ok);box-shadow:0 0 8px var(--ok);}
.dot.warn{background:var(--warn);box-shadow:0 0 8px var(--warn);}
.dot.err{background:var(--danger);box-shadow:0 0 8px var(--danger);}
.nav-tabs{
  display:flex;
  gap:8px;
  max-width:1280px;
  margin:16px auto 0;
  padding:0 20px;
}
.tab-btn{
  flex:1;
  padding:12px 16px;
  border-radius:11px;
  border:1px solid var(--border);
  background:var(--panel);
  color:var(--muted);
  font-weight:600;
  font-size:0.9rem;
  cursor:pointer;
  transition:all 0.2s ease;
  text-align:center;
}
.tab-btn:hover{color:var(--text);border-color:#3b5284;}
.tab-btn.active{
  background:linear-gradient(135deg,#0284c7,#0ea5e9);
  color:#fff;
  border-color:transparent;
  box-shadow:0 4px 14px rgba(14,165,233,0.35);
}
.container{
  max-width:1280px;
  margin:20px auto;
  padding:0 20px;
}
.page{display:none;animation:fadeIn 0.2s ease;}
.page.active{display:block;}
@keyframes fadeIn{from{opacity:0;transform:translateY(4px);}to{opacity:1;transform:translateY(0);}}
.grid-3{
  display:grid;
  grid-template-columns:repeat(3,1fr);
  gap:18px;
  margin-bottom:18px;
}
.grid-2{
  display:grid;
  grid-template-columns:repeat(2,1fr);
  gap:18px;
  margin-bottom:18px;
}
@media(max-width:1024px){
  .grid-3{grid-template-columns:repeat(2,1fr);}
}
@media(max-width:720px){
  .grid-3,.grid-2{grid-template-columns:1fr;}
  .nav-tabs{
    position:fixed;
    bottom:0;
    left:0;
    right:0;
    margin:0;
    padding:10px 12px;
    background:rgba(11,17,32,0.96);
    backdrop-filter:blur(12px);
    border-top:1px solid var(--border);
    z-index:100;
  }
  .tab-btn{padding:10px 8px;font-size:0.78rem;}
}
.card{
  background:var(--panel);
  border:1px solid var(--border);
  border-radius:var(--radius);
  padding:20px;
  box-shadow:var(--shadow);
  display:flex;
  flex-direction:column;
  justify-content:space-between;
}
.card-header{
  display:flex;
  align-items:center;
  justify-content:space-between;
  margin-bottom:14px;
  gap:10px;
}
.card-title{
  font-size:0.95rem;
  font-weight:700;
  color:var(--muted);
  text-transform:uppercase;
  letter-spacing:0.05em;
}
.badge{
  padding:4px 10px;
  border-radius:999px;
  font-size:0.72rem;
  font-weight:700;
  text-transform:uppercase;
  letter-spacing:0.04em;
}
.badge-ok{background:rgba(16,185,129,0.16);color:#34d399;border:1px solid rgba(16,185,129,0.35);}
.badge-warn{background:rgba(245,158,11,0.16);color:#fbbf24;border:1px solid rgba(245,158,11,0.35);}
.badge-danger{background:rgba(239,68,68,0.18);color:#f87171;border:1px solid rgba(239,68,68,0.4);}
.badge-muted{background:rgba(148,163,184,0.15);color:#cbd5e1;border:1px solid rgba(148,163,184,0.3);}
.metric-main{
  display:flex;
  align-items:baseline;
  gap:8px;
  margin:4px 0 6px;
}
.metric-val{
  font-size:2.55rem;
  font-weight:800;
  letter-spacing:-0.03em;
  font-variant-numeric:tabular-nums;
}
.metric-unit{
  font-size:1.1rem;
  font-weight:600;
  color:var(--muted);
}
.metric-sub{
  font-size:0.83rem;
  color:var(--muted);
  margin-bottom:14px;
  font-variant-numeric:tabular-nums;
}
.bar-wrap{
  width:100%;
  height:8px;
  background:var(--panel-alt);
  border-radius:999px;
  overflow:hidden;
  margin-bottom:10px;
  border:1px solid var(--border);
}
.bar-fill{
  height:100%;
  width:50%;
  background:linear-gradient(90deg,#0284c7,#38bdf8);
  border-radius:999px;
  transition:width 0.35s ease,background 0.35s ease;
}
.bar-fill.trig{background:linear-gradient(90deg,#dc2626,#f87171);}
.thresh-meta{
  display:flex;
  justify-content:space-between;
  font-size:0.76rem;
  color:var(--muted);
  margin-bottom:12px;
  font-variant-numeric:tabular-nums;
}
canvas.spark{
  width:100%;
  height:56px;
  display:block;
  background:rgba(11,17,32,0.45);
  border-radius:8px;
  border:1px solid rgba(40,55,92,0.6);
}
.relay-grid{
  display:grid;
  grid-template-columns:repeat(3,1fr);
  gap:12px;
  margin-top:8px;
}
@media(max-width:640px){
  .relay-grid{grid-template-columns:1fr;}
}
.relay-box{
  background:var(--panel-alt);
  border:1px solid var(--border);
  border-radius:11px;
  padding:14px;
  display:flex;
  flex-direction:column;
  gap:6px;
}
.relay-box.active{
  border-color:rgba(16,185,129,0.6);
  background:rgba(16,185,129,0.08);
}
.relay-head{
  display:flex;
  align-items:center;
  justify-content:space-between;
  font-weight:700;
  font-size:0.88rem;
}
.relay-meta{
  font-size:0.77rem;
  color:var(--muted);
  font-variant-numeric:tabular-nums;
}
.kv-list{
  display:grid;
  grid-template-columns:repeat(2,1fr);
  gap:10px;
}
@media(max-width:480px){
  .kv-list{grid-template-columns:1fr;}
}
.kv-item{
  background:var(--panel-alt);
  border:1px solid var(--border);
  border-radius:10px;
  padding:10px 13px;
}
.kv-label{font-size:0.74rem;color:var(--muted);margin-bottom:2px;}
.kv-val{font-size:0.9rem;font-weight:700;font-variant-numeric:tabular-nums;}
.form-row{
  display:grid;
  grid-template-columns:repeat(2,1fr);
  gap:14px;
  margin-bottom:14px;
}
@media(max-width:600px){
  .form-row{grid-template-columns:1fr;}
}
.form-group{
  display:flex;
  flex-direction:column;
  gap:6px;
  margin-bottom:12px;
}
.form-label{
  font-size:0.82rem;
  font-weight:600;
  color:#cbd5e1;
}
.form-hint{
  font-size:0.73rem;
  color:var(--muted);
}
.input,.select{
  width:100%;
  padding:10px 12px;
  background:#0b1120;
  border:1px solid var(--border);
  border-radius:9px;
  color:var(--text);
  font-size:0.92rem;
  outline:none;
  transition:border-color 0.2s;
}
.input:focus,.select:focus{border-color:var(--accent);}
.toggle-row{
  display:flex;
  align-items:center;
  justify-content:space-between;
  background:var(--panel-alt);
  border:1px solid var(--border);
  padding:11px 14px;
  border-radius:10px;
  margin-bottom:14px;
}
.switch{
  position:relative;
  display:inline-block;
  width:46px;
  height:26px;
}
.switch input{opacity:0;width:0;height:0;}
.slider{
  position:absolute;
  cursor:pointer;
  inset:0;
  background:#334155;
  transition:.25s;
  border-radius:26px;
}
.slider:before{
  position:absolute;
  content:"";
  height:20px;
  width:20px;
  left:3px;
  bottom:3px;
  background:#fff;
  transition:.25s;
  border-radius:50%;
}
input:checked + .slider{background:var(--ok);}
input:checked + .slider:before{transform:translateX(20px);}
.btn-row{
  display:flex;
  flex-wrap:wrap;
  gap:12px;
  margin-top:12px;
}
.btn{
  padding:11px 20px;
  border-radius:10px;
  border:none;
  font-weight:700;
  font-size:0.9rem;
  cursor:pointer;
  transition:all 0.2s ease;
  display:inline-flex;
  align-items:center;
  justify-content:center;
  gap:8px;
}
.btn-primary{
  background:linear-gradient(135deg,#0284c7,#0ea5e9);
  color:#fff;
  box-shadow:0 4px 14px rgba(14,165,233,0.3);
}
.btn-primary:hover{filter:brightness(1.1);}
.btn-secondary{
  background:var(--panel-alt);
  color:var(--text);
  border:1px solid var(--border);
}
.btn-secondary:hover{border-color:var(--accent);}
.btn-danger{
  background:rgba(239,68,68,0.18);
  color:#f87171;
  border:1px solid rgba(239,68,68,0.45);
}
.btn-danger:hover{background:rgba(239,68,68,0.3);}
.toast{
  position:fixed;
  bottom:24px;
  right:24px;
  z-index:999;
  padding:13px 18px;
  border-radius:11px;
  background:var(--panel);
  border:1px solid var(--border);
  color:var(--text);
  font-weight:600;
  font-size:0.88rem;
  box-shadow:0 12px 32px rgba(0,0,0,0.55);
  transform:translateY(120%);
  opacity:0;
  transition:all 0.28s ease;
}
.toast.show{transform:translateY(0);opacity:1;}
.toast.ok{border-left:4px solid var(--ok);}
.toast.err{border-left:4px solid var(--danger);}
@media(max-width:720px){
  .toast{bottom:78px;left:16px;right:16px;}
}
.modal-backdrop{
  position:fixed;
  inset:0;
  background:rgba(0,0,0,0.7);
  backdrop-filter:blur(4px);
  display:none;
  align-items:center;
  justify-content:center;
  z-index:500;
  padding:20px;
}
.modal-backdrop.open{display:flex;}
.modal{
  background:var(--panel);
  border:1px solid var(--border);
  border-radius:var(--radius);
  max-width:440px;
  width:100%;
  padding:22px;
  box-shadow:var(--shadow);
}
.modal-title{font-size:1.1rem;font-weight:700;margin-bottom:8px;}
.modal-body{font-size:0.88rem;color:var(--muted);margin-bottom:18px;}
.pass-wrap{position:relative;display:flex;align-items:center;}
.pass-toggle{
  position:absolute;
  right:10px;
  background:none;
  border:none;
  color:var(--accent);
  font-size:0.78rem;
  font-weight:700;
  cursor:pointer;
  padding:4px 6px;
}
</style>
</head>
<body>

<header class="header">
  <div class="header-inner">
    <div class="brand">
      <div class="brand-icon">BME</div>
      <div>
        <div class="brand-title">ESP32 BME280 Environmental Controller</div>
        <div class="brand-sub">64-Bit Non-Blocking Production Telemetry &amp; Relay Automation</div>
      </div>
    </div>
    <div class="status-bar">
      <div class="pill"><span id="dotConn" class="dot ok"></span><span id="txtConn">Link Active</span></div>
      <div class="pill"><span id="dotBme" class="dot ok"></span><span id="txtBme">BME280 Online</span></div>
      <div class="pill" id="pillMode">Mode: Unified (GPIO 25)</div>
      <div class="pill" id="pillUptime">Uptime: 0s</div>
    </div>
  </div>
</header>

<nav class="nav-tabs">
  <button type="button" class="tab-btn active" data-tab="page-dash" onclick="switchTab('page-dash')">Dashboard Status</button>
  <button type="button" class="tab-btn" data-tab="page-sensor" onclick="switchTab('page-sensor')">BME280 &amp; Relay Settings</button>
  <button type="button" class="tab-btn" data-tab="page-wifi" onclick="switchTab('page-wifi')">Wi-Fi AP &amp; System</button>
</nav>

<main class="container">
  <!-- PAGE 1: DASHBOARD -->
  <section id="page-dash" class="page active">
    <div class="grid-3">
      <!-- Temperature Card -->
      <div class="card">
        <div>
          <div class="card-header">
            <span id="dashTitleTemp" class="card-title">Temperature (GPIO 25)</span>
            <span id="badgeTemp" class="badge badge-ok">NORMAL</span>
          </div>
          <div class="metric-main">
            <span id="valTempC" class="metric-val">--.-</span>
            <span class="metric-unit">&deg;C</span>
          </div>
          <div id="valTempSub" class="metric-sub">--.- &deg;F | Offset: 0.00 &deg;C</div>
          <div class="bar-wrap"><div id="barTemp" class="bar-fill"></div></div>
          <div class="thresh-meta">
            <span id="metaTempLow">Low: -- &deg;C</span>
            <span id="metaTempCond">Rule: --</span>
            <span id="metaTempHigh">High: -- &deg;C</span>
          </div>
        </div>
        <canvas id="sparkTemp" class="spark" width="320" height="56"></canvas>
      </div>

      <!-- Humidity Card -->
      <div class="card">
        <div>
          <div class="card-header">
            <span id="dashTitleHum" class="card-title">Humidity (GPIO 25 Unified)</span>
            <span id="badgeHum" class="badge badge-ok">NORMAL</span>
          </div>
          <div class="metric-main">
            <span id="valHum" class="metric-val">--.-</span>
            <span class="metric-unit">%RH</span>
          </div>
          <div id="valHumSub" class="metric-sub">Dew Point: --.- &deg;C | Offset: 0.00 %</div>
          <div class="bar-wrap"><div id="barHum" class="bar-fill"></div></div>
          <div class="thresh-meta">
            <span id="metaHumLow">Low: -- %</span>
            <span id="metaHumCond">Rule: --</span>
            <span id="metaHumHigh">High: -- %</span>
          </div>
        </div>
        <canvas id="sparkHum" class="spark" width="320" height="56"></canvas>
      </div>

      <!-- Pressure Card -->
      <div class="card">
        <div>
          <div class="card-header">
            <span id="dashTitlePres" class="card-title">Pressure (GPIO 25 Unified)</span>
            <span id="badgePres" class="badge badge-ok">NORMAL</span>
          </div>
          <div class="metric-main">
            <span id="valPres" class="metric-val">----.-</span>
            <span class="metric-unit">hPa</span>
          </div>
          <div id="valPresSub" class="metric-sub">Est. Altitude: ---.- m | Offset: 0.00 hPa</div>
          <div class="bar-wrap"><div id="barPres" class="bar-fill"></div></div>
          <div class="thresh-meta">
            <span id="metaPresLow">Low: -- hPa</span>
            <span id="metaPresCond">Rule: --</span>
            <span id="metaPresHigh">High: -- hPa</span>
          </div>
        </div>
        <canvas id="sparkPres" class="spark" width="320" height="56"></canvas>
      </div>
    </div>

    <div class="grid-2">
      <!-- Relay Control Status Card -->
      <div class="card">
        <div>
          <div class="card-header">
            <span class="card-title">Relay Output States (GPIO 25 / 26 / 27)</span>
            <span id="badgeRelayMode" class="badge badge-ok">UNIFIED MODE</span>
          </div>
          <div class="relay-grid">
            <div id="boxR0" class="relay-box">
              <div class="relay-head">
                <span id="lblR0Title">GPIO 25 (Unified)</span>
                <span id="badgeR0" class="badge badge-muted">OFF</span>
              </div>
              <div id="metaR0Role" class="relay-meta">Trigger: Any Threshold</div>
              <div id="metaR0Stat" class="relay-meta">Switches: 0</div>
            </div>
            <div id="boxR1" class="relay-box">
              <div class="relay-head">
                <span>GPIO 26 (Humidity)</span>
                <span id="badgeR1" class="badge badge-muted">OFF</span>
              </div>
              <div id="metaR1Role" class="relay-meta">Trigger: Humidity</div>
              <div id="metaR1Stat" class="relay-meta">Switches: 0</div>
            </div>
            <div id="boxR2" class="relay-box">
              <div class="relay-head">
                <span>GPIO 27 (Pressure)</span>
                <span id="badgeR2" class="badge badge-muted">OFF</span>
              </div>
              <div id="metaR2Role" class="relay-meta">Trigger: Pressure</div>
              <div id="metaR2Stat" class="relay-meta">Switches: 0</div>
            </div>
          </div>
        </div>
      </div>

      <!-- System & Hardware Telemetry -->
      <div class="card">
        <div>
          <div class="card-header">
            <span class="card-title">Hardware &amp; Longevity Telemetry</span>
            <span id="badgeNvs" class="badge badge-ok">NVS DUAL-BANK OK</span>
          </div>
          <div class="kv-list">
            <div class="kv-item">
              <div class="kv-label">BME280 Bus (SDA 21 / SCL 22)</div>
              <div id="kvI2c" class="kv-val">0x76 | 0 Faults</div>
            </div>
            <div class="kv-item">
              <div class="kv-label">Sensor Poll / Reads</div>
              <div id="kvReads" class="kv-val">1000 ms | 0 Reads</div>
            </div>
            <div class="kv-item">
              <div class="kv-label">Heap Integrity (Free / Min)</div>
              <div id="kvHeap" class="kv-val">-- KB / -- KB</div>
            </div>
            <div class="kv-item">
              <div class="kv-label">NVS Wear &amp; Boot Button (GPIO 0)</div>
              <div id="kvNvsBtn" class="kv-val">Seq #1 | Idle</div>
            </div>
          </div>
        </div>
      </div>
    </div>
  </section>

  <!-- PAGE 2: SENSOR & RELAY SETTINGS -->
  <section id="page-sensor" class="page">
    <form id="formSensor" onsubmit="saveSensorSettings(event)">
      <div class="card" style="margin-bottom:18px;">
        <div class="card-header">
          <span class="card-title">Relay Architecture &amp; Hardware Protection</span>
          <span class="badge badge-ok">GPIO 25 / 26 / 27</span>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label class="form-label" for="relayMode">Relay Control Mode</label>
            <select id="relayMode" class="select" onchange="syncRelayModeLabels()">
              <option value="0">Unified Control (GPIO 25 triggers on ANY sensor threshold)</option>
              <option value="1">Individual Control (GPIO 25=Temp, GPIO 26=Humidity, GPIO 27=Pressure)</option>
            </select>
            <span class="form-hint">Select Unified (single relay on GPIO 25) or Individual 3-channel relay outputs (GPIO 25, 26, 27).</span>
          </div>
          <div class="form-group">
            <label class="form-label" for="relayActiveLow">Relay Electrical Polarity</label>
            <select id="relayActiveLow" class="select">
              <option value="0">Active HIGH (3.3V = Relay ON, 0V = Relay OFF)</option>
              <option value="1">Active LOW (0V = Relay ON, 3.3V = Relay OFF)</option>
            </select>
            <span class="form-hint">Matches optocoupler or solid-state relay module polarity.</span>
          </div>
        </div>
        <div class="form-row">
          <div class="form-group">
            <label class="form-label" for="failsafeMode">Sensor Fault Failsafe Behavior</label>
            <select id="failsafeMode" class="select">
              <option value="0">Force Relays OFF on Sensor Fault (Recommended Safe State)</option>
              <option value="1">Hold Last Relay State on Sensor Fault</option>
              <option value="2">Force Active Relays ON on Sensor Fault</option>
            </select>
          </div>
          <div class="form-group">
            <label class="form-label" for="minRelaySwitchMs">Minimum Relay Switch Hold Time (ms)</label>
            <input type="number" id="minRelaySwitchMs" class="input" min="0" max="300000" step="1" required>
            <span class="form-hint">Anti-short-cycle protection (0 to 300,000 ms).</span>
          </div>
        </div>
        <div class="form-group">
          <label class="form-label" for="sensorPollMs">BME280 Sampling Interval (ms)</label>
          <input type="number" id="sensorPollMs" class="input" min="250" max="60000" step="1" required>
          <span class="form-hint">Non-blocking sensor read period between 250 ms and 60,000 ms.</span>
        </div>
      </div>

      <div class="grid-3">
        <!-- Temperature Threshold Settings -->
        <div class="card">
          <div>
            <div class="card-header">
              <span id="cfgTitleTemp" class="card-title">Temperature Rules (GPIO 25)</span>
              <span class="badge badge-ok">&deg;C</span>
            </div>
            <div class="toggle-row">
              <span class="form-label">Enable Temperature Trigger</span>
              <label class="switch"><input type="checkbox" id="tempEnable"><span class="slider"></span></label>
            </div>
            <div class="form-group">
              <label class="form-label" for="tempCondition">Trigger Condition</label>
              <select id="tempCondition" class="select">
                <option value="0">Above High Threshold (&ge; High)</option>
                <option value="1">Below Low Threshold (&le; Low)</option>
                <option value="2">Outside Window (&le; Low OR &ge; High)</option>
                <option value="3">Inside Window (&ge; Low AND &le; High)</option>
              </select>
            </div>
            <div class="form-row">
              <div class="form-group">
                <label class="form-label" for="tempHigh">High Threshold (&deg;C)</label>
                <input type="number" id="tempHigh" class="input" min="-40" max="85" step="0.01" required>
              </div>
              <div class="form-group">
                <label class="form-label" for="tempLow">Low Threshold (&deg;C)</label>
                <input type="number" id="tempLow" class="input" min="-40" max="85" step="0.01" required>
              </div>
            </div>
            <div class="form-row">
              <div class="form-group">
                <label class="form-label" for="tempHyst">Hysteresis (&deg;C)</label>
                <input type="number" id="tempHyst" class="input" min="0" max="20" step="0.01" required>
              </div>
              <div class="form-group">
                <label class="form-label" for="tempOffset">Calibration Offset (&deg;C)</label>
                <input type="number" id="tempOffset" class="input" min="-15" max="15" step="0.01" required>
              </div>
            </div>
          </div>
        </div>

        <!-- Humidity Threshold Settings -->
        <div class="card">
          <div>
            <div class="card-header">
              <span id="cfgTitleHum" class="card-title">Humidity Rules (GPIO 25 Unified)</span>
              <span class="badge badge-ok">%RH</span>
            </div>
            <div class="toggle-row">
              <span class="form-label">Enable Humidity Trigger</span>
              <label class="switch"><input type="checkbox" id="humEnable"><span class="slider"></span></label>
            </div>
            <div class="form-group">
              <label class="form-label" for="humCondition">Trigger Condition</label>
              <select id="humCondition" class="select">
                <option value="0">Above High Threshold (&ge; High)</option>
                <option value="1">Below Low Threshold (&le; Low)</option>
                <option value="2">Outside Window (&le; Low OR &ge; High)</option>
                <option value="3">Inside Window (&ge; Low AND &le; High)</option>
              </select>
            </div>
            <div class="form-row">
              <div class="form-group">
                <label class="form-label" for="humHigh">High Threshold (%RH)</label>
                <input type="number" id="humHigh" class="input" min="0" max="100" step="0.01" required>
              </div>
              <div class="form-group">
                <label class="form-label" for="humLow">Low Threshold (%RH)</label>
                <input type="number" id="humLow" class="input" min="0" max="100" step="0.01" required>
              </div>
            </div>
            <div class="form-row">
              <div class="form-group">
                <label class="form-label" for="humHyst">Hysteresis (%RH)</label>
                <input type="number" id="humHyst" class="input" min="0" max="30" step="0.01" required>
              </div>
              <div class="form-group">
                <label class="form-label" for="humOffset">Calibration Offset (%RH)</label>
                <input type="number" id="humOffset" class="input" min="-25" max="25" step="0.01" required>
              </div>
            </div>
          </div>
        </div>

        <!-- Pressure Threshold Settings -->
        <div class="card">
          <div>
            <div class="card-header">
              <span id="cfgTitlePres" class="card-title">Pressure Rules (GPIO 25 Unified)</span>
              <span class="badge badge-ok">hPa</span>
            </div>
            <div class="toggle-row">
              <span class="form-label">Enable Pressure Trigger</span>
              <label class="switch"><input type="checkbox" id="presEnable"><span class="slider"></span></label>
            </div>
            <div class="form-group">
              <label class="form-label" for="presCondition">Trigger Condition</label>
              <select id="presCondition" class="select">
                <option value="0">Above High Threshold (&ge; High)</option>
                <option value="1">Below Low Threshold (&le; Low)</option>
                <option value="2">Outside Window (&le; Low OR &ge; High)</option>
                <option value="3">Inside Window (&ge; Low AND &le; High)</option>
              </select>
            </div>
            <div class="form-row">
              <div class="form-group">
                <label class="form-label" for="presHigh">High Threshold (hPa)</label>
                <input type="number" id="presHigh" class="input" min="300" max="1100" step="0.01" required>
              </div>
              <div class="form-group">
                <label class="form-label" for="presLow">Low Threshold (hPa)</label>
                <input type="number" id="presLow" class="input" min="300" max="1100" step="0.01" required>
              </div>
            </div>
            <div class="form-row">
              <div class="form-group">
                <label class="form-label" for="presHyst">Hysteresis (hPa)</label>
                <input type="number" id="presHyst" class="input" min="0" max="50" step="0.01" required>
              </div>
              <div class="form-group">
                <label class="form-label" for="presOffset">Calibration Offset (hPa)</label>
                <input type="number" id="presOffset" class="input" min="-50" max="50" step="0.01" required>
              </div>
            </div>
          </div>
        </div>
      </div>

      <div class="card">
        <div class="btn-row">
          <button type="submit" class="btn btn-primary">Save BME280 &amp; Relay Configuration</button>
          <button type="button" class="btn btn-secondary" onclick="loadAllSettings(true)">Reload Saved Values</button>
        </div>
      </div>
    </form>
  </section>

  <!-- PAGE 3: WI-FI AP & SYSTEM -->
  <section id="page-wifi" class="page">
    <div class="grid-2">
      <div class="card">
        <form id="formWifi" onsubmit="saveWifiSettings(event)">
          <div class="card-header">
            <span class="card-title">Wi-Fi Access Point &amp; Captive Portal Settings</span>
            <span class="badge badge-ok">192.168.4.1</span>
          </div>
          <div class="form-group">
            <label class="form-label" for="wifiSsid">Access Point SSID (1 - 32 Characters)</label>
            <input type="text" id="wifiSsid" class="input" minlength="1" maxlength="32" required>
            <span class="form-hint">Default SSID: ESP32_BME280</span>
          </div>
          <div class="form-group">
            <label class="form-label" for="wifiPass">WPA2 Password (8 - 63 Characters)</label>
            <div class="pass-wrap">
              <input type="password" id="wifiPass" class="input" minlength="8" maxlength="63" required style="padding-right:64px;">
              <button type="button" class="pass-toggle" onclick="togglePass()">SHOW</button>
            </div>
            <span class="form-hint">Default Password: 12345678</span>
          </div>
          <div class="form-row">
            <div class="form-group">
              <label class="form-label" for="wifiChannel">Wi-Fi Channel (2.4 GHz)</label>
              <select id="wifiChannel" class="select">
                <option value="1">Channel 1</option>
                <option value="2">Channel 2</option>
                <option value="3">Channel 3</option>
                <option value="4">Channel 4</option>
                <option value="5">Channel 5</option>
                <option value="6">Channel 6</option>
                <option value="7">Channel 7</option>
                <option value="8">Channel 8</option>
                <option value="9">Channel 9</option>
                <option value="10">Channel 10</option>
                <option value="11">Channel 11</option>
                <option value="12">Channel 12</option>
                <option value="13">Channel 13</option>
              </select>
            </div>
            <div class="form-group">
              <label class="form-label" for="wifiMaxConn">Max Simultaneous Clients</label>
              <select id="wifiMaxConn" class="select">
                <option value="1">1 Client</option>
                <option value="2">2 Clients</option>
                <option value="3">3 Clients</option>
                <option value="4">4 Clients</option>
              </select>
            </div>
          </div>
          <div class="toggle-row">
            <span class="form-label">Hide Access Point SSID Broadcast</span>
            <label class="switch"><input type="checkbox" id="wifiHidden"><span class="slider"></span></label>
          </div>
          <div class="btn-row">
            <button type="submit" class="btn btn-primary">Save &amp; Restart Wi-Fi AP</button>
          </div>
        </form>
      </div>

      <div class="card">
        <div>
          <div class="card-header">
            <span class="card-title">System Maintenance &amp; Factory Reset</span>
            <span class="badge badge-warn">GPIO 0 BOOT BUTTON</span>
          </div>
          <p style="font-size:0.86rem;color:var(--muted);margin-bottom:14px;">
            Hold the onboard <strong>BOOT (GPIO 0)</strong> button continuously for <strong>5 seconds</strong> at any time, or use the controls below to restore all sensor thresholds, relay modes, and Wi-Fi AP credentials (<strong>ESP32_BME280</strong> / <strong>12345678</strong>) to factory defaults.
          </p>
          <div class="kv-list" style="margin-bottom:16px;">
            <div class="kv-item">
              <div class="kv-label">Captive Portal Gateway</div>
              <div class="kv-val">http://192.168.4.1</div>
            </div>
            <div class="kv-item">
              <div class="kv-label">Connected AP Stations</div>
              <div id="kvStations" class="kv-val">1 Client</div>
            </div>
            <div class="kv-item">
              <div class="kv-label">NVS Dual-Bank Checksum</div>
              <div id="kvChecksum" class="kv-val">Verified 64-Bit</div>
            </div>
            <div class="kv-item">
              <div class="kv-label">GPIO 0 Reset Button State</div>
              <div id="kvBtnState" class="kv-val">Released (0 ms)</div>
            </div>
          </div>
        </div>
        <div class="btn-row">
          <button type="button" class="btn btn-secondary" onclick="openModal('reboot')">Reboot Controller</button>
          <button type="button" class="btn btn-danger" onclick="openModal('reset')">Factory Reset All Settings</button>
        </div>
      </div>
    </div>
  </section>
</main>

<div id="modalBackdrop" class="modal-backdrop">
  <div class="modal">
    <div id="modalTitle" class="modal-title">Confirm Action</div>
    <div id="modalBody" class="modal-body">Are you sure?</div>
    <div class="btn-row" style="justify-content:flex-end;">
      <button type="button" class="btn btn-secondary" onclick="closeModal()">Cancel</button>
      <button type="button" id="modalConfirmBtn" class="btn btn-danger">Confirm</button>
    </div>
  </div>
</div>

<div id="toast" class="toast"></div>

<script>
const histTemp = [];
const histHum  = [];
const histPres = [];
const MAX_PTS  = 60;
let pollBusy   = false;
let toastTimer = null;

function showToast(msg, ok) {
  const el = document.getElementById('toast');
  el.textContent = msg;
  el.className = 'toast show ' + (ok ? 'ok' : 'err');
  if (toastTimer) clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { el.className = 'toast'; }, 3800);
}

function switchTab(tabId) {
  document.querySelectorAll('.page').forEach(p => p.classList.remove('active'));
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
  const target = document.getElementById(tabId);
  if (target) target.classList.add('active');
  const btn = document.querySelector(`.tab-btn[data-tab="${tabId}"]`);
  if (btn) btn.classList.add('active');
}

function syncRelayModeLabels() {
  const isUnif = Number(document.getElementById('relayMode').value) === 0;
  document.getElementById('cfgTitleTemp').textContent = isUnif ? 'Temperature Rules (GPIO 25 Unified)' : 'Temperature Rules (GPIO 25)';
  document.getElementById('cfgTitleHum').textContent  = isUnif ? 'Humidity Rules (GPIO 25 Unified)'    : 'Humidity Rules (GPIO 26)';
  document.getElementById('cfgTitlePres').textContent = isUnif ? 'Pressure Rules (GPIO 25 Unified)'    : 'Pressure Rules (GPIO 27)';
}

function togglePass() {
  const inp = document.getElementById('wifiPass');
  const btn = document.querySelector('.pass-toggle');
  if (inp.type === 'password') {
    inp.type = 'text';
    btn.textContent = 'HIDE';
  } else {
    inp.type = 'password';
    btn.textContent = 'SHOW';
  }
}

function condName(c) {
  switch (Number(c)) {
    case 0: return 'Above High';
    case 1: return 'Below Low';
    case 2: return 'Outside Window';
    case 3: return 'Inside Window';
    default: return 'Rule';
  }
}

function formatUptime(ms) {
  let s = Math.floor(Number(ms) / 1000);
  const y = Math.floor(s / 31536000); s %= 31536000;
  const d = Math.floor(s / 86400);    s %= 86400;
  const h = Math.floor(s / 3600);     s %= 3600;
  const m = Math.floor(s / 60);       s %= 60;
  const parts = [];
  if (y > 0) parts.push(y + 'y');
  if (d > 0) parts.push(d + 'd');
  if (h > 0) parts.push(h + 'h');
  if (m > 0) parts.push(m + 'm');
  parts.push(s + 's');
  return parts.join(' ');
}

function drawSpark(canvasId, arr, color) {
  const c = document.getElementById(canvasId);
  if (!c) return;
  const ctx = c.getContext('2d');
  const w = c.width;
  const h = c.height;
  ctx.clearRect(0, 0, w, h);
  if (arr.length < 2) return;
  let min = arr[0], max = arr[0];
  for (let i = 1; i < arr.length; i++) {
    if (arr[i] < min) min = arr[i];
    if (arr[i] > max) max = arr[i];
  }
  if (Math.abs(max - min) < 0.001) { max += 0.5; min -= 0.5; }
  ctx.beginPath();
  for (let i = 0; i < arr.length; i++) {
    const x = (i / (MAX_PTS - 1)) * (w - 8) + 4;
    const y = h - 6 - ((arr[i] - min) / (max - min)) * (h - 12);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.strokeStyle = color;
  ctx.lineWidth = 2;
  ctx.stroke();
}

function updateBar(barId, val, low, high, trig) {
  const el = document.getElementById(barId);
  if (!el) return;
  const span = high - low;
  let pct = 50;
  if (span > 0.0001) {
    const pad = span * 0.25;
    pct = ((val - (low - pad)) / (span + pad * 2)) * 100;
  }
  if (pct < 4) pct = 4;
  if (pct > 100) pct = 100;
  el.style.width = pct.toFixed(1) + '%';
  if (trig) el.classList.add('trig');
  else el.classList.remove('trig');
}

function updateBadge(id, enabled, trig, healthy, hasSample) {
  const el = document.getElementById(id);
  if (!healthy) {
    el.className = 'badge badge-danger';
    el.textContent = 'SENSOR FAULT';
  } else if (!hasSample) {
    el.className = 'badge badge-warn';
    el.textContent = 'WARMING UP';
  } else if (!enabled) {
    el.className = 'badge badge-muted';
    el.textContent = 'DISABLED';
  } else if (trig) {
    el.className = 'badge badge-danger';
    el.textContent = 'TRIGGERED';
  } else {
    el.className = 'badge badge-ok';
    el.textContent = 'NORMAL';
  }
}

async function pollStatus() {
  if (pollBusy) return;
  pollBusy = true;
  const ctrl = new AbortController();
  const timer = setTimeout(() => ctrl.abort(), 2500);
  try {
    const res = await fetch('/api/status', { signal: ctrl.signal, cache: 'no-store' });
    if (!res.ok) throw new Error('HTTP ' + res.status);
    const d = await res.json();

    document.getElementById('dotConn').className = 'dot ok';
    document.getElementById('txtConn').textContent = 'Link Active';

    const healthy   = Number(d.sensorHealthy) === 1;
    const hasSample = Number(d.hasSample) === 1;
    document.getElementById('dotBme').className = healthy ? 'dot ok' : 'dot err';
    document.getElementById('txtBme').textContent = healthy ? ('BME280 0x' + Number(d.i2cAddr).toString(16).toUpperCase()) : 'BME280 Fault';

    const unif = Number(d.relayMode) === 0;
    document.getElementById('pillMode').textContent = unif ? 'Mode: Unified (GPIO 25)' : 'Mode: Individual (25/26/27)';
    document.getElementById('pillUptime').textContent = 'Uptime: ' + formatUptime(d.uptimeMs);

    document.getElementById('dashTitleTemp').textContent = unif ? 'Temperature (GPIO 25 Unified)' : 'Temperature (GPIO 25)';
    document.getElementById('dashTitleHum').textContent  = unif ? 'Humidity (GPIO 25 Unified)'    : 'Humidity (GPIO 26)';
    document.getElementById('dashTitlePres').textContent = unif ? 'Pressure (GPIO 25 Unified)'    : 'Pressure (GPIO 27)';

    if (healthy && hasSample) {
      document.getElementById('valTempC').textContent = Number(d.tempC).toFixed(2);
      document.getElementById('valTempSub').textContent = Number(d.tempF).toFixed(2) + ' \u00B0F | Offset: ' + Number(d.tempOffset).toFixed(2) + ' \u00B0C';
      document.getElementById('valHum').textContent = Number(d.humPct).toFixed(2);
      document.getElementById('valHumSub').textContent = 'Dew Point: ' + Number(d.dewC).toFixed(2) + ' \u00B0C | Offset: ' + Number(d.humOffset).toFixed(2) + ' %';
      document.getElementById('valPres').textContent = Number(d.presHpa).toFixed(2);
      document.getElementById('valPresSub').textContent = 'Est. Altitude: ' + Number(d.altM).toFixed(1) + ' m | Offset: ' + Number(d.presOffset).toFixed(2) + ' hPa';

      histTemp.push(Number(d.tempC));   if (histTemp.length > MAX_PTS) histTemp.shift();
      histHum.push(Number(d.humPct));   if (histHum.length > MAX_PTS)  histHum.shift();
      histPres.push(Number(d.presHpa)); if (histPres.length > MAX_PTS) histPres.shift();

      drawSpark('sparkTemp', histTemp, '#38bdf8');
      drawSpark('sparkHum',  histHum,  '#10b981');
      drawSpark('sparkPres', histPres, '#f59e0b');
    } else {
      document.getElementById('valTempC').textContent = '--.-';
      document.getElementById('valHum').textContent   = '--.-';
      document.getElementById('valPres').textContent  = '----.-';
    }

    document.getElementById('metaTempLow').textContent  = 'Low: ' + Number(d.tempLow).toFixed(1) + ' \u00B0C';
    document.getElementById('metaTempHigh').textContent = 'High: ' + Number(d.tempHigh).toFixed(1) + ' \u00B0C';
    document.getElementById('metaTempCond').textContent = condName(d.tempCond);

    document.getElementById('metaHumLow').textContent  = 'Low: ' + Number(d.humLow).toFixed(1) + ' %';
    document.getElementById('metaHumHigh').textContent = 'High: ' + Number(d.humHigh).toFixed(1) + ' %';
    document.getElementById('metaHumCond').textContent = condName(d.humCond);

    document.getElementById('metaPresLow').textContent  = 'Low: ' + Number(d.presLow).toFixed(1) + ' hPa';
    document.getElementById('metaPresHigh').textContent = 'High: ' + Number(d.presHigh).toFixed(1) + ' hPa';
    document.getElementById('metaPresCond').textContent = condName(d.presCond);

    updateBadge('badgeTemp', Number(d.tempEn) === 1, Number(d.tempTrig) === 1, healthy, hasSample);
    updateBadge('badgeHum',  Number(d.humEn)  === 1, Number(d.humTrig)  === 1, healthy, hasSample);
    updateBadge('badgePres', Number(d.presEn) === 1, Number(d.presTrig) === 1, healthy, hasSample);

    if (healthy && hasSample) {
      updateBar('barTemp', Number(d.tempC),   Number(d.tempLow), Number(d.tempHigh), Number(d.tempTrig) === 1);
      updateBar('barHum',  Number(d.humPct),  Number(d.humLow),  Number(d.humHigh),  Number(d.humTrig)  === 1);
      updateBar('barPres', Number(d.presHpa), Number(d.presLow), Number(d.presHigh), Number(d.presTrig) === 1);
    }

    document.getElementById('badgeRelayMode').textContent = unif ? 'UNIFIED RELAY (GPIO 25)' : 'INDIVIDUAL RELAYS (25/26/27)';
    document.getElementById('lblR0Title').textContent = unif ? 'GPIO 25 (Unified)' : 'GPIO 25 (Temperature)';
    document.getElementById('metaR0Role').textContent = unif ? 'Trigger: Any Enabled Sensor' : 'Trigger: Temperature Only';
    document.getElementById('metaR1Role').textContent = unif ? 'Standby in Unified Mode' : 'Trigger: Humidity Only';
    document.getElementById('metaR2Role').textContent = unif ? 'Standby in Unified Mode' : 'Trigger: Pressure Only';

    const rStates = [Number(d.r0), Number(d.r1), Number(d.r2)];
    const rCounts = [Number(d.r0Cnt), Number(d.r1Cnt), Number(d.r2Cnt)];
    for (let i = 0; i < 3; i++) {
      const box = document.getElementById('boxR' + i);
      const bdg = document.getElementById('badgeR' + i);
      const st  = document.getElementById('metaR' + i + 'Stat');
      const on  = rStates[i] === 1;
      if (on) {
        box.classList.add('active');
        bdg.className = 'badge badge-ok';
        bdg.textContent = 'ENERGIZED (ON)';
      } else {
        box.classList.remove('active');
        bdg.className = 'badge badge-muted';
        bdg.textContent = (unif && i > 0) ? 'STANDBY (OFF)' : 'DE-ENERGIZED (OFF)';
      }
      st.textContent = 'Switches: ' + rCounts[i] + ' | Polarity: ' + (Number(d.activeLow) ? 'Active LOW' : 'Active HIGH');
    }

    document.getElementById('kvI2c').textContent = (healthy ? ('0x' + Number(d.i2cAddr).toString(16).toUpperCase()) : 'Disconnected') + ' | ' + d.sensorFaults + ' Faults';
    document.getElementById('kvReads').textContent = d.pollMs + ' ms | ' + d.sensorReads + ' Reads';
    document.getElementById('kvHeap').textContent = (Number(d.freeHeap) / 1024).toFixed(1) + ' KB / ' + (Number(d.minHeap) / 1024).toFixed(1) + ' KB';
    document.getElementById('kvNvsBtn').textContent = 'Seq #' + d.cfgSeq + ' (' + d.nvsWrites + ' writes) | ' + (Number(d.btnPressed) ? ('Hold ' + d.btnHoldMs + 'ms') : 'GPIO0 Idle');
    document.getElementById('kvStations').textContent = d.apClients + ' Connected';
    document.getElementById('kvBtnState').textContent = Number(d.btnPressed) ? ('Pressed (' + d.btnHoldMs + ' / 5000 ms)') : 'Released (0 ms)';
  } catch (e) {
    document.getElementById('dotConn').className = 'dot err';
    document.getElementById('txtConn').textContent = 'Reconnecting...';
  } finally {
    clearTimeout(timer);
    pollBusy = false;
  }
}

async function loadAllSettings(notify) {
  try {
    const res = await fetch('/api/settings', { cache: 'no-store' });
    if (!res.ok) throw new Error('Failed to load settings');
    const s = await res.json();

    document.getElementById('relayMode').value        = String(s.relayMode);
    document.getElementById('relayActiveLow').value   = String(s.relayActiveLow);
    document.getElementById('failsafeMode').value     = String(s.failsafeMode);
    document.getElementById('minRelaySwitchMs').value = s.minRelaySwitchMs;
    document.getElementById('sensorPollMs').value     = s.sensorPollMs;

    document.getElementById('tempEnable').checked     = Number(s.tempEnable) === 1;
    document.getElementById('tempCondition').value    = String(s.tempCondition);
    document.getElementById('tempHigh').value         = Number(s.tempHigh).toFixed(2);
    document.getElementById('tempLow').value          = Number(s.tempLow).toFixed(2);
    document.getElementById('tempHyst').value         = Number(s.tempHyst).toFixed(2);
    document.getElementById('tempOffset').value       = Number(s.tempOffset).toFixed(2);

    document.getElementById('humEnable').checked      = Number(s.humEnable) === 1;
    document.getElementById('humCondition').value     = String(s.humCondition);
    document.getElementById('humHigh').value          = Number(s.humHigh).toFixed(2);
    document.getElementById('humLow').value           = Number(s.humLow).toFixed(2);
    document.getElementById('humHyst').value          = Number(s.humHyst).toFixed(2);
    document.getElementById('humOffset').value        = Number(s.humOffset).toFixed(2);

    document.getElementById('presEnable').checked     = Number(s.presEnable) === 1;
    document.getElementById('presCondition').value    = String(s.presCondition);
    document.getElementById('presHigh').value         = Number(s.presHigh).toFixed(2);
    document.getElementById('presLow').value          = Number(s.presLow).toFixed(2);
    document.getElementById('presHyst').value         = Number(s.presHyst).toFixed(2);
    document.getElementById('presOffset').value       = Number(s.presOffset).toFixed(2);

    document.getElementById('wifiSsid').value         = s.wifiSsid || 'ESP32_BME280';
    document.getElementById('wifiPass').value         = s.wifiPass || '12345678';
    document.getElementById('wifiChannel').value      = String(s.wifiChannel);
    document.getElementById('wifiMaxConn').value      = String(s.wifiMaxConn);
    document.getElementById('wifiHidden').checked     = Number(s.wifiHidden) === 1;

    syncRelayModeLabels();
    if (notify) showToast('Configuration reloaded from controller', true);
  } catch (e) {
    if (notify) showToast('Unable to load configuration', false);
  }
}

async function saveSensorSettings(ev) {
  ev.preventDefault();
  const tHigh = parseFloat(document.getElementById('tempHigh').value);
  const tLow  = parseFloat(document.getElementById('tempLow').value);
  const hHigh = parseFloat(document.getElementById('humHigh').value);
  const hLow  = parseFloat(document.getElementById('humLow').value);
  const pHigh = parseFloat(document.getElementById('presHigh').value);
  const pLow  = parseFloat(document.getElementById('presLow').value);

  if ((tHigh - tLow) < 0.1) { showToast('Temperature High threshold must be at least 0.1 \u00B0C above Low threshold', false); return; }
  if ((hHigh - hLow) < 0.5) { showToast('Humidity High threshold must be at least 0.5 %RH above Low threshold', false); return; }
  if ((pHigh - pLow) < 0.5) { showToast('Pressure High threshold must be at least 0.5 hPa above Low threshold', false); return; }

  const payload = {
    relayMode:        parseInt(document.getElementById('relayMode').value, 10),
    relayActiveLow:   parseInt(document.getElementById('relayActiveLow').value, 10),
    failsafeMode:     parseInt(document.getElementById('failsafeMode').value, 10),
    minRelaySwitchMs: parseInt(document.getElementById('minRelaySwitchMs').value, 10),
    sensorPollMs:     parseInt(document.getElementById('sensorPollMs').value, 10),
    tempEnable:       document.getElementById('tempEnable').checked ? 1 : 0,
    tempCondition:    parseInt(document.getElementById('tempCondition').value, 10),
    tempHigh:         tHigh,
    tempLow:          tLow,
    tempHyst:         parseFloat(document.getElementById('tempHyst').value),
    tempOffset:       parseFloat(document.getElementById('tempOffset').value),
    humEnable:        document.getElementById('humEnable').checked ? 1 : 0,
    humCondition:     parseInt(document.getElementById('humCondition').value, 10),
    humHigh:          hHigh,
    humLow:           hLow,
    humHyst:          parseFloat(document.getElementById('humHyst').value),
    humOffset:        parseFloat(document.getElementById('humOffset').value),
    presEnable:       document.getElementById('presEnable').checked ? 1 : 0,
    presCondition:    parseInt(document.getElementById('presCondition').value, 10),
    presHigh:         pHigh,
    presLow:          pLow,
    presHyst:         parseFloat(document.getElementById('presHyst').value),
    presOffset:       parseFloat(document.getElementById('presOffset').value)
  };

  try {
    const res = await fetch('/api/settings', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
    const out = await res.json();
    if (res.ok && out.ok) {
      showToast(out.msg || 'Sensor and relay settings saved', true);
      pollStatus();
    } else {
      showToast(out.error || 'Validation error saving settings', false);
    }
  } catch (e) {
    showToast('Network error saving settings', false);
  }
}

async function saveWifiSettings(ev) {
  ev.preventDefault();
  const ssid = document.getElementById('wifiSsid').value.trim();
  const pass = document.getElementById('wifiPass').value;
  if (ssid.length < 1 || ssid.length > 32) {
    showToast('SSID must be between 1 and 32 characters', false);
    return;
  }
  if (pass.length < 8 || pass.length > 63) {
    showToast('Password must be between 8 and 63 characters', false);
    return;
  }
  const payload = {
    wifiSsid:    ssid,
    wifiPass:    pass,
    wifiChannel: parseInt(document.getElementById('wifiChannel').value, 10),
    wifiMaxConn: parseInt(document.getElementById('wifiMaxConn').value, 10),
    wifiHidden:  document.getElementById('wifiHidden').checked ? 1 : 0
  };
  try {
    const res = await fetch('/api/wifi', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(payload)
    });
    const out = await res.json();
    if (res.ok && out.ok) {
      showToast('Wi-Fi AP updated. Reconnect to "' + ssid + '" if changed.', true);
    } else {
      showToast(out.error || 'Error saving Wi-Fi settings', false);
    }
  } catch (e) {
    showToast('Network error applying Wi-Fi settings', false);
  }
}

function openModal(type) {
  const bd = document.getElementById('modalBackdrop');
  const title = document.getElementById('modalTitle');
  const body = document.getElementById('modalBody');
  const btn = document.getElementById('modalConfirmBtn');
  bd.classList.add('open');
  if (type === 'reset') {
    title.textContent = 'Factory Reset Controller';
    body.textContent = 'This will restore all BME280 thresholds, relay configurations, and Wi-Fi AP settings (SSID: ESP32_BME280 / Pass: 12345678) to factory defaults and reboot.';
    btn.textContent = 'Confirm Factory Reset';
    btn.onclick = async () => {
      closeModal();
      try {
        await fetch('/api/reset', { method: 'POST' });
        showToast('Factory reset initiated. Controller rebooting...', true);
      } catch (e) {
        showToast('Request failed', false);
      }
    };
  } else {
    title.textContent = 'Reboot Controller';
    body.textContent = 'The controller will safely de-energize relays and restart cleanly.';
    btn.textContent = 'Confirm Reboot';
    btn.onclick = async () => {
      closeModal();
      try {
        await fetch('/api/reboot', { method: 'POST' });
        showToast('Reboot initiated...', true);
      } catch (e) {
        showToast('Request failed', false);
      }
    };
  }
}

function closeModal() {
  document.getElementById('modalBackdrop').classList.remove('open');
}

window.addEventListener('DOMContentLoaded', () => {
  const path = window.location.pathname;
  if (path === '/settings') switchTab('page-sensor');
  else if (path === '/wifi') switchTab('page-wifi');
  loadAllSettings(false);
  pollStatus();
  setInterval(pollStatus, 1000);
});
</script>
</body>
</html>)rawliteral";

static uint64_t getMicros64() {
  return (uint64_t)esp_timer_get_time();
}

static uint64_t getMillis64() {
  return (uint64_t)(esp_timer_get_time() / 1000ULL);
}

static void spinWaitMicros64(uint64_t us) {
  const uint64_t startUs = getMicros64();
  while ((getMicros64() - startUs) < us) {
    NOP();
  }
}

static uint64_t computeConfigChecksum(const SystemConfig* cfg) {
  const uint8_t* bytes = reinterpret_cast<const uint8_t*>(cfg);
  const uint64_t len   = (uint64_t)(sizeof(SystemConfig) - sizeof(uint64_t));
  uint64_t hash        = 14695981039346656037ULL;
  for (uint64_t i = 0ULL; i < len; i++) {
    hash ^= (uint64_t)bytes[i];
    hash *= 1099511628211ULL;
  }
  return hash;
}

static uint64_t isDoubleFinite(double val) {
  return (!isnan(val) && !isinf(val)) ? 1ULL : 0ULL;
}

static double safeJsonDouble(double val) {
  return isDoubleFinite(val) ? val : 0.0;
}

static uint64_t validateConfig(const SystemConfig* cfg) {
  if (cfg->magic != CFG_MAGIC) return 0ULL;
  if (cfg->version != CFG_VERSION) return 0ULL;
  if (cfg->relayMode > RELAY_MODE_INDIVIDUAL) return 0ULL;
  if (cfg->relayActiveLow > 1ULL) return 0ULL;
  if (cfg->failsafeMode > FAILSAFE_FORCE_ON) return 0ULL;
  if (cfg->minRelaySwitchMs > 300000ULL) return 0ULL;
  if (cfg->sensorPollMs < 250ULL || cfg->sensorPollMs > 60000ULL) return 0ULL;

  if (cfg->tempEnable > 1ULL || cfg->tempCondition > COND_INSIDE_RANGE) return 0ULL;
  if (!isDoubleFinite(cfg->tempHigh) || !isDoubleFinite(cfg->tempLow) ||
      !isDoubleFinite(cfg->tempHyst) || !isDoubleFinite(cfg->tempOffset)) return 0ULL;
  if ((cfg->tempHigh - cfg->tempLow) < 0.1 || cfg->tempLow < -45.0 || cfg->tempHigh > 90.0) return 0ULL;
  if (cfg->tempHyst < 0.0 || cfg->tempHyst > 25.0) return 0ULL;
  if (cfg->tempOffset < -20.0 || cfg->tempOffset > 20.0) return 0ULL;

  if (cfg->humEnable > 1ULL || cfg->humCondition > COND_INSIDE_RANGE) return 0ULL;
  if (!isDoubleFinite(cfg->humHigh) || !isDoubleFinite(cfg->humLow) ||
      !isDoubleFinite(cfg->humHyst) || !isDoubleFinite(cfg->humOffset)) return 0ULL;
  if ((cfg->humHigh - cfg->humLow) < 0.5 || cfg->humLow < 0.0 || cfg->humHigh > 100.0) return 0ULL;
  if (cfg->humHyst < 0.0 || cfg->humHyst > 35.0) return 0ULL;
  if (cfg->humOffset < -30.0 || cfg->humOffset > 30.0) return 0ULL;

  if (cfg->presEnable > 1ULL || cfg->presCondition > COND_INSIDE_RANGE) return 0ULL;
  if (!isDoubleFinite(cfg->presHigh) || !isDoubleFinite(cfg->presLow) ||
      !isDoubleFinite(cfg->presHyst) || !isDoubleFinite(cfg->presOffset)) return 0ULL;
  if ((cfg->presHigh - cfg->presLow) < 0.5 || cfg->presLow < 300.0 || cfg->presHigh > 1100.0) return 0ULL;
  if (cfg->presHyst < 0.0 || cfg->presHyst > 60.0) return 0ULL;
  if (cfg->presOffset < -60.0 || cfg->presOffset > 60.0) return 0ULL;

  if (cfg->wifiChannel < 1ULL || cfg->wifiChannel > 13ULL) return 0ULL;
  if (cfg->wifiHidden > 1ULL) return 0ULL;
  if (cfg->wifiMaxConn < 1ULL || cfg->wifiMaxConn > 4ULL) return 0ULL;

  uint64_t ssidLen = 0ULL;
  while (ssidLen < 64ULL && cfg->wifiSsid[ssidLen] != '\0') ssidLen++;
  if (ssidLen < 1ULL || ssidLen > 32ULL) return 0ULL;

  uint64_t passLen = 0ULL;
  while (passLen < 64ULL && cfg->wifiPass[passLen] != '\0') passLen++;
  if (passLen < 8ULL || passLen > 63ULL) return 0ULL;

  if (computeConfigChecksum(cfg) != cfg->checksum) return 0ULL;
  return 1ULL;
}

static void loadDefaultConfig(SystemConfig* cfg) {
  memset(cfg, 0, sizeof(SystemConfig));
  cfg->magic            = CFG_MAGIC;
  cfg->version          = CFG_VERSION;
  cfg->sequence         = 1ULL;

  cfg->relayMode        = RELAY_MODE_UNIFIED;
  cfg->relayActiveLow   = 0ULL;
  cfg->failsafeMode     = FAILSAFE_FORCE_OFF;
  cfg->minRelaySwitchMs = 1000ULL;
  cfg->sensorPollMs     = 1000ULL;

  cfg->tempEnable       = 1ULL;
  cfg->tempCondition    = COND_ABOVE_HIGH;
  cfg->tempHigh         = 30.0;
  cfg->tempLow          = 15.0;
  cfg->tempHyst         = 0.5;
  cfg->tempOffset       = 0.0;

  cfg->humEnable        = 1ULL;
  cfg->humCondition     = COND_ABOVE_HIGH;
  cfg->humHigh          = 70.0;
  cfg->humLow           = 30.0;
  cfg->humHyst          = 2.0;
  cfg->humOffset        = 0.0;

  cfg->presEnable       = 1ULL;
  cfg->presCondition    = COND_BELOW_LOW;
  cfg->presHigh         = 1025.0;
  cfg->presLow          = 995.0;
  cfg->presHyst         = 1.0;
  cfg->presOffset       = 0.0;

  cfg->wifiChannel      = 1ULL;
  cfg->wifiHidden       = 0ULL;
  cfg->wifiMaxConn      = 4ULL;

  strncpy(cfg->wifiSsid, "ESP32_BME280", sizeof(cfg->wifiSsid) - 1U);
  strncpy(cfg->wifiPass, "12345678", sizeof(cfg->wifiPass) - 1U);

  cfg->checksum         = computeConfigChecksum(cfg);
}

static uint64_t configsEqualPayload(const SystemConfig* a, const SystemConfig* b) {
  SystemConfig ta = *a;
  SystemConfig tb = *b;
  ta.sequence = 0ULL;
  tb.sequence = 0ULL;
  ta.checksum = 0ULL;
  tb.checksum = 0ULL;
  return (memcmp(&ta, &tb, sizeof(SystemConfig)) == 0) ? 1ULL : 0ULL;
}

static void writeConfigToNvsNow() {
  g_config.sequence += 1ULL;
  g_config.checksum  = computeConfigChecksum(&g_config);

  if (g_prefs.begin("bme280_nvs", false)) {
    SystemConfig verifyBuf;
    memset(&verifyBuf, 0, sizeof(SystemConfig));

    uint64_t wroteA = (uint64_t)g_prefs.putBytes("bank_a", &g_config, sizeof(SystemConfig));
    uint64_t okA    = 0ULL;
    if (wroteA == (uint64_t)sizeof(SystemConfig) &&
        g_prefs.getBytesLength("bank_a") == sizeof(SystemConfig)) {
      g_prefs.getBytes("bank_a", &verifyBuf, sizeof(SystemConfig));
      okA = validateConfig(&verifyBuf);
    }

    if (okA) {
      g_prefs.putBytes("bank_b", &g_config, sizeof(SystemConfig));
    }
    g_prefs.end();
  }

  g_state.nvsDirty       = 0ULL;
  g_state.nvsLastWriteMs = getMillis64();
  g_state.nvsWriteCount += 1ULL;
  g_state.nvsBankHealth  = 1ULL;
}

static void stageConfigSave(const SystemConfig* newCfg, uint64_t immediate) {
  if (configsEqualPayload(&g_config, newCfg)) {
    return;
  }
  uint64_t currentSeq = g_config.sequence;
  g_config            = *newCfg;
  g_config.sequence   = currentSeq;
  g_config.checksum   = computeConfigChecksum(&g_config);

  if (immediate) {
    writeConfigToNvsNow();
  } else {
    g_state.nvsDirty   = 1ULL;
    g_state.nvsStageMs = getMillis64();
  }
}

static void initStorageAndConfig() {
  SystemConfig bankA;
  SystemConfig bankB;
  memset(&bankA, 0, sizeof(SystemConfig));
  memset(&bankB, 0, sizeof(SystemConfig));

  uint64_t validA = 0ULL;
  uint64_t validB = 0ULL;

  if (g_prefs.begin("bme280_nvs", false)) {
    if (g_prefs.getBytesLength("bank_a") == sizeof(SystemConfig)) {
      g_prefs.getBytes("bank_a", &bankA, sizeof(SystemConfig));
      validA = validateConfig(&bankA);
    }
    if (g_prefs.getBytesLength("bank_b") == sizeof(SystemConfig)) {
      g_prefs.getBytes("bank_b", &bankB, sizeof(SystemConfig));
      validB = validateConfig(&bankB);
    }

    if (validA && validB) {
      if (bankA.sequence >= bankB.sequence) {
        g_config = bankA;
        if (!configsEqualPayload(&bankA, &bankB) || bankA.sequence != bankB.sequence) {
          g_prefs.putBytes("bank_b", &g_config, sizeof(SystemConfig));
        }
      } else {
        g_config = bankB;
        g_prefs.putBytes("bank_a", &g_config, sizeof(SystemConfig));
      }
      g_state.nvsBankHealth = 1ULL;
    } else if (validA) {
      g_config = bankA;
      g_prefs.putBytes("bank_b", &g_config, sizeof(SystemConfig));
      g_state.nvsBankHealth = 1ULL;
    } else if (validB) {
      g_config = bankB;
      g_prefs.putBytes("bank_a", &g_config, sizeof(SystemConfig));
      g_state.nvsBankHealth = 1ULL;
    } else {
      loadDefaultConfig(&g_config);
      g_prefs.putBytes("bank_a", &g_config, sizeof(SystemConfig));
      g_prefs.putBytes("bank_b", &g_config, sizeof(SystemConfig));
      g_state.nvsBankHealth = 1ULL;
    }
    g_prefs.end();
  } else {
    loadDefaultConfig(&g_config);
  }
}

static void writePhysicalRelayPin(uint64_t relayIdx, uint64_t logicalOn) {
  uint64_t pin = PIN_RELAY_TEMP_UNIF;
  if (relayIdx == 1ULL) pin = PIN_RELAY_HUMIDITY;
  else if (relayIdx == 2ULL) pin = PIN_RELAY_PRESSURE;

  uint64_t pinHigh = logicalOn ? 1ULL : 0ULL;
  if (g_config.relayActiveLow) {
    pinHigh = logicalOn ? 0ULL : 1ULL;
  }
  digitalWrite((uint8_t)pin, pinHigh ? HIGH : LOW);
}

static void initRelayHardware() {
  uint64_t offLevel = g_config.relayActiveLow ? HIGH : LOW;
  digitalWrite((uint8_t)PIN_RELAY_TEMP_UNIF, (uint8_t)offLevel);
  digitalWrite((uint8_t)PIN_RELAY_HUMIDITY,  (uint8_t)offLevel);
  digitalWrite((uint8_t)PIN_RELAY_PRESSURE,  (uint8_t)offLevel);

  pinMode((uint8_t)PIN_RELAY_TEMP_UNIF, OUTPUT);
  pinMode((uint8_t)PIN_RELAY_HUMIDITY,  OUTPUT);
  pinMode((uint8_t)PIN_RELAY_PRESSURE,  OUTPUT);

  for (uint64_t i = 0ULL; i < 3ULL; i++) {
    g_state.relayLogicalState[i] = 0ULL;
    g_state.relayDesiredState[i] = 0ULL;
    g_state.relayLastSwitchMs[i] = 0ULL;
    g_state.relaySwitchCount[i]  = 0ULL;
    writePhysicalRelayPin(i, 0ULL);
  }
}

static void recomputeCalibratedMetricsFromRaw() {
  if (!g_state.hasValidSample) return;

  double calT = g_state.rawTempC + g_config.tempOffset;
  double calH = g_state.rawHumPct + g_config.humOffset;
  if (calH < 0.0) calH = 0.0;
  if (calH > 100.0) calH = 100.0;
  double calP = g_state.rawPresHpa + g_config.presOffset;
  if (calP < 100.0) calP = 100.0;

  g_state.temperatureC = calT;
  g_state.temperatureF = (calT * 1.8) + 32.0;
  g_state.humidityPct  = calH;
  g_state.pressureHpa  = calP;

  double safeH = (calH < 1.0) ? 1.0 : calH;
  double a     = 17.625;
  double b     = 243.04;
  double alpha = log(safeH / 100.0) + ((a * calT) / (b + calT));
  g_state.dewPointC = (b * alpha) / (a - alpha);
  g_state.altitudeM = 44330.0 * (1.0 - pow(calP / 1013.25, 0.1903));
}

static uint64_t evaluateChannelTrigger(
    uint64_t enabled,
    uint64_t condition,
    double   val,
    double   low,
    double   high,
    double   hyst,
    uint64_t currentState) {
  if (!enabled || !isDoubleFinite(val)) {
    return 0ULL;
  }

  double span = high - low;
  if (span < 0.01) span = 0.01;
  double cleanHyst = (hyst < 0.0) ? 0.0 : hyst;

  if (condition == COND_ABOVE_HIGH) {
    if (!currentState) {
      return (val >= high) ? 1ULL : 0ULL;
    } else {
      return (val <= (high - cleanHyst)) ? 0ULL : 1ULL;
    }
  } else if (condition == COND_BELOW_LOW) {
    if (!currentState) {
      return (val <= low) ? 1ULL : 0ULL;
    } else {
      return (val >= (low + cleanHyst)) ? 0ULL : 1ULL;
    }
  } else if (condition == COND_OUTSIDE_RANGE) {
    if (!currentState) {
      return (val <= low || val >= high) ? 1ULL : 0ULL;
    } else {
      double maxWindowHyst = span * 0.45;
      double effHyst       = (cleanHyst > maxWindowHyst) ? maxWindowHyst : cleanHyst;
      double relLow        = low + effHyst;
      double relHigh       = high - effHyst;
      return (val >= relLow && val <= relHigh) ? 0ULL : 1ULL;
    }
  } else if (condition == COND_INSIDE_RANGE) {
    if (!currentState) {
      return (val >= low && val <= high) ? 1ULL : 0ULL;
    } else {
      return (val <= (low - cleanHyst) || val >= (high + cleanHyst)) ? 0ULL : 1ULL;
    }
  }
  return 0ULL;
}

static void updateRelaysNonBlocking(uint64_t forceImmediate) {
  const uint64_t nowMs = getMillis64();

  if (g_state.sensorHealthy && g_state.hasValidSample) {
    g_state.tempTriggered = evaluateChannelTrigger(
        g_config.tempEnable,
        g_config.tempCondition,
        g_state.temperatureC,
        g_config.tempLow,
        g_config.tempHigh,
        g_config.tempHyst,
        g_state.tempTriggered);

    g_state.humTriggered = evaluateChannelTrigger(
        g_config.humEnable,
        g_config.humCondition,
        g_state.humidityPct,
        g_config.humLow,
        g_config.humHigh,
        g_config.humHyst,
        g_state.humTriggered);

    g_state.presTriggered = evaluateChannelTrigger(
        g_config.presEnable,
        g_config.presCondition,
        g_state.pressureHpa,
        g_config.presLow,
        g_config.presHigh,
        g_config.presHyst,
        g_state.presTriggered);

    if (g_config.relayMode == RELAY_MODE_UNIFIED) {
      uint64_t anyTrig = (g_state.tempTriggered || g_state.humTriggered || g_state.presTriggered) ? 1ULL : 0ULL;
      g_state.relayDesiredState[0] = anyTrig;
      g_state.relayDesiredState[1] = 0ULL;
      g_state.relayDesiredState[2] = 0ULL;
    } else {
      g_state.relayDesiredState[0] = g_state.tempTriggered;
      g_state.relayDesiredState[1] = g_state.humTriggered;
      g_state.relayDesiredState[2] = g_state.presTriggered;
    }
  } else if (!g_state.sensorHealthy) {
    g_state.tempTriggered = 0ULL;
    g_state.humTriggered  = 0ULL;
    g_state.presTriggered = 0ULL;

    if (g_config.failsafeMode == FAILSAFE_FORCE_OFF) {
      g_state.relayDesiredState[0] = 0ULL;
      g_state.relayDesiredState[1] = 0ULL;
      g_state.relayDesiredState[2] = 0ULL;
    } else if (g_config.failsafeMode == FAILSAFE_FORCE_ON) {
      if (g_config.relayMode == RELAY_MODE_UNIFIED) {
        uint64_t anyEnabled = (g_config.tempEnable || g_config.humEnable || g_config.presEnable) ? 1ULL : 0ULL;
        g_state.relayDesiredState[0] = anyEnabled;
        g_state.relayDesiredState[1] = 0ULL;
        g_state.relayDesiredState[2] = 0ULL;
      } else {
        g_state.relayDesiredState[0] = g_config.tempEnable ? 1ULL : 0ULL;
        g_state.relayDesiredState[1] = g_config.humEnable  ? 1ULL : 0ULL;
        g_state.relayDesiredState[2] = g_config.presEnable ? 1ULL : 0ULL;
      }
    } else {
      if (g_config.relayMode == RELAY_MODE_UNIFIED) {
        g_state.relayDesiredState[1] = 0ULL;
        g_state.relayDesiredState[2] = 0ULL;
      }
    }
  }

  for (uint64_t i = 0ULL; i < 3ULL; i++) {
    uint64_t forceStandbyOff = (g_config.relayMode == RELAY_MODE_UNIFIED && i > 0ULL) ? 1ULL : 0ULL;
    if (g_state.relayDesiredState[i] != g_state.relayLogicalState[i]) {
      uint64_t elapsed = nowMs - g_state.relayLastSwitchMs[i];
      if (forceImmediate || forceStandbyOff || g_state.relayLastSwitchMs[i] == 0ULL || elapsed >= g_config.minRelaySwitchMs) {
        g_state.relayLogicalState[i] = g_state.relayDesiredState[i];
        g_state.relayLastSwitchMs[i] = nowMs;
        g_state.relaySwitchCount[i] += 1ULL;
        writePhysicalRelayPin(i, g_state.relayLogicalState[i]);
      }
    } else if (forceImmediate) {
      writePhysicalRelayPin(i, g_state.relayLogicalState[i]);
    }
  }
}

static void initOrRecoverI2CBus(uint64_t forceBusClear) {
  if (forceBusClear) {
    if (g_state.i2cInitialized) {
      Wire.end();
      g_state.i2cInitialized = 0ULL;
    }
    pinMode((uint8_t)PIN_I2C_SDA, INPUT_PULLUP);
    pinMode((uint8_t)PIN_I2C_SCL, OUTPUT);
    digitalWrite((uint8_t)PIN_I2C_SCL, HIGH);
    spinWaitMicros64(10ULL);
    for (uint64_t i = 0ULL; i < 9ULL; i++) {
      if (digitalRead((uint8_t)PIN_I2C_SDA) == HIGH) {
        break;
      }
      digitalWrite((uint8_t)PIN_I2C_SCL, LOW);
      spinWaitMicros64(10ULL);
      digitalWrite((uint8_t)PIN_I2C_SCL, HIGH);
      spinWaitMicros64(10ULL);
    }
  }
  if (!g_state.i2cInitialized) {
    Wire.begin((int)PIN_I2C_SDA, (int)PIN_I2C_SCL, 100000U);
    Wire.setTimeOut(20U);
    g_state.i2cInitialized = 1ULL;
  }
}

static uint64_t probeI2CAddress(uint64_t addr) {
  Wire.beginTransmission((uint8_t)addr);
  uint64_t err = (uint64_t)Wire.endTransmission();
  return (err == 0ULL) ? 1ULL : 0ULL;
}

static uint64_t tryInitBME280() {
  initOrRecoverI2CBus(0ULL);
  const uint64_t candidates[2] = { 0x76ULL, 0x77ULL };
  for (uint64_t i = 0ULL; i < 2ULL; i++) {
    uint64_t addr = candidates[i];
    if (probeI2CAddress(addr)) {
      if (g_bme.begin((uint8_t)addr, &Wire)) {
        g_bme.setSampling(
            Adafruit_BME280::MODE_NORMAL,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::FILTER_X4,
            Adafruit_BME280::STANDBY_MS_125);
        g_state.activeI2CAddress        = addr;
        g_state.sensorHealthy           = 1ULL;
        g_state.consecutiveSensorErrors = 0ULL;
        g_state.sensorWarmupUntilMs     = getMillis64() + BME_WARMUP_DELAY_MS;
        return 1ULL;
      }
    }
  }
  g_state.sensorHealthy  = 0ULL;
  g_state.hasValidSample = 0ULL;
  return 0ULL;
}

static void pollBME280NonBlocking() {
  const uint64_t nowMs = getMillis64();

  if (!g_state.sensorHealthy) {
    if (g_state.lastSensorInitTryMs == 0ULL || (nowMs - g_state.lastSensorInitTryMs) >= BME_RECOVERY_RETRY_MS) {
      g_state.lastSensorInitTryMs = nowMs;
      if (g_state.totalSensorFaults > 0ULL) {
        initOrRecoverI2CBus(1ULL);
      }
      if (!tryInitBME280()) {
        updateRelaysNonBlocking(0ULL);
        return;
      }
    } else {
      return;
    }
  }

  if (nowMs < g_state.sensorWarmupUntilMs) {
    return;
  }

  if (g_state.lastSensorPollMs != 0ULL && (nowMs - g_state.lastSensorPollMs) < g_config.sensorPollMs) {
    return;
  }
  g_state.lastSensorPollMs = nowMs;

  if (!probeI2CAddress(g_state.activeI2CAddress)) {
    g_state.consecutiveSensorErrors += 1ULL;
    g_state.totalSensorFaults       += 1ULL;
    if (g_state.consecutiveSensorErrors >= 3ULL) {
      g_state.sensorHealthy  = 0ULL;
      g_state.hasValidSample = 0ULL;
    }
    updateRelaysNonBlocking(0ULL);
    return;
  }

  double rawT = (double)g_bme.readTemperature();
  double rawP = (double)g_bme.readPressure() / 100.0;
  double rawH = (double)g_bme.readHumidity();

  uint64_t validRead = 1ULL;
  if (!isDoubleFinite(rawT) || rawT < -45.0 || rawT > 90.0) validRead = 0ULL;
  if (!isDoubleFinite(rawH) || rawH < 0.0   || rawH > 100.0) validRead = 0ULL;
  if (!isDoubleFinite(rawP) || rawP < 300.0 || rawP > 1150.0) validRead = 0ULL;

  if (!validRead) {
    g_state.consecutiveSensorErrors += 1ULL;
    g_state.totalSensorFaults       += 1ULL;
    if (g_state.consecutiveSensorErrors >= 3ULL) {
      g_state.sensorHealthy  = 0ULL;
      g_state.hasValidSample = 0ULL;
    }
    updateRelaysNonBlocking(0ULL);
    return;
  }

  g_state.consecutiveSensorErrors = 0ULL;
  g_state.sensorHealthy           = 1ULL;
  g_state.hasValidSample          = 1ULL;
  g_state.totalSensorReads       += 1ULL;

  g_state.rawTempC   = rawT;
  g_state.rawHumPct  = rawH;
  g_state.rawPresHpa = rawP;

  recomputeCalibratedMetricsFromRaw();
  updateRelaysNonBlocking(0ULL);
}

static void startOrReloadWiFiAP() {
  WiFi.persistent(false);
  WiFi.mode(WIFI_AP);
  esp_wifi_set_ps(WIFI_PS_NONE);
  WiFi.softAPConfig(g_apIP, g_apIP, g_netMask);
  WiFi.softAP(
      g_config.wifiSsid,
      g_config.wifiPass,
      (int)g_config.wifiChannel,
      (int)g_config.wifiHidden,
      (int)g_config.wifiMaxConn);
  g_dnsServer.stop();
  g_dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
  g_dnsServer.start((uint16_t)DNS_PORT, "*", g_apIP);
}

static void executeFactoryResetAndScheduleReboot() {
  for (uint64_t i = 0ULL; i < 3ULL; i++) {
    g_state.relayLogicalState[i] = 0ULL;
    g_state.relayDesiredState[i] = 0ULL;
    writePhysicalRelayPin(i, 0ULL);
  }
  SystemConfig defaults;
  loadDefaultConfig(&defaults);
  defaults.sequence = g_config.sequence + 1ULL;
  g_config          = defaults;
  writeConfigToNvsNow();

  g_state.pendingReboot = 1ULL;
  g_state.rebootAtMs    = getMillis64() + ACTION_DELAY_MS;
}

static void pollFactoryResetButtonNonBlocking() {
  const uint64_t nowMs   = getMillis64();
  uint64_t       pressed = (digitalRead((uint8_t)PIN_FACTORY_RESET_BTN) == LOW) ? 1ULL : 0ULL;

  if (pressed != g_state.btnRawState) {
    g_state.btnRawState     = pressed;
    g_state.btnLastChangeMs = nowMs;
  }

  if ((nowMs - g_state.btnLastChangeMs) >= BTN_DEBOUNCE_MS) {
    if (pressed != g_state.btnStableState) {
      g_state.btnStableState = pressed;
      if (pressed) {
        g_state.btnPressStartMs   = nowMs;
        g_state.btnResetTriggered = 0ULL;
      } else {
        g_state.btnPressStartMs   = 0ULL;
        g_state.btnResetTriggered = 0ULL;
      }
    }
  }

  if (g_state.btnStableState && !g_state.btnResetTriggered) {
    if ((nowMs - g_state.btnPressStartMs) >= FACTORY_RESET_HOLD_MS) {
      g_state.btnResetTriggered = 1ULL;
      executeFactoryResetAndScheduleReboot();
    }
  }
}

static void sendNoCacheHeaders() {
  g_webServer.sendHeader("Cache-Control", "no-store, no-cache, must-revalidate, max-age=0");
  g_webServer.sendHeader("Pragma", "no-cache");
}

static uint64_t isCaptiveRedirectRequired() {
  const String& host = g_webServer.hostHeader();
  const char*   h    = host.c_str();
  if (h == nullptr || h[0] == '\0') return 0ULL;
  if (strcmp(h, "192.168.4.1") == 0 || strcmp(h, "192.168.4.1:80") == 0) {
    return 0ULL;
  }
  return 1ULL;
}

static void sendCaptivePortalRedirect() {
  sendNoCacheHeaders();
  g_webServer.sendHeader("Location", "http://192.168.4.1/", true);
  g_webServer.send(302, "text/plain", "");
}

static void handleRootPage() {
  if (isCaptiveRedirectRequired()) {
    sendCaptivePortalRedirect();
    return;
  }
  sendNoCacheHeaders();
  g_webServer.send_P(200, "text/html; charset=utf-8", INDEX_HTML);
}

static void handleFavicon() {
  sendNoCacheHeaders();
  g_webServer.send(204, "image/x-icon", "");
}

static void handleApiStatus() {
  const uint64_t nowMs     = getMillis64();
  uint64_t       btnHoldMs = 0ULL;
  if (g_state.btnStableState && g_state.btnPressStartMs > 0ULL && nowMs >= g_state.btnPressStartMs) {
    btnHoldMs = nowMs - g_state.btnPressStartMs;
  }
  uint64_t freeHeap  = (uint64_t)ESP.getFreeHeap();
  uint64_t minHeap   = (uint64_t)ESP.getMinFreeHeap();
  uint64_t apClients = (uint64_t)WiFi.softAPgetStationNum();

  snprintf(
      g_jsonBuf,
      sizeof(g_jsonBuf),
      "{"
      "\"uptimeMs\":%llu,"
      "\"sensorHealthy\":%llu,"
      "\"hasSample\":%llu,"
      "\"i2cAddr\":%llu,"
      "\"sensorReads\":%llu,"
      "\"sensorFaults\":%llu,"
      "\"pollMs\":%llu,"
      "\"tempC\":%.2f,"
      "\"tempF\":%.2f,"
      "\"humPct\":%.2f,"
      "\"presHpa\":%.2f,"
      "\"dewC\":%.2f,"
      "\"altM\":%.2f,"
      "\"tempEn\":%llu,"
      "\"tempCond\":%llu,"
      "\"tempHigh\":%.2f,"
      "\"tempLow\":%.2f,"
      "\"tempOffset\":%.2f,"
      "\"tempTrig\":%llu,"
      "\"humEn\":%llu,"
      "\"humCond\":%llu,"
      "\"humHigh\":%.2f,"
      "\"humLow\":%.2f,"
      "\"humOffset\":%.2f,"
      "\"humTrig\":%llu,"
      "\"presEn\":%llu,"
      "\"presCond\":%llu,"
      "\"presHigh\":%.2f,"
      "\"presLow\":%.2f,"
      "\"presOffset\":%.2f,"
      "\"presTrig\":%llu,"
      "\"relayMode\":%llu,"
      "\"activeLow\":%llu,"
      "\"r0\":%llu,"
      "\"r1\":%llu,"
      "\"r2\":%llu,"
      "\"r0Cnt\":%llu,"
      "\"r1Cnt\":%llu,"
      "\"r2Cnt\":%llu,"
      "\"freeHeap\":%llu,"
      "\"minHeap\":%llu,"
      "\"cfgSeq\":%llu,"
      "\"nvsWrites\":%llu,"
      "\"btnPressed\":%llu,"
      "\"btnHoldMs\":%llu,"
      "\"apClients\":%llu"
      "}",
      (unsigned long long)nowMs,
      (unsigned long long)g_state.sensorHealthy,
      (unsigned long long)g_state.hasValidSample,
      (unsigned long long)g_state.activeI2CAddress,
      (unsigned long long)g_state.totalSensorReads,
      (unsigned long long)g_state.totalSensorFaults,
      (unsigned long long)g_config.sensorPollMs,
      safeJsonDouble(g_state.temperatureC),
      safeJsonDouble(g_state.temperatureF),
      safeJsonDouble(g_state.humidityPct),
      safeJsonDouble(g_state.pressureHpa),
      safeJsonDouble(g_state.dewPointC),
      safeJsonDouble(g_state.altitudeM),
      (unsigned long long)g_config.tempEnable,
      (unsigned long long)g_config.tempCondition,
      safeJsonDouble(g_config.tempHigh),
      safeJsonDouble(g_config.tempLow),
      safeJsonDouble(g_config.tempOffset),
      (unsigned long long)g_state.tempTriggered,
      (unsigned long long)g_config.humEnable,
      (unsigned long long)g_config.humCondition,
      safeJsonDouble(g_config.humHigh),
      safeJsonDouble(g_config.humLow),
      safeJsonDouble(g_config.humOffset),
      (unsigned long long)g_state.humTriggered,
      (unsigned long long)g_config.presEnable,
      (unsigned long long)g_config.presCondition,
      safeJsonDouble(g_config.presHigh),
      safeJsonDouble(g_config.presLow),
      safeJsonDouble(g_config.presOffset),
      (unsigned long long)g_state.presTriggered,
      (unsigned long long)g_config.relayMode,
      (unsigned long long)g_config.relayActiveLow,
      (unsigned long long)g_state.relayLogicalState[0],
      (unsigned long long)g_state.relayLogicalState[1],
      (unsigned long long)g_state.relayLogicalState[2],
      (unsigned long long)g_state.relaySwitchCount[0],
      (unsigned long long)g_state.relaySwitchCount[1],
      (unsigned long long)g_state.relaySwitchCount[2],
      (unsigned long long)freeHeap,
      (unsigned long long)minHeap,
      (unsigned long long)g_config.sequence,
      (unsigned long long)g_state.nvsWriteCount,
      (unsigned long long)g_state.btnStableState,
      (unsigned long long)btnHoldMs,
      (unsigned long long)apClients);

  sendNoCacheHeaders();
  g_webServer.send(200, "application/json", g_jsonBuf);
}

static void escapeJsonString(const char* src, char* dst, uint64_t dstSize) {
  uint64_t j = 0ULL;
  for (uint64_t i = 0ULL; src[i] != '\0' && (j + 2ULL) < dstSize; i++) {
    char c = src[i];
    if (c == '\"' || c == '\\') {
      dst[j++] = '\\';
    }
    dst[j++] = c;
  }
  dst[j] = '\0';
}

static void handleApiGetSettings() {
  char escapedSsid[132];
  char escapedPass[132];
  escapeJsonString(g_config.wifiSsid, escapedSsid, sizeof(escapedSsid));
  escapeJsonString(g_config.wifiPass, escapedPass, sizeof(escapedPass));

  snprintf(
      g_jsonBuf,
      sizeof(g_jsonBuf),
      "{"
      "\"relayMode\":%llu,"
      "\"relayActiveLow\":%llu,"
      "\"failsafeMode\":%llu,"
      "\"minRelaySwitchMs\":%llu,"
      "\"sensorPollMs\":%llu,"
      "\"tempEnable\":%llu,"
      "\"tempCondition\":%llu,"
      "\"tempHigh\":%.2f,"
      "\"tempLow\":%.2f,"
      "\"tempHyst\":%.2f,"
      "\"tempOffset\":%.2f,"
      "\"humEnable\":%llu,"
      "\"humCondition\":%llu,"
      "\"humHigh\":%.2f,"
      "\"humLow\":%.2f,"
      "\"humHyst\":%.2f,"
      "\"humOffset\":%.2f,"
      "\"presEnable\":%llu,"
      "\"presCondition\":%llu,"
      "\"presHigh\":%.2f,"
      "\"presLow\":%.2f,"
      "\"presHyst\":%.2f,"
      "\"presOffset\":%.2f,"
      "\"wifiChannel\":%llu,"
      "\"wifiHidden\":%llu,"
      "\"wifiMaxConn\":%llu,"
      "\"wifiSsid\":\"%s\","
      "\"wifiPass\":\"%s\""
      "}",
      (unsigned long long)g_config.relayMode,
      (unsigned long long)g_config.relayActiveLow,
      (unsigned long long)g_config.failsafeMode,
      (unsigned long long)g_config.minRelaySwitchMs,
      (unsigned long long)g_config.sensorPollMs,
      (unsigned long long)g_config.tempEnable,
      (unsigned long long)g_config.tempCondition,
      safeJsonDouble(g_config.tempHigh),
      safeJsonDouble(g_config.tempLow),
      safeJsonDouble(g_config.tempHyst),
      safeJsonDouble(g_config.tempOffset),
      (unsigned long long)g_config.humEnable,
      (unsigned long long)g_config.humCondition,
      safeJsonDouble(g_config.humHigh),
      safeJsonDouble(g_config.humLow),
      safeJsonDouble(g_config.humHyst),
      safeJsonDouble(g_config.humOffset),
      (unsigned long long)g_config.presEnable,
      (unsigned long long)g_config.presCondition,
      safeJsonDouble(g_config.presHigh),
      safeJsonDouble(g_config.presLow),
      safeJsonDouble(g_config.presHyst),
      safeJsonDouble(g_config.presOffset),
      (unsigned long long)g_config.wifiChannel,
      (unsigned long long)g_config.wifiHidden,
      (unsigned long long)g_config.wifiMaxConn,
      escapedSsid,
      escapedPass);

  sendNoCacheHeaders();
  g_webServer.send(200, "application/json", g_jsonBuf);
}

static void handleApiPostSettings() {
  if (!g_webServer.hasArg("plain")) {
    sendNoCacheHeaders();
    g_webServer.send(400, "application/json", "{\"ok\":0,\"error\":\"Missing JSON body\"}");
    return;
  }

#if ARDUINOJSON_VERSION_MAJOR >= 7
  JsonDocument doc;
#else
  StaticJsonDocument<1024> doc;
#endif
  DeserializationError err = deserializeJson(doc, g_webServer.arg("plain"));
  if (err) {
    sendNoCacheHeaders();
    g_webServer.send(400, "application/json", "{\"ok\":0,\"error\":\"Malformed JSON payload\"}");
    return;
  }

  int64_t relayMode        = doc["relayMode"]        | (int64_t)g_config.relayMode;
  int64_t relayActiveLow   = doc["relayActiveLow"]   | (int64_t)g_config.relayActiveLow;
  int64_t failsafeMode     = doc["failsafeMode"]     | (int64_t)g_config.failsafeMode;
  int64_t minRelaySwitchMs = doc["minRelaySwitchMs"] | (int64_t)g_config.minRelaySwitchMs;
  int64_t sensorPollMs     = doc["sensorPollMs"]     | (int64_t)g_config.sensorPollMs;

  int64_t tempEnable       = doc["tempEnable"]       | (int64_t)g_config.tempEnable;
  int64_t tempCondition    = doc["tempCondition"]    | (int64_t)g_config.tempCondition;
  int64_t humEnable        = doc["humEnable"]        | (int64_t)g_config.humEnable;
  int64_t humCondition     = doc["humCondition"]     | (int64_t)g_config.humCondition;
  int64_t presEnable       = doc["presEnable"]       | (int64_t)g_config.presEnable;
  int64_t presCondition    = doc["presCondition"]    | (int64_t)g_config.presCondition;

  if (relayMode < 0LL || relayActiveLow < 0LL || failsafeMode < 0LL ||
      minRelaySwitchMs < 0LL || sensorPollMs < 250LL ||
      tempEnable < 0LL || tempCondition < 0LL ||
      humEnable < 0LL  || humCondition < 0LL  ||
      presEnable < 0LL || presCondition < 0LL) {
    sendNoCacheHeaders();
    g_webServer.send(400, "application/json", "{\"ok\":0,\"error\":\"Negative or invalid control parameters\"}");
    return;
  }

  SystemConfig cand     = g_config;
  cand.relayMode        = (uint64_t)relayMode;
  cand.relayActiveLow   = (uint64_t)relayActiveLow;
  cand.failsafeMode     = (uint64_t)failsafeMode;
  cand.minRelaySwitchMs = (uint64_t)minRelaySwitchMs;
  cand.sensorPollMs     = (uint64_t)sensorPollMs;

  cand.tempEnable       = (uint64_t)tempEnable;
  cand.tempCondition    = (uint64_t)tempCondition;
  cand.tempHigh         = (double)(doc["tempHigh"]   | cand.tempHigh);
  cand.tempLow          = (double)(doc["tempLow"]    | cand.tempLow);
  cand.tempHyst         = (double)(doc["tempHyst"]   | cand.tempHyst);
  cand.tempOffset       = (double)(doc["tempOffset"] | cand.tempOffset);

  cand.humEnable        = (uint64_t)humEnable;
  cand.humCondition     = (uint64_t)humCondition;
  cand.humHigh          = (double)(doc["humHigh"]    | cand.humHigh);
  cand.humLow           = (double)(doc["humLow"]     | cand.humLow);
  cand.humHyst          = (double)(doc["humHyst"]    | cand.humHyst);
  cand.humOffset        = (double)(doc["humOffset"]  | cand.humOffset);

  cand.presEnable       = (uint64_t)presEnable;
  cand.presCondition    = (uint64_t)presCondition;
  cand.presHigh         = (double)(doc["presHigh"]   | cand.presHigh);
  cand.presLow          = (double)(doc["presLow"]    | cand.presLow);
  cand.presHyst         = (double)(doc["presHyst"]   | cand.presHyst);
  cand.presOffset       = (double)(doc["presOffset"] | cand.presOffset);

  cand.checksum         = computeConfigChecksum(&cand);

  if (!validateConfig(&cand)) {
    sendNoCacheHeaders();
    g_webServer.send(400, "application/json", "{\"ok\":0,\"error\":\"Threshold or timing values out of valid range\"}");
    return;
  }

  if (cand.tempCondition != g_config.tempCondition) g_state.tempTriggered = 0ULL;
  if (cand.humCondition  != g_config.humCondition)  g_state.humTriggered  = 0ULL;
  if (cand.presCondition != g_config.presCondition) g_state.presTriggered = 0ULL;

  stageConfigSave(&cand, 0ULL);
  recomputeCalibratedMetricsFromRaw();
  g_state.lastSensorPollMs = 0ULL;
  pollBME280NonBlocking();
  updateRelaysNonBlocking(1ULL);

  sendNoCacheHeaders();
  g_webServer.send(200, "application/json", "{\"ok\":1,\"msg\":\"Sensor & relay settings applied\"}");
}

static void handleApiPostWifi() {
  if (!g_webServer.hasArg("plain")) {
    sendNoCacheHeaders();
    g_webServer.send(400, "application/json", "{\"ok\":0,\"error\":\"Missing JSON body\"}");
    return;
  }

#if ARDUINOJSON_VERSION_MAJOR >= 7
  JsonDocument doc;
#else
  StaticJsonDocument<512> doc;
#endif
  DeserializationError err = deserializeJson(doc, g_webServer.arg("plain"));
  if (err) {
    sendNoCacheHeaders();
    g_webServer.send(400, "application/json", "{\"ok\":0,\"error\":\"Malformed JSON payload\"}");
    return;
  }

  const char* newSsid = doc["wifiSsid"] | "";
  const char* newPass = doc["wifiPass"] | "";

  uint64_t ssidLen = (uint64_t)strlen(newSsid);
  uint64_t passLen = (uint64_t)strlen(newPass);

  if (ssidLen < 1ULL || ssidLen > 32ULL) {
    sendNoCacheHeaders();
    g_webServer.send(400, "application/json", "{\"ok\":0,\"error\":\"SSID length must be 1 to 32 characters\"}");
    return;
  }
  if (passLen < 8ULL || passLen > 63ULL) {
    sendNoCacheHeaders();
    g_webServer.send(400, "application/json", "{\"ok\":0,\"error\":\"WPA2 password length must be 8 to 63 characters\"}");
    return;
  }

  int64_t wifiChannel = doc["wifiChannel"] | (int64_t)g_config.wifiChannel;
  int64_t wifiMaxConn = doc["wifiMaxConn"] | (int64_t)g_config.wifiMaxConn;
  int64_t wifiHidden  = doc["wifiHidden"]  | (int64_t)g_config.wifiHidden;

  if (wifiChannel < 1LL || wifiChannel > 13LL || wifiMaxConn < 1LL || wifiMaxConn > 4LL || wifiHidden < 0LL || wifiHidden > 1LL) {
    sendNoCacheHeaders();
    g_webServer.send(400, "application/json", "{\"ok\":0,\"error\":\"Invalid Wi-Fi channel or client limit\"}");
    return;
  }

  SystemConfig cand = g_config;
  memset(cand.wifiSsid, 0, sizeof(cand.wifiSsid));
  memset(cand.wifiPass, 0, sizeof(cand.wifiPass));
  strncpy(cand.wifiSsid, newSsid, sizeof(cand.wifiSsid) - 1U);
  strncpy(cand.wifiPass, newPass, sizeof(cand.wifiPass) - 1U);

  cand.wifiChannel = (uint64_t)wifiChannel;
  cand.wifiMaxConn = (uint64_t)wifiMaxConn;
  cand.wifiHidden  = (uint64_t)wifiHidden;
  cand.checksum    = computeConfigChecksum(&cand);

  if (!validateConfig(&cand)) {
    sendNoCacheHeaders();
    g_webServer.send(400, "application/json", "{\"ok\":0,\"error\":\"Invalid Wi-Fi AP parameters\"}");
    return;
  }

  stageConfigSave(&cand, 1ULL);
  g_state.pendingWifiReload = 1ULL;
  g_state.wifiReloadAtMs    = getMillis64() + ACTION_DELAY_MS;

  sendNoCacheHeaders();
  g_webServer.send(200, "application/json", "{\"ok\":1,\"msg\":\"Wi-Fi AP configuration saved\"}");
}

static void handleApiFactoryReset() {
  sendNoCacheHeaders();
  g_webServer.send(200, "application/json", "{\"ok\":1,\"msg\":\"Factory reset scheduled\"}");
  executeFactoryResetAndScheduleReboot();
}

static void handleApiReboot() {
  sendNoCacheHeaders();
  g_webServer.send(200, "application/json", "{\"ok\":1,\"msg\":\"Reboot scheduled\"}");
  for (uint64_t i = 0ULL; i < 3ULL; i++) {
    g_state.relayLogicalState[i] = 0ULL;
    g_state.relayDesiredState[i] = 0ULL;
    writePhysicalRelayPin(i, 0ULL);
  }
  if (g_state.nvsDirty) {
    writeConfigToNvsNow();
  }
  g_state.pendingReboot = 1ULL;
  g_state.rebootAtMs    = getMillis64() + ACTION_DELAY_MS;
}

static void setupWebServerRoutes() {
  g_webServer.on("/",            HTTP_GET, handleRootPage);
  g_webServer.on("/dashboard",   HTTP_GET, handleRootPage);
  g_webServer.on("/settings",    HTTP_GET, handleRootPage);
  g_webServer.on("/wifi",        HTTP_GET, handleRootPage);
  g_webServer.on("/favicon.ico", HTTP_GET, handleFavicon);

  g_webServer.on("/generate_204",              HTTP_GET, sendCaptivePortalRedirect);
  g_webServer.on("/gen_204",                   HTTP_GET, sendCaptivePortalRedirect);
  g_webServer.on("/hotspot-detect.html",       HTTP_GET, sendCaptivePortalRedirect);
  g_webServer.on("/library/test/success.html", HTTP_GET, sendCaptivePortalRedirect);
  g_webServer.on("/ncsi.txt",                  HTTP_GET, sendCaptivePortalRedirect);
  g_webServer.on("/connecttest.txt",           HTTP_GET, sendCaptivePortalRedirect);
  g_webServer.on("/redirect",                  HTTP_GET, sendCaptivePortalRedirect);
  g_webServer.on("/success.txt",               HTTP_GET, sendCaptivePortalRedirect);
  g_webServer.on("/canonical.html",            HTTP_GET, sendCaptivePortalRedirect);
  g_webServer.on("/check_network_status.txt",  HTTP_GET, sendCaptivePortalRedirect);

  g_webServer.on("/api/status",   HTTP_GET,  handleApiStatus);
  g_webServer.on("/api/settings", HTTP_GET,  handleApiGetSettings);
  g_webServer.on("/api/settings", HTTP_POST, handleApiPostSettings);
  g_webServer.on("/api/wifi",     HTTP_POST, handleApiPostWifi);
  g_webServer.on("/api/reset",    HTTP_POST, handleApiFactoryReset);
  g_webServer.on("/api/reboot",   HTTP_POST, handleApiReboot);

  g_webServer.onNotFound([]() {
    sendCaptivePortalRedirect();
  });

  g_webServer.begin();
}

static void serviceSystemTasksNonBlocking() {
  const uint64_t nowMs = getMillis64();

  if (g_state.nvsDirty) {
    if ((nowMs - g_state.nvsStageMs) >= NVS_COMMIT_DELAY_MS &&
        (g_state.nvsLastWriteMs == 0ULL || (nowMs - g_state.nvsLastWriteMs) >= NVS_MIN_WRITE_GAP_MS)) {
      writeConfigToNvsNow();
    }
  }

  if (g_state.pendingWifiReload && nowMs >= g_state.wifiReloadAtMs) {
    g_state.pendingWifiReload = 0ULL;
    startOrReloadWiFiAP();
  }

  if (g_state.pendingReboot && nowMs >= g_state.rebootAtMs) {
    g_state.pendingReboot = 0ULL;
    ESP.restart();
  }
}

static void initWatchdogTimer() {
#if ESP_IDF_VERSION_MAJOR >= 5
  esp_task_wdt_config_t twdt_config = {
      .timeout_ms     = (uint32_t)(WDT_TIMEOUT_SEC * 1000ULL),
      .idle_core_mask = (1U << portNUM_PROCESSORS) - 1U,
      .trigger_panic  = true
  };
  if (esp_task_wdt_reconfigure(&twdt_config) != ESP_OK) {
    esp_task_wdt_init(&twdt_config);
  }
  if (esp_task_wdt_status(NULL) != ESP_OK) {
    esp_task_wdt_add(NULL);
  }
#else
  esp_task_wdt_init((uint32_t)WDT_TIMEOUT_SEC, true);
  esp_task_wdt_add(NULL);
#endif
}

void setup() {
  esp_log_level_set("*", ESP_LOG_NONE);
  memset(&g_state, 0, sizeof(RuntimeState));

  pinMode((uint8_t)PIN_FACTORY_RESET_BTN, INPUT_PULLUP);
  initStorageAndConfig();
  initRelayHardware();

  initOrRecoverI2CBus(0ULL);
  tryInitBME280();

  WiFi.useStaticBuffers(true);
  startOrReloadWiFiAP();
  setupWebServerRoutes();
  initWatchdogTimer();
}

void loop() {
  esp_task_wdt_reset();
  g_dnsServer.processNextRequest();
  g_webServer.handleClient();
  pollFactoryResetButtonNonBlocking();
  pollBME280NonBlocking();
  updateRelaysNonBlocking(0ULL);
  serviceSystemTasksNonBlocking();
}
