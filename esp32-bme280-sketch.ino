/*
  ESP32 Unified Environment Controller
  Target: classic ESP32, Arduino-ESP32 core 3.x (compiled with 3.3.12)

  Wiring: BME280 SDA=GPIO21, SCL=GPIO22; relay driver input=GPIO25;
  factory reset=BOOT button on GPIO0 (active LOW, hold 5 seconds).

  No delay(), polling wait loops, WiFi scans, filesystem, CDN, or periodic NVS
  writes are used. Arduino WebServer, Wire and Preferences operations are
  synchronous bounded library transactions; application scheduling is cooperative.
*/

#include <Arduino.h>
#include <Wire.h>
#include <WiFi.h>
#include <WebServer.h>
#include <DNSServer.h>
#include <ESPmDNS.h>
#include <Preferences.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <esp_wifi.h>
#include <esp_system.h>
#include <math.h>
#include <new>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#define FIRMWARE_VERSION "1.0.1"
#define DEVICE_NAME "ESP32 Environment Controller"

static constexpr uint8_t PIN_I2C_SDA = 21;
static constexpr uint8_t PIN_I2C_SCL = 22;
static constexpr uint8_t PIN_RELAY = 25;
static constexpr uint8_t PIN_FACTORY = 0;
static constexpr uint16_t HTTP_PORT = 80;
static constexpr uint16_t DNS_PORT = 53;
static constexpr uint32_t SENSOR_PERIOD_MS = 1000;
static constexpr uint32_t SENSOR_STALE_MS = 5000;
static constexpr uint32_t SENSOR_RETRY_MS = 10000;
static constexpr uint32_t SENSOR_RESET_MS = 10;
static constexpr uint32_t SENSOR_CALIBRATION_TIMEOUT_MS = 250;
static constexpr uint32_t SENSOR_MEASUREMENT_TIMEOUT_MS = 100;
static constexpr uint32_t NETWORK_PERIOD_MS = 500;
static constexpr uint32_t AP_WATCHDOG_MS = 10000;
static constexpr uint32_t CONFIG_DEBOUNCE_MS = 1500;
static constexpr uint32_t CONFIG_RETRY_MS = 10000;
static constexpr uint32_t FACTORY_HOLD_MS = 5000;
static constexpr uint32_t FACTORY_DEBOUNCE_MS = 40;
static constexpr uint32_t RESTART_RESPONSE_MS = 1400;
static constexpr size_t JSON_CAPACITY = 6144;
static constexpr uint32_t RECORD_MAGIC = 0x454E5632UL;
static constexpr uint16_t RECORD_VERSION = 1;

struct SensorConfig {
  uint8_t address;
  float temperatureOffset;
  float humidityOffset;
  float pressureOffset;
};

struct RelayConfig {
  uint8_t mode;
  uint8_t source;
  uint8_t condition;
  uint8_t activeHigh;
  float threshold;
  float hysteresis;
};

struct ApConfig {
  char ssid[33];
  char password[64];
  uint8_t channel;
  uint8_t hidden;
  uint8_t maxClients;
  int8_t txPowerQuarterDbm;
};

struct StaConfig {
  char ssid[33];
  char password[64];
  char hostname[33];
  uint8_t dhcp;
  uint8_t useCustomMac;
  uint8_t mac[6];
  uint8_t ip[4];
  uint8_t gateway[4];
  uint8_t subnet[4];
  uint8_t dns1[4];
  uint8_t dns2[4];
};

struct Config {
  SensorConfig sensor;
  RelayConfig relay;
  ApConfig ap;
  StaConfig sta;
};

struct ConfigRecord {
  uint32_t magic;
  uint16_t version;
  uint16_t size;
  uint32_t generation;
  uint32_t crc;
  Config config;
};

static_assert(sizeof(Config) < 1024, "Config unexpectedly large");

class ManagedBME280 : public Adafruit_BME280 {
public:
  bool prepare(uint8_t address, TwoWire *wire) {
    if (i2c_dev && _i2caddr == address) return i2c_dev->begin();
    if (i2c_dev) {
      delete i2c_dev;
      i2c_dev = nullptr;
    }
    _i2caddr = address;
    i2c_dev = new (std::nothrow) Adafruit_I2CDevice(address, wire);
    return i2c_dev && i2c_dev->begin();
  }
  bool identify() {
    _sensorID = read8(BME280_REGISTER_CHIPID);
    return _sensorID == 0x60;
  }
  void requestReset() { write8(BME280_REGISTER_SOFTRESET, 0xB6); }
  bool calibrationPending() { return isReadingCalibration(); }
  bool measurementReady() { return (read8(BME280_REGISTER_STATUS) & 0x08) == 0; }
  void loadCoefficientsAndConfigure() {
    readCoefficients();
    setSampling(Adafruit_BME280::MODE_NORMAL,
                Adafruit_BME280::SAMPLING_X2,
                Adafruit_BME280::SAMPLING_X2,
                Adafruit_BME280::SAMPLING_X2,
                Adafruit_BME280::FILTER_X2,
                Adafruit_BME280::STANDBY_MS_500);
  }
};

static Config cfg;
static Preferences preferences;
static bool preferencesReady = false;
static bool storageOk = false;
static bool configDirty = false;
static int8_t activeSlot = -1;
static uint8_t validSlotMask = 0;
static uint32_t activeGeneration = 0;
static uint32_t successfulSaves = 0;
static uint32_t lastConfigEditMs = 0;
static uint32_t lastSaveAttemptMs = 0;
static uint32_t lastStorageRetryMs = 0;

static ManagedBME280 bme;
enum class SensorInitStage : uint8_t { Idle, ResetWait, CalibrationWait, MeasurementWait };
static SensorInitStage sensorInitStage = SensorInitStage::Idle;
static uint32_t sensorInitStageStartedMs = 0;
static bool sensorPresent = false;
static bool sensorReadingValid = false;
static bool sensorEverGood = false;
static float temperatureC = NAN;
static float humidityPct = NAN;
static float pressureHpa = NAN;
static uint32_t sensorChipId = 0;
static uint32_t sensorErrors = 0;
static uint8_t consecutiveSensorErrors = 0;
static uint32_t lastSensorSampleMs = 0;
static uint32_t lastSensorRetryMs = 0;
static uint64_t lastGoodUptimeMs = 0;
static bool sensorApplyPending = false;

static bool relayOn = false;
static const char *relayReason = "boot safe off";

static WebServer server(HTTP_PORT);
static DNSServer dnsServer;
static bool dnsActive = false;
static bool apRunning = false;
static bool txPowerApplyOk = false;
static bool apApplyPending = false;
static bool staApplyPending = false;
static bool staConfigOk = true;
static bool macApplyOk = true;
static uint32_t lastNetworkServiceMs = 0;
static uint32_t lastApWatchdogMs = 0;
static uint32_t lastStaAttemptMs = 0;
static uint32_t staRetryBackoffMs = 5000;
static wl_status_t previousStaStatus = WL_NO_SHIELD;

static bool restartPending = false;
static uint32_t restartRequestedMs = 0;
static uint32_t restartDelayMs = RESTART_RESPONSE_MS;
static const char *restartReason = "";
static bool factoryPending = false;

static bool buttonStable = HIGH;
static bool buttonLastRaw = HIGH;
static uint32_t buttonRawChangedMs = 0;
static uint32_t buttonPressedMs = 0;
static RTC_DATA_ATTR bool factoryResetLatched = false;

static uint32_t clockLastMillis = 0;
static uint32_t clockLastMicros = 0;
static uint64_t monotonicMillis = 0;
static uint64_t monotonicMicros = 0;

static char jsonBuffer[JSON_CAPACITY];

static inline bool elapsed(uint32_t now, uint32_t since, uint32_t interval) {
  return static_cast<uint32_t>(now - since) >= interval;
}

static void updateMonotonicClock() {
  const uint32_t nowMs = millis();
  const uint32_t nowUs = micros();
  monotonicMillis += static_cast<uint32_t>(nowMs - clockLastMillis);
  monotonicMicros += static_cast<uint32_t>(nowUs - clockLastMicros);
  clockLastMillis = nowMs;
  clockLastMicros = nowUs;
}

struct JsonWriter {
  char *buffer;
  size_t capacity;
  size_t length = 0;
  bool ok = true;

  void append(const char *text, size_t count) {
    if (!ok || count > capacity - 1 - length) {
      ok = false;
      return;
    }
    memcpy(buffer + length, text, count);
    length += count;
    buffer[length] = '\0';
  }

  void raw(const char *text) { append(text, strlen(text)); }

  void formatted(const char *format, ...) {
    if (!ok) return;
    va_list args;
    va_start(args, format);
    const int remaining = static_cast<int>(capacity - length);
    const int written = vsnprintf(buffer + length, remaining, format, args);
    va_end(args);
    if (written < 0 || written >= remaining) {
      ok = false;
      buffer[length] = '\0';
      return;
    }
    length += static_cast<size_t>(written);
  }

  void string(const char *text) {
    raw("\"");
    for (const unsigned char *p = reinterpret_cast<const unsigned char *>(text); *p && ok; ++p) {
      char escaped[7];
      switch (*p) {
        case '\"': raw("\\\""); break;
        case '\\': raw("\\\\"); break;
        case '\b': raw("\\b"); break;
        case '\f': raw("\\f"); break;
        case '\n': raw("\\n"); break;
        case '\r': raw("\\r"); break;
        case '\t': raw("\\t"); break;
        default:
          if (*p < 0x20) {
            snprintf(escaped, sizeof(escaped), "\\u%04x", *p);
            raw(escaped);
          } else {
            append(reinterpret_cast<const char *>(p), 1);
          }
      }
    }
    raw("\"");
  }

  void ip(const IPAddress &address) {
    formatted("\"%u.%u.%u.%u\"", address[0], address[1], address[2], address[3]);
  }

  void ip(const uint8_t address[4]) {
    formatted("\"%u.%u.%u.%u\"", address[0], address[1], address[2], address[3]);
  }

  void mac(const uint8_t address[6]) {
    formatted("\"%02X:%02X:%02X:%02X:%02X:%02X\"", address[0], address[1],
              address[2], address[3], address[4], address[5]);
  }

  void nullableFloat(float value, uint8_t decimals) {
    if (isfinite(value)) formatted("%.*f", decimals, value);
    else raw("null");
  }
};

static void macString(JsonWriter &json, wifi_interface_t interface);

static IPAddress toIp(const uint8_t value[4]) {
  return IPAddress(value[0], value[1], value[2], value[3]);
}

static uint32_t ipToU32(const uint8_t value[4]) {
  return (static_cast<uint32_t>(value[0]) << 24) |
         (static_cast<uint32_t>(value[1]) << 16) |
         (static_cast<uint32_t>(value[2]) << 8) | value[3];
}

static bool parseIp(const String &text, uint8_t output[4]) {
  if (strlen(text.c_str()) != text.length()) return false;
  const char *p = text.c_str();
  for (uint8_t i = 0; i < 4; ++i) {
    if (*p < '0' || *p > '9') return false;
    uint16_t value = 0;
    uint8_t digits = 0;
    while (*p >= '0' && *p <= '9') {
      value = static_cast<uint16_t>(value * 10 + (*p - '0'));
      if (value > 255 || ++digits > 3) return false;
      ++p;
    }
    output[i] = static_cast<uint8_t>(value);
    if (i < 3) {
      if (*p != '.') return false;
      ++p;
    } else if (*p != '\0') {
      return false;
    }
  }
  return true;
}

static bool parseMac(const String &text, uint8_t output[6]) {
  if (strlen(text.c_str()) != text.length()) return false;
  const char *p = text.c_str();
  for (uint8_t i = 0; i < 6; ++i) {
    uint8_t value = 0;
    for (uint8_t digit = 0; digit < 2; ++digit) {
      const char c = *p++;
      uint8_t nibble;
      if (c >= '0' && c <= '9') nibble = c - '0';
      else if (c >= 'a' && c <= 'f') nibble = c - 'a' + 10;
      else if (c >= 'A' && c <= 'F') nibble = c - 'A' + 10;
      else return false;
      value = static_cast<uint8_t>((value << 4) | nibble);
    }
    output[i] = value;
    if (i < 5) {
      if (*p != ':' && *p != '-') return false;
      ++p;
    } else if (*p != '\0') {
      return false;
    }
  }
  return true;
}

static bool isZeroIp(const uint8_t ip[4]) {
  return (ip[0] | ip[1] | ip[2] | ip[3]) == 0;
}

static bool isUnicastIp(const uint8_t ip[4]) {
  if (ip[0] == 0 || ip[0] == 127 || ip[0] >= 224) return false;
  return !(ip[0] == 255 && ip[1] == 255 && ip[2] == 255 && ip[3] == 255);
}

static bool isContiguousMask(const uint8_t mask[4]) {
  const uint32_t value = ipToU32(mask);
  const uint32_t inverse = ~value;
  const uint8_t prefix = static_cast<uint8_t>(__builtin_popcount(value));
  return ((inverse & (inverse + 1U)) == 0U) && prefix >= 1 && prefix <= 30;
}

static bool validStaticNetwork(const StaConfig &sta, char *error, size_t capacity) {
  if (!isContiguousMask(sta.subnet)) {
    snprintf(error, capacity, "Subnet must be a contiguous mask between /1 and /30.");
    return false;
  }
  if (!isUnicastIp(sta.ip) || !isUnicastIp(sta.gateway)) {
    snprintf(error, capacity, "Static IP and gateway must be valid unicast IPv4 addresses.");
    return false;
  }
  const uint32_t ip = ipToU32(sta.ip);
  const uint32_t gateway = ipToU32(sta.gateway);
  const uint32_t mask = ipToU32(sta.subnet);
  const uint32_t network = ip & mask;
  const uint32_t broadcast = network | ~mask;
  if ((gateway & mask) != network) {
    snprintf(error, capacity, "Gateway must be in the same subnet as the static IP.");
    return false;
  }
  if (ip == network || ip == broadcast || gateway == network || gateway == broadcast || ip == gateway) {
    snprintf(error, capacity, "IP/gateway cannot be network, broadcast, or duplicate each other.");
    return false;
  }
  const uint32_t apNetwork = 0xC0A80400UL;
  const uint32_t apMask = 0xFFFFFF00UL;
  if ((network & apMask) == (apNetwork & mask)) {
    snprintf(error, capacity, "Static STA subnet overlaps recovery AP subnet 192.168.4.0/24.");
    return false;
  }
  const auto invalidLocalDns = [&](const uint8_t dns[4]) {
    if (isZeroIp(dns)) return false;
    if (!isUnicastIp(dns)) return true;
    const uint32_t address = ipToU32(dns);
    return address == ip || ((address & mask) == network &&
           (address == network || address == broadcast));
  };
  if (invalidLocalDns(sta.dns1)) {
    snprintf(error, capacity, "DNS 1 is not a usable unicast address for this configuration.");
    return false;
  }
  if (invalidLocalDns(sta.dns2)) {
    snprintf(error, capacity, "DNS 2 is not a usable unicast address for this configuration.");
    return false;
  }
  return true;
}

static bool validUtf8(const char *text, size_t length) {
  const unsigned char *s = reinterpret_cast<const unsigned char *>(text);
  size_t i = 0;
  while (i < length) {
    const unsigned char c = s[i++];
    if (c <= 0x7F) continue;
    uint8_t continuation;
    unsigned char secondMin = 0x80, secondMax = 0xBF;
    if (c >= 0xC2 && c <= 0xDF) continuation = 1;
    else if (c >= 0xE0 && c <= 0xEF) {
      continuation = 2;
      if (c == 0xE0) secondMin = 0xA0;
      if (c == 0xED) secondMax = 0x9F;
    } else if (c >= 0xF0 && c <= 0xF4) {
      continuation = 3;
      if (c == 0xF0) secondMin = 0x90;
      if (c == 0xF4) secondMax = 0x8F;
    } else return false;
    if (i + continuation > length || s[i] < secondMin || s[i] > secondMax) return false;
    ++i;
    for (uint8_t n = 1; n < continuation; ++n, ++i)
      if (s[i] < 0x80 || s[i] > 0xBF) return false;
  }
  return true;
}

static bool validSsid(const char *ssid, bool allowEmpty) {
  const size_t length = strlen(ssid);
  if (length > 32 || (!allowEmpty && length == 0) || !validUtf8(ssid, length)) return false;
  for (size_t i = 0; i < length; ++i) {
    const unsigned char c = static_cast<unsigned char>(ssid[i]);
    if (c < 0x20 || c == 0x7F) return false;
  }
  return true;
}

static bool validPassword(const char *password) {
  const size_t length = strlen(password);
  if (length == 0) return true;
  if (length < 8 || length > 63) return false;
  for (size_t i = 0; i < length; ++i) {
    const unsigned char c = static_cast<unsigned char>(password[i]);
    if (c < 0x20 || c > 0x7E) return false;
  }
  return true;
}

static bool validHostname(const char *hostname) {
  const size_t length = strlen(hostname);
  if (length < 1 || length > 32 || hostname[0] == '-' || hostname[length - 1] == '-') return false;
  for (size_t i = 0; i < length; ++i) {
    const char c = hostname[i];
    if (!((c >= 'a' && c <= 'z') || (c >= 'A' && c <= 'Z') ||
          (c >= '0' && c <= '9') || c == '-')) return false;
  }
  return true;
}

static bool validTxPower(int8_t value) {
  switch (value) {
    case WIFI_POWER_MINUS_1dBm:
    case WIFI_POWER_2dBm:
    case WIFI_POWER_5dBm:
    case WIFI_POWER_7dBm:
    case WIFI_POWER_8_5dBm:
    case WIFI_POWER_11dBm:
    case WIFI_POWER_13dBm:
    case WIFI_POWER_15dBm:
    case WIFI_POWER_17dBm:
    case WIFI_POWER_18_5dBm:
    case WIFI_POWER_19dBm:
    case WIFI_POWER_19_5dBm:
      return true;
    default:
      return false;
  }
}

static bool validateConfig(const Config &value, char *error, size_t capacity) {
  if (!memchr(value.ap.ssid, '\0', sizeof(value.ap.ssid)) ||
      !memchr(value.ap.password, '\0', sizeof(value.ap.password)) ||
      !memchr(value.sta.ssid, '\0', sizeof(value.sta.ssid)) ||
      !memchr(value.sta.password, '\0', sizeof(value.sta.password)) ||
      !memchr(value.sta.hostname, '\0', sizeof(value.sta.hostname))) {
    snprintf(error, capacity, "A fixed-size configuration string is not terminated."); return false;
  }
  const SensorConfig &sensor = value.sensor;
  if (sensor.address != 0x76 && sensor.address != 0x77) {
    snprintf(error, capacity, "BME280 address must be 0x76 or 0x77."); return false;
  }
  if (!isfinite(sensor.temperatureOffset) || fabsf(sensor.temperatureOffset) > 20.0f ||
      !isfinite(sensor.humidityOffset) || fabsf(sensor.humidityOffset) > 20.0f ||
      !isfinite(sensor.pressureOffset) || fabsf(sensor.pressureOffset) > 50.0f) {
    snprintf(error, capacity, "Sensor calibration offset is outside its permitted range."); return false;
  }
  const RelayConfig &relay = value.relay;
  if (relay.mode > 2 || relay.source > 2 || relay.condition > 1 || relay.activeHigh > 1 ||
      !isfinite(relay.threshold) || !isfinite(relay.hysteresis) || relay.hysteresis < 0.0f) {
    snprintf(error, capacity, "Relay mode, source, condition, or hysteresis is invalid."); return false;
  }
  float minimum = -40.0f, maximum = 85.0f, maximumHysteresis = 20.0f;
  if (relay.source == 1) { minimum = 0.0f; maximum = 100.0f; maximumHysteresis = 50.0f; }
  if (relay.source == 2) { minimum = 300.0f; maximum = 1100.0f; maximumHysteresis = 100.0f; }
  if (relay.threshold < minimum || relay.threshold > maximum || relay.hysteresis > maximumHysteresis) {
    snprintf(error, capacity, "Relay threshold or hysteresis is outside the selected sensor range."); return false;
  }
  if (!validSsid(value.ap.ssid, false) || !validPassword(value.ap.password) ||
      value.ap.channel < 1 || value.ap.channel > 13 || value.ap.hidden > 1 ||
      value.ap.maxClients < 1 || value.ap.maxClients > 4 || !validTxPower(value.ap.txPowerQuarterDbm)) {
    snprintf(error, capacity, "AP SSID, password, channel, client limit, or TX power is invalid."); return false;
  }
  if (!validSsid(value.sta.ssid, true) || !validPassword(value.sta.password) ||
      !validHostname(value.sta.hostname) || value.sta.dhcp > 1 || value.sta.useCustomMac > 1) {
    snprintf(error, capacity, "STA credentials, hostname, or IP mode is invalid."); return false;
  }
  if (value.sta.ssid[0] == '\0' && value.sta.password[0] != '\0') {
    snprintf(error, capacity, "Clear the STA password when no STA SSID is configured."); return false;
  }
  if (value.sta.useCustomMac) {
    bool allZero = true;
    for (uint8_t i = 0; i < 6; ++i) if (value.sta.mac[i] != 0) allZero = false;
    if (allZero || (value.sta.mac[0] & 1U)) {
      snprintf(error, capacity, "Custom MAC must be nonzero and unicast (first octet LSB=0)."); return false;
    }
  }
  if (!value.sta.dhcp && !validStaticNetwork(value.sta, error, capacity)) return false;
  return true;
}

static uint32_t crcUpdate(uint32_t crc, const void *data, size_t length) {
  const uint8_t *bytes = static_cast<const uint8_t *>(data);
  crc = ~crc;
  for (size_t i = 0; i < length; ++i) {
    crc ^= bytes[i];
    for (uint8_t bit = 0; bit < 8; ++bit)
      crc = (crc >> 1) ^ (0xEDB88320UL & (0U - (crc & 1U)));
  }
  return ~crc;
}

static uint32_t recordCrc(const ConfigRecord &record) {
  uint32_t crc = 0;
  crc = crcUpdate(crc, &record.magic, sizeof(record.magic));
  crc = crcUpdate(crc, &record.version, sizeof(record.version));
  crc = crcUpdate(crc, &record.size, sizeof(record.size));
  crc = crcUpdate(crc, &record.generation, sizeof(record.generation));
  crc = crcUpdate(crc, &record.config, sizeof(record.config));
  return crc;
}

static bool readSlot(uint8_t slot, ConfigRecord &record) {
  const char *key = slot == 0 ? "cfg0" : "cfg1";
  memset(&record, 0, sizeof(record));
  if (!preferencesReady || preferences.getBytesLength(key) != sizeof(record)) return false;
  if (preferences.getBytes(key, &record, sizeof(record)) != sizeof(record)) return false;
  if (record.magic != RECORD_MAGIC || record.version != RECORD_VERSION ||
      record.size != sizeof(record) || record.crc != recordCrc(record)) return false;
  char error[96];
  return validateConfig(record.config, error, sizeof(error));
}

static bool writeSlot(uint8_t slot, const Config &value, uint32_t generation) {
  if (!preferencesReady) return false;
  ConfigRecord record;
  memset(&record, 0, sizeof(record));
  record.magic = RECORD_MAGIC;
  record.version = RECORD_VERSION;
  record.size = sizeof(record);
  record.generation = generation;
  record.config = value;
  record.crc = recordCrc(record);
  const char *key = slot == 0 ? "cfg0" : "cfg1";
  if (preferences.putBytes(key, &record, sizeof(record)) != sizeof(record)) return false;
  ConfigRecord verify;
  if (preferences.getBytes(key, &verify, sizeof(verify)) != sizeof(verify)) return false;
  return memcmp(&record, &verify, sizeof(record)) == 0 && readSlot(slot, verify);
}

static void setDefaults() {
  memset(&cfg, 0, sizeof(cfg));
  cfg.sensor.address = 0x76;
  cfg.relay.mode = 0;
  cfg.relay.source = 0;
  cfg.relay.condition = 0;
  cfg.relay.activeHigh = 1;
  cfg.relay.threshold = 30.0f;
  cfg.relay.hysteresis = 1.0f;
  cfg.ap.channel = 1;
  cfg.ap.hidden = 0;
  cfg.ap.maxClients = 4;
  cfg.ap.txPowerQuarterDbm = WIFI_POWER_19_5dBm;
  strcpy(cfg.ap.password, "configure123");
  const uint64_t chip = ESP.getEfuseMac();
  snprintf(cfg.ap.ssid, sizeof(cfg.ap.ssid), "ESP32-ENV-%04X",
           static_cast<unsigned>(chip & 0xFFFFU));
  strcpy(cfg.sta.hostname, "esp32-environment");
  cfg.sta.dhcp = 1;
  cfg.sta.useCustomMac = 0;
  const uint8_t localMac[6] = {0x02, 0x00, 0x00, 0x00, 0x00, 0x01};
  memcpy(cfg.sta.mac, localMac, sizeof(localMac));
  const uint8_t ip[4] = {192, 168, 1, 50};
  const uint8_t gateway[4] = {192, 168, 1, 1};
  const uint8_t subnet[4] = {255, 255, 255, 0};
  const uint8_t dns1[4] = {1, 1, 1, 1};
  const uint8_t dns2[4] = {8, 8, 8, 8};
  memcpy(cfg.sta.ip, ip, 4); memcpy(cfg.sta.gateway, gateway, 4);
  memcpy(cfg.sta.subnet, subnet, 4); memcpy(cfg.sta.dns1, dns1, 4); memcpy(cfg.sta.dns2, dns2, 4);
}

static bool generationNewer(uint32_t a, uint32_t b) {
  return static_cast<int32_t>(a - b) > 0;
}

static void loadConfiguration() {
  setDefaults();
  preferencesReady = preferences.begin("env32", false);
  if (!preferencesReady) {
    storageOk = false;
    return;
  }
  ConfigRecord records[2];
  const bool valid0 = readSlot(0, records[0]);
  const bool valid1 = readSlot(1, records[1]);
  validSlotMask = (valid0 ? 1U : 0U) | (valid1 ? 2U : 0U);
  if (!valid0 && !valid1) {
    activeGeneration = 2;
    const bool first = writeSlot(0, cfg, 1);
    const bool second = writeSlot(1, cfg, 2);
    activeSlot = second ? 1 : (first ? 0 : -1);
    activeGeneration = second ? 2 : (first ? 1 : 0);
    validSlotMask = (first ? 1U : 0U) | (second ? 2U : 0U);
    storageOk = second || first;
    return;
  }
  activeSlot = (valid1 && (!valid0 || generationNewer(records[1].generation, records[0].generation))) ? 1 : 0;
  activeGeneration = records[activeSlot].generation;
  cfg = records[activeSlot].config;
  storageOk = true;
}

static bool saveConfigurationNow() {
  lastSaveAttemptMs = millis();
  if (!preferencesReady) {
    preferencesReady = preferences.begin("env32", false);
    if (!preferencesReady) { storageOk = false; return false; }
  }
  ConfigRecord records[2];
  const bool valid0 = readSlot(0, records[0]);
  const bool valid1 = readSlot(1, records[1]);
  validSlotMask = (valid0 ? 1U : 0U) | (valid1 ? 2U : 0U);
  uint8_t target;
  if (!valid0) target = 0;
  else if (!valid1) target = 1;
  else {
    const uint32_t generation0 = records[0].generation;
    const uint32_t generation1 = records[1].generation;
    target = generationNewer(generation0, generation1) ? 1 : 0;
  }
  uint32_t newest = activeGeneration;
  if (valid0 && generationNewer(records[0].generation, newest)) newest = records[0].generation;
  if (valid1 && generationNewer(records[1].generation, newest)) newest = records[1].generation;
  const uint32_t nextGeneration = newest + 1U;
  if (!writeSlot(target, cfg, nextGeneration)) {
    storageOk = false;
    return false;
  }
  activeSlot = target;
  activeGeneration = nextGeneration;
  validSlotMask |= static_cast<uint8_t>(1U << target);
  configDirty = false;
  storageOk = true;
  ++successfulSaves;
  return true;
}

static void markConfigDirty() {
  configDirty = true;
  lastConfigEditMs = millis();
}

static void serviceStorage(uint32_t now) {
  if (!preferencesReady && elapsed(now, lastStorageRetryMs, CONFIG_RETRY_MS)) {
    lastStorageRetryMs = now;
    preferencesReady = preferences.begin("env32", false);
  }
  if (configDirty && elapsed(now, lastConfigEditMs, CONFIG_DEBOUNCE_MS) &&
      elapsed(now, lastSaveAttemptMs, CONFIG_RETRY_MS)) {
    saveConfigurationNow();
  }
}

static void failSensorInitialization(uint32_t now) {
  sensorPresent = false;
  sensorReadingValid = false;
  sensorChipId = 0;
  sensorInitStage = SensorInitStage::Idle;
  consecutiveSensorErrors = 0;
  lastSensorRetryMs = now;
  ++sensorErrors;
}

static void startSensorInitialization(uint32_t now) {
  sensorPresent = false;
  sensorReadingValid = false;
  sensorChipId = 0;
  lastSensorRetryMs = now;
  if (!bme.prepare(cfg.sensor.address, &Wire) || !bme.identify()) {
    failSensorInitialization(now);
    return;
  }
  sensorChipId = 0x60;
  bme.requestReset();
  sensorInitStage = SensorInitStage::ResetWait;
  sensorInitStageStartedMs = now;
}

static void serviceSensorInitialization(uint32_t now) {
  switch (sensorInitStage) {
    case SensorInitStage::Idle:
      return;
    case SensorInitStage::ResetWait:
      if (elapsed(now, sensorInitStageStartedMs, SENSOR_RESET_MS)) {
        sensorInitStage = SensorInitStage::CalibrationWait;
        sensorInitStageStartedMs = now;
      }
      return;
    case SensorInitStage::CalibrationWait:
      if (!bme.calibrationPending()) {
        if (!bme.identify()) { failSensorInitialization(now); return; }
        bme.loadCoefficientsAndConfigure();
        sensorInitStage = SensorInitStage::MeasurementWait;
        sensorInitStageStartedMs = now;
      } else if (elapsed(now, sensorInitStageStartedMs, SENSOR_CALIBRATION_TIMEOUT_MS)) {
        failSensorInitialization(now);
      }
      return;
    case SensorInitStage::MeasurementWait:
      if (bme.measurementReady()) {
        if (!bme.identify()) { failSensorInitialization(now); return; }
        sensorPresent = true;
        sensorInitStage = SensorInitStage::Idle;
        lastSensorSampleMs = now - SENSOR_PERIOD_MS;
      } else if (elapsed(now, sensorInitStageStartedMs, SENSOR_MEASUREMENT_TIMEOUT_MS)) {
        failSensorInitialization(now);
      }
      return;
  }
}

static void registerSensorError(uint32_t now) {
  sensorReadingValid = false;
  ++sensorErrors;
  if (consecutiveSensorErrors < 255) ++consecutiveSensorErrors;
  if (consecutiveSensorErrors >= 5) {
    sensorPresent = false;
    sensorChipId = 0;
    sensorInitStage = SensorInitStage::Idle;
    consecutiveSensorErrors = 0;
    lastSensorRetryMs = now;
  }
}

static void sampleSensor(uint32_t now) {
  if (!sensorPresent) {
    sensorReadingValid = false;
    if (sensorInitStage == SensorInitStage::Idle && elapsed(now, lastSensorRetryMs, SENSOR_RETRY_MS))
      startSensorInitialization(now);
    return;
  }
  if (!elapsed(now, lastSensorSampleMs, SENSOR_PERIOD_MS)) return;
  lastSensorSampleMs = now;
  const float newTemperature = bme.readTemperature();
  const float newHumidity = bme.readHumidity();
  const float newPressure = bme.readPressure() / 100.0f;
  const bool plausible = isfinite(newTemperature) && isfinite(newHumidity) && isfinite(newPressure) &&
                         newTemperature >= -40.0f && newTemperature <= 85.0f &&
                         newHumidity >= 0.0f && newHumidity <= 100.0f &&
                         newPressure >= 300.0f && newPressure <= 1100.0f;
  if (!plausible) {
    registerSensorError(now);
    return;
  }
  temperatureC = newTemperature + cfg.sensor.temperatureOffset;
  humidityPct = newHumidity + cfg.sensor.humidityOffset;
  pressureHpa = newPressure + cfg.sensor.pressureOffset;
  sensorReadingValid = isfinite(temperatureC) && isfinite(humidityPct) && isfinite(pressureHpa);
  if (sensorReadingValid) {
    consecutiveSensorErrors = 0;
    sensorEverGood = true;
    lastGoodUptimeMs = monotonicMillis;
  } else {
    registerSensorError(now);
  }
}

static void serviceSensor(uint32_t now) {
  if (sensorApplyPending) {
    sensorApplyPending = false;
    startSensorInitialization(now);
  }
  serviceSensorInitialization(now);
  sampleSensor(now);
  if (sensorEverGood && monotonicMillis - lastGoodUptimeMs > SENSOR_STALE_MS)
    sensorReadingValid = false;
}

static void writeRelay(bool on, const char *reason, bool force = false) {
  if (force || relayOn != on) {
    relayOn = on;
    digitalWrite(PIN_RELAY, (relayOn == static_cast<bool>(cfg.relay.activeHigh)) ? HIGH : LOW);
  }
  relayReason = reason;
}

static float selectedSensorValue() {
  if (cfg.relay.source == 1) return humidityPct;
  if (cfg.relay.source == 2) return pressureHpa;
  return temperatureC;
}

static void serviceRelay() {
  if (cfg.relay.mode == 1) {
    writeRelay(false, "manual off");
    return;
  }
  if (cfg.relay.mode == 2) {
    writeRelay(true, "manual on");
    return;
  }
  if (!sensorPresent || !sensorReadingValid) {
    writeRelay(false, "automatic fail-safe: sensor unavailable or stale");
    return;
  }
  const float value = selectedSensorValue();
  bool requested = relayOn;
  const char *reason = relayOn ? "automatic hysteresis hold ON" : "automatic hysteresis hold OFF";
  if (cfg.relay.condition == 0) {
    if (!relayOn && value >= cfg.relay.threshold) {
      requested = true; reason = "automatic: value at/above threshold";
    } else if (relayOn && value < cfg.relay.threshold - cfg.relay.hysteresis) {
      requested = false; reason = "automatic: value below release threshold";
    }
  } else {
    if (!relayOn && value <= cfg.relay.threshold) {
      requested = true; reason = "automatic: value at/below threshold";
    } else if (relayOn && value > cfg.relay.threshold + cfg.relay.hysteresis) {
      requested = false; reason = "automatic: value above release threshold";
    }
  }
  writeRelay(requested, reason);
}

static const IPAddress AP_IP(192, 168, 4, 1);
static const IPAddress AP_MASK(255, 255, 255, 0);

static bool accessPointHealthy() {
  if (WiFi.getMode() != WIFI_AP_STA || !WiFi.AP.started() ||
      WiFi.softAPIP() == IPAddress(0, 0, 0, 0)) return false;
  wifi_config_t config;
  if (esp_wifi_get_config(WIFI_IF_AP, &config) != ESP_OK) return false;
  const size_t ssidLength = strlen(cfg.ap.ssid);
  return config.ap.ssid_len == ssidLength &&
         memcmp(config.ap.ssid, cfg.ap.ssid, ssidLength) == 0 &&
         memcmp(config.ap.password, cfg.ap.password, sizeof(cfg.ap.password)) == 0 &&
         config.ap.ssid_hidden == cfg.ap.hidden &&
         config.ap.max_connection == cfg.ap.maxClients &&
         (WiFi.status() == WL_CONNECTED || config.ap.channel == cfg.ap.channel);
}

static bool startAccessPoint() {
  WiFi.mode(WIFI_AP_STA);
  const bool ipOk = WiFi.softAPConfig(AP_IP, AP_IP, AP_MASK);
  const bool apOk = WiFi.softAP(cfg.ap.ssid, cfg.ap.password[0] ? cfg.ap.password : nullptr,
                                cfg.ap.channel, cfg.ap.hidden, cfg.ap.maxClients);
  txPowerApplyOk = WiFi.setTxPower(static_cast<wifi_power_t>(cfg.ap.txPowerQuarterDbm));
  apRunning = ipOk && apOk && accessPointHealthy();
  lastApWatchdogMs = millis();
  return apRunning;
}

static bool applyStationIp() {
  if (cfg.sta.dhcp)
    return WiFi.STA.config(INADDR_NONE, INADDR_NONE, INADDR_NONE, INADDR_NONE);
  return WiFi.STA.config(toIp(cfg.sta.ip), toIp(cfg.sta.gateway), toIp(cfg.sta.subnet),
                         toIp(cfg.sta.dns1), toIp(cfg.sta.dns2));
}

static void beginStationAttempt() {
  lastStaAttemptMs = millis();
  if (cfg.sta.ssid[0] && (!cfg.sta.useCustomMac || macApplyOk))
    WiFi.begin(cfg.sta.ssid, cfg.sta.password[0] ? cfg.sta.password : nullptr);
}

static void applyStation() {
  WiFi.disconnect(false, false, 0);
  staConfigOk = applyStationIp();
  WiFi.setAutoReconnect(true);
  staRetryBackoffMs = 5000;
  if (staConfigOk) beginStationAttempt();
}

static void serviceNetwork(uint32_t now) {
  if (apApplyPending && !restartPending) {
    apApplyPending = false;
    WiFi.softAPdisconnect(false);
    startAccessPoint();
  }
  if (staApplyPending && !restartPending) {
    staApplyPending = false;
    applyStation();
  }
  if (!elapsed(now, lastNetworkServiceMs, NETWORK_PERIOD_MS)) return;
  lastNetworkServiceMs = now;

  const wl_status_t status = WiFi.status();
  if (status != previousStaStatus) {
    previousStaStatus = status;
    if (status == WL_CONNECTED) staRetryBackoffMs = 5000;
  }
  if (cfg.sta.ssid[0] && staConfigOk && (!cfg.sta.useCustomMac || macApplyOk) && status != WL_CONNECTED &&
      elapsed(now, lastStaAttemptMs, staRetryBackoffMs)) {
    beginStationAttempt();
    if (staRetryBackoffMs < 120000UL) {
      staRetryBackoffMs *= 2;
      if (staRetryBackoffMs > 120000UL) staRetryBackoffMs = 120000UL;
    }
  }
  if (elapsed(now, lastApWatchdogMs, AP_WATCHDOG_MS)) {
    lastApWatchdogMs = now;
    if (!accessPointHealthy()) apRunning = startAccessPoint();
    else apRunning = true;
    if (!dnsActive) {
      dnsActive = dnsServer.start(DNS_PORT, "*", AP_IP);
    }
  }
}

static void requestRestart(const char *reason, uint32_t delayMs = RESTART_RESPONSE_MS) {
  if (!restartPending || delayMs < restartDelayMs) restartDelayMs = delayMs;
  restartPending = true;
  restartRequestedMs = millis();
  restartReason = reason;
}

static void commonHeaders() {
  server.sendHeader("Cache-Control", "no-store");
  server.sendHeader("X-Content-Type-Options", "nosniff");
  server.sendHeader("X-Frame-Options", "DENY");
  server.sendHeader("Referrer-Policy", "no-referrer");
  server.sendHeader("Content-Security-Policy",
    "default-src 'self'; script-src 'self'; style-src 'self'; connect-src 'self'; "
    "img-src 'self' data:; base-uri 'none'; frame-ancestors 'none'; form-action 'self'");
}

static void sendProgmem(int code, const char *contentType, PGM_P content) {
  commonHeaders();
  server.send_P(code, contentType, content, strlen_P(content));
}

static void sendJson(int code, const char *json, size_t length) {
  commonHeaders();
  server.setContentLength(length);
  server.send(code, "application/json; charset=utf-8", "");
  server.sendContent(json, length);
}

static void sendMessage(int code, bool success, const char *message) {
  JsonWriter json{jsonBuffer, JSON_CAPACITY};
  json.raw("{\"ok\":"); json.raw(success ? "true" : "false"); json.raw(",\"message\":");
  json.string(message); json.raw("}");
  if (json.ok) sendJson(code, jsonBuffer, json.length);
}

static bool postAllowed() {
  if (!server.hasHeader("X-Requested-With") || server.header("X-Requested-With") != "XMLHttpRequest") {
    sendMessage(403, false, "Missing same-origin request header.");
    return false;
  }
  if (server.hasHeader("Host")) {
    String host = server.header("Host");
    host.trim();
    const int portSeparator = host.lastIndexOf(':');
    if (portSeparator > 0) host.remove(portSeparator);
    if (host.endsWith(".")) host.remove(host.length() - 1);
    host.toLowerCase();
    String localHostname = cfg.sta.hostname;
    localHostname.toLowerCase();
    const bool hostOk = host == AP_IP.toString() ||
                        (WiFi.localIP() != IPAddress(0, 0, 0, 0) && host == WiFi.localIP().toString()) ||
                        host == localHostname || host == localHostname + ".local";
    if (!hostOk) {
      sendMessage(403, false, "Management Host header is not a local controller address.");
      return false;
    }
  } else {
    sendMessage(403, false, "Missing HTTP Host header.");
    return false;
  }
  if (server.hasHeader("Content-Length")) {
    const long length = server.header("Content-Length").toInt();
    if (length < 0 || length > 4096) {
      sendMessage(413, false, "Request body too large.");
      return false;
    }
  }
  return true;
}

static bool parseLongArg(const char *name, long minimum, long maximum, long &output) {
  if (!server.hasArg(name)) return false;
  const String value = server.arg(name);
  if (!value.length() || strlen(value.c_str()) != value.length()) return false;
  char *end = nullptr;
  const long parsed = strtol(value.c_str(), &end, 10);
  if (*end != '\0' || parsed < minimum || parsed > maximum) return false;
  output = parsed;
  return true;
}

static bool parseFloatArg(const char *name, float &output) {
  if (!server.hasArg(name)) return false;
  const String value = server.arg(name);
  if (!value.length() || value.length() > 40 || strlen(value.c_str()) != value.length()) return false;
  char *end = nullptr;
  const float parsed = strtof(value.c_str(), &end);
  if (*end != '\0' || !isfinite(parsed)) return false;
  output = parsed;
  return true;
}

static bool copyArg(const char *name, char *output, size_t capacity, size_t maximum, bool allowEmpty = true) {
  if (!server.hasArg(name)) return false;
  const String value = server.arg(name);
  const size_t length = value.length();
  if (length > maximum || length >= capacity || (!allowEmpty && length == 0) ||
      strlen(value.c_str()) != length) return false;
  memset(output, 0, capacity);
  memcpy(output, value.c_str(), length + 1);
  return true;
}

static bool configError(const Config &candidate) {
  char error[128];
  if (validateConfig(candidate, error, sizeof(error))) return false;
  sendMessage(400, false, error);
  return true;
}

static void handleSensorConfig() {
  if (!postAllowed()) return;
  Config next;
  memset(&next, 0, sizeof(next));
  next = cfg;
  long number;
  if (!parseLongArg("address", 0x76, 0x77, number) || (number != 0x76 && number != 0x77)) {
    sendMessage(400, false, "Invalid BME280 address."); return;
  }
  next.sensor.address = static_cast<uint8_t>(number);
  if (!parseFloatArg("temperatureOffset", next.sensor.temperatureOffset) ||
      !parseFloatArg("humidityOffset", next.sensor.humidityOffset) ||
      !parseFloatArg("pressureOffset", next.sensor.pressureOffset) ||
      !parseLongArg("relayMode", 0, 2, number)) {
    sendMessage(400, false, "Missing or invalid sensor/relay field."); return;
  }
  next.relay.mode = static_cast<uint8_t>(number);
  if (!parseLongArg("source", 0, 2, number)) { sendMessage(400, false, "Invalid relay source."); return; }
  next.relay.source = static_cast<uint8_t>(number);
  if (!parseLongArg("condition", 0, 1, number)) { sendMessage(400, false, "Invalid relay condition."); return; }
  next.relay.condition = static_cast<uint8_t>(number);
  next.relay.activeHigh = server.hasArg("activeHigh") ? 1 : 0;
  if (!parseFloatArg("threshold", next.relay.threshold) ||
      !parseFloatArg("hysteresis", next.relay.hysteresis)) {
    sendMessage(400, false, "Invalid threshold or hysteresis."); return;
  }
  if (configError(next)) return;
  const bool sensorChanged = memcmp(&next.sensor, &cfg.sensor, sizeof(SensorConfig)) != 0;
  const bool polarityChanged = next.relay.activeHigh != cfg.relay.activeHigh;
  if (memcmp(&next, &cfg, sizeof(Config)) != 0) {
    cfg = next;
    markConfigDirty();
    if (sensorChanged) sensorApplyPending = true;
    if (polarityChanged) writeRelay(relayOn, "relay polarity updated", true);
  }
  sendMessage(200, true, "Sensor and relay settings saved; NVS commit is wear-deferred.");
}

static void handleRelayMode() {
  if (!postAllowed()) return;
  long mode;
  if (!parseLongArg("relayMode", 0, 2, mode)) {
    sendMessage(400, false, "Invalid relay mode."); return;
  }
  if (cfg.relay.mode != mode) {
    cfg.relay.mode = static_cast<uint8_t>(mode);
    markConfigDirty();
  }
  sendMessage(200, true, "Relay mode updated.");
}

static void handleApConfig() {
  if (!postAllowed()) return;
  Config next;
  memset(&next, 0, sizeof(next));
  next = cfg;
  long number;
  if (!copyArg("ssid", next.ap.ssid, sizeof(next.ap.ssid), 32, false) ||
      !parseLongArg("channel", 1, 13, number)) {
    sendMessage(400, false, "Invalid AP SSID or channel."); return;
  }
  next.ap.channel = static_cast<uint8_t>(number);
  if (!parseLongArg("maxClients", 1, 4, number)) {
    sendMessage(400, false, "Invalid AP client limit."); return;
  }
  next.ap.maxClients = static_cast<uint8_t>(number);
  if (!parseLongArg("txPower", -4, 78, number)) {
    sendMessage(400, false, "Invalid TX power."); return;
  }
  next.ap.txPowerQuarterDbm = static_cast<int8_t>(number);
  next.ap.hidden = server.hasArg("hidden") ? 1 : 0;
  if (server.hasArg("open")) {
    memset(next.ap.password, 0, sizeof(next.ap.password));
  } else if (server.hasArg("password") && server.arg("password").length()) {
    if (!copyArg("password", next.ap.password, sizeof(next.ap.password), 63, false)) {
      sendMessage(400, false, "Invalid AP password."); return;
    }
  }
  if (configError(next)) return;
  if (memcmp(&next.ap, &cfg.ap, sizeof(ApConfig)) != 0) {
    cfg = next;
    markConfigDirty();
    apApplyPending = true;
  }
  sendMessage(200, true, "AP settings saved. Connected AP clients will rejoin using the new settings.");
}

static bool copyIpArg(const char *name, uint8_t output[4], bool allowZero = false) {
  if (!server.hasArg(name)) return false;
  const String value = server.arg(name);
  if (allowZero && !value.length()) {
    memset(output, 0, 4);
    return true;
  }
  return parseIp(value, output);
}

static void handleStaConfig() {
  if (!postAllowed()) return;
  Config next;
  memset(&next, 0, sizeof(next));
  next = cfg;
  long number;
  if (!copyArg("ssid", next.sta.ssid, sizeof(next.sta.ssid), 32, true) ||
      !copyArg("hostname", next.sta.hostname, sizeof(next.sta.hostname), 32, false) ||
      !parseLongArg("ipMode", 0, 1, number)) {
    sendMessage(400, false, "Invalid STA SSID, hostname, or IP mode."); return;
  }
  next.sta.dhcp = static_cast<uint8_t>(number);
  if (server.hasArg("open")) {
    memset(next.sta.password, 0, sizeof(next.sta.password));
  } else if (server.hasArg("password") && server.arg("password").length()) {
    if (!copyArg("password", next.sta.password, sizeof(next.sta.password), 63, false)) {
      sendMessage(400, false, "Invalid STA password."); return;
    }
  }
  if (!next.sta.ssid[0]) memset(next.sta.password, 0, sizeof(next.sta.password));
  next.sta.useCustomMac = server.hasArg("useCustomMac") ? 1 : 0;
  if (next.sta.useCustomMac && server.hasArg("mac") && !parseMac(server.arg("mac"), next.sta.mac)) {
    sendMessage(400, false, "MAC must contain exactly six hexadecimal octets."); return;
  }
  if (!next.sta.dhcp) {
    if (!copyIpArg("ip", next.sta.ip) || !copyIpArg("gateway", next.sta.gateway) ||
        !copyIpArg("subnet", next.sta.subnet) || !copyIpArg("dns1", next.sta.dns1, true) ||
        !copyIpArg("dns2", next.sta.dns2, true)) {
      sendMessage(400, false, "Invalid static IPv4 field."); return;
    }
  }
  if (configError(next)) return;
  const bool restartNeeded = strcmp(next.sta.hostname, cfg.sta.hostname) != 0 ||
                             next.sta.useCustomMac != cfg.sta.useCustomMac ||
                             (next.sta.useCustomMac && memcmp(next.sta.mac, cfg.sta.mac, 6) != 0);
  if (memcmp(&next.sta, &cfg.sta, sizeof(StaConfig)) != 0) {
    cfg = next;
    markConfigDirty();
    if (restartNeeded) requestRestart("Applying hostname/custom MAC safely");
    else staApplyPending = true;
  }
  sendMessage(200, true, restartNeeded ?
    "Station settings saved. A short restart will safely apply hostname/MAC changes." :
    "Station settings saved; reconnection is in progress.");
}

static void handleReboot() {
  if (!postAllowed()) return;
  requestRestart("Manual restart requested");
  sendMessage(200, true, "Restart scheduled. Reconnect after a few seconds.");
}

static void handleFactoryReset() {
  if (!postAllowed()) return;
  if (!server.hasArg("confirm") || server.arg("confirm") != "FACTORY") {
    sendMessage(400, false, "Factory reset requires explicit confirmation."); return;
  }
  factoryPending = true;
  requestRestart("Factory reset", RESTART_RESPONSE_MS);
  sendMessage(200, true, "Factory reset scheduled. The default AP will return after restart.");
}

static uint8_t actualApChannel() {
  wifi_config_t config;
  if (esp_wifi_get_config(WIFI_IF_AP, &config) == ESP_OK) return config.ap.channel;
  return 0;
}

static bool staApSubnetConflict() {
  if (WiFi.status() != WL_CONNECTED) return false;
  const IPAddress address = WiFi.localIP();
  const IPAddress subnet = WiFi.subnetMask();
  const auto toU32 = [](const IPAddress &value) {
    return (static_cast<uint32_t>(value[0]) << 24) |
           (static_cast<uint32_t>(value[1]) << 16) |
           (static_cast<uint32_t>(value[2]) << 8) | value[3];
  };
  const uint32_t mask = toU32(subnet);
  const uint32_t network = toU32(address) & mask;
  const uint32_t apNetwork = 0xC0A80400UL;
  const uint32_t apMask = 0xFFFFFF00UL;
  return mask != 0 && (network & apMask) == (apNetwork & mask);
}

static void macString(JsonWriter &json, wifi_interface_t interface) {
  uint8_t address[6] = {0};
  if (esp_wifi_get_mac(interface, address) == ESP_OK) json.mac(address);
  else json.raw("null");
}

static void buildStateJson() {
  JsonWriter json{jsonBuffer, JSON_CAPACITY};
  const uint32_t now = millis();
  const wl_status_t staStatus = WiFi.status();
  const bool hasAge = sensorEverGood;
  const uint64_t age = hasAge ? monotonicMillis - lastGoodUptimeMs : 0;
  const uint32_t sinceAttempt = now - lastStaAttemptMs;
  const uint32_t retryRemaining = (!cfg.sta.ssid[0] || !staConfigOk || (cfg.sta.useCustomMac && !macApplyOk) || staStatus == WL_CONNECTED || sinceAttempt >= staRetryBackoffMs) ? 0 : staRetryBackoffMs - sinceAttempt;
  json.raw("{\"ok\":true,\"firmware\":\"" FIRMWARE_VERSION "\",\"uptimeMs\":");
  json.formatted("%llu", static_cast<unsigned long long>(monotonicMillis));
  json.raw(",\"sensor\":{\"present\":"); json.raw(sensorPresent ? "true" : "false");
  json.raw(",\"valid\":"); json.raw(sensorReadingValid ? "true" : "false");
  json.raw(",\"temperature\":"); json.nullableFloat(sensorReadingValid ? temperatureC : NAN, 3);
  json.raw(",\"humidity\":"); json.nullableFloat(sensorReadingValid ? humidityPct : NAN, 3);
  json.raw(",\"pressure\":"); json.nullableFloat(sensorReadingValid ? pressureHpa : NAN, 3);
  json.raw(",\"ageMs\":"); if (hasAge) json.formatted("%llu", static_cast<unsigned long long>(age)); else json.raw("null");
  json.formatted(",\"address\":%u,\"chipId\":%lu,\"errors\":%lu,\"temperatureOffset\":%.3f,\"humidityOffset\":%.3f,\"pressureOffset\":%.3f",
                 cfg.sensor.address, static_cast<unsigned long>(sensorChipId), static_cast<unsigned long>(sensorErrors),
                 cfg.sensor.temperatureOffset, cfg.sensor.humidityOffset, cfg.sensor.pressureOffset);
  json.raw("},\"relay\":{\"on\":"); json.raw(relayOn ? "true" : "false");
  json.formatted(",\"mode\":%u,\"source\":%u,\"condition\":%u,\"activeHigh\":%s,\"threshold\":%.4f,\"hysteresis\":%.4f,\"reason\":",
                 cfg.relay.mode, cfg.relay.source, cfg.relay.condition, cfg.relay.activeHigh ? "true" : "false",
                 cfg.relay.threshold, cfg.relay.hysteresis);
  json.string(relayReason);
  json.raw("},\"ap\":{\"running\":"); json.raw(apRunning ? "true" : "false");
  json.raw(",\"ip\":"); json.ip(WiFi.softAPIP());
  json.raw(",\"ssid\":"); json.string(cfg.ap.ssid);
  json.formatted(",\"hasPassword\":%s,\"hidden\":%s,\"channel\":%u,\"actualChannel\":%u,\"txPower\":%d,\"txPowerActual\":%d,\"txPowerApplyOk\":%s,\"clients\":%u,\"maxClients\":%u,\"mac\":",
                 cfg.ap.password[0] ? "true" : "false", cfg.ap.hidden ? "true" : "false",
                 cfg.ap.channel, actualApChannel(), cfg.ap.txPowerQuarterDbm,
                 static_cast<int>(WiFi.getTxPower()), txPowerApplyOk ? "true" : "false",
                 WiFi.softAPgetStationNum(), cfg.ap.maxClients);
  macString(json, WIFI_IF_AP);
  json.raw("},\"sta\":{\"configured\":"); json.raw(cfg.sta.ssid[0] ? "true" : "false");
  json.raw(",\"ssid\":"); json.string(cfg.sta.ssid);
  json.raw(",\"hostname\":"); json.string(cfg.sta.hostname);
  json.formatted(",\"hasPassword\":%s,\"dhcp\":%s,\"useCustomMac\":%s,\"mac\":",
                 cfg.sta.password[0] ? "true" : "false", cfg.sta.dhcp ? "true" : "false",
                 cfg.sta.useCustomMac ? "true" : "false");
  json.mac(cfg.sta.mac);
  json.raw(",\"actualMac\":"); macString(json, WIFI_IF_STA);
  json.formatted(",\"status\":%d,\"rssi\":%d,\"retryMs\":%lu,\"ipConfigOk\":%s,\"macApplyOk\":%s,\"subnetConflict\":%s,\"ip\":",
                 static_cast<int>(staStatus), staStatus == WL_CONNECTED ? WiFi.RSSI() : 0,
                 static_cast<unsigned long>(retryRemaining), staConfigOk ? "true" : "false",
                 macApplyOk ? "true" : "false", staApSubnetConflict() ? "true" : "false");
  json.ip(WiFi.localIP());
  json.raw(",\"gateway\":"); json.ip(WiFi.gatewayIP());
  json.raw(",\"subnet\":"); json.ip(WiFi.subnetMask());
  json.raw(",\"dns1Actual\":"); json.ip(WiFi.dnsIP(0));
  json.raw(",\"configuredIp\":"); json.ip(cfg.sta.ip);
  json.raw(",\"configuredGateway\":"); json.ip(cfg.sta.gateway);
  json.raw(",\"configuredSubnet\":"); json.ip(cfg.sta.subnet);
  json.raw(",\"configuredDns1\":"); json.ip(cfg.sta.dns1);
  json.raw(",\"configuredDns2\":"); json.ip(cfg.sta.dns2);
  json.raw("},\"system\":{\"heap\":"); json.formatted("%lu", static_cast<unsigned long>(ESP.getFreeHeap()));
  json.formatted(",\"minHeap\":%lu,\"maxAllocHeap\":%lu,\"storageOk\":%s,\"storageReady\":%s,\"configDirty\":%s,\"validSlots\":%u,\"generation\":%lu,\"saves\":%lu,\"activeSlot\":%d,\"dnsActive\":%s,\"restartPending\":%s,\"factoryLatched\":%s,\"resetReason\":%d,\"restartReason\":",
                 static_cast<unsigned long>(ESP.getMinFreeHeap()), static_cast<unsigned long>(ESP.getMaxAllocHeap()),
                 storageOk ? "true" : "false", preferencesReady ? "true" : "false", configDirty ? "true" : "false",
                 validSlotMask, static_cast<unsigned long>(activeGeneration), static_cast<unsigned long>(successfulSaves),
                 activeSlot, dnsActive ? "true" : "false", restartPending ? "true" : "false",
                 factoryResetLatched ? "true" : "false", static_cast<int>(esp_reset_reason()));
  json.string(restartReason);
  json.formatted(",\"sdk\":\"%s\",\"chipModel\":\"%s\",\"chipRevision\":%u}", ESP.getSdkVersion(), ESP.getChipModel(), ESP.getChipRevision());
  json.raw("}");
  if (!json.ok) {
    static const char overflow[] = "{\"ok\":false,\"message\":\"State response capacity exceeded\"}";
    sendJson(500, overflow, sizeof(overflow) - 1);
  } else {
    sendJson(200, jsonBuffer, json.length);
  }
}

static void redirectToPortal() {
  commonHeaders();
  server.sendHeader("Location", "http://192.168.4.1/");
  server.send(302, "text/plain; charset=utf-8", "Captive portal: http://192.168.4.1/\n");
}

static bool isStaManagementRequest() {
  if (!server.hasHeader("Host")) return false;
  String host = server.header("Host");
  host.trim();
  const int portSeparator = host.lastIndexOf(':');
  if (portSeparator > 0) host.remove(portSeparator);
  if (host.endsWith(".")) host.remove(host.length() - 1);
  host.toLowerCase();
  String hostname = cfg.sta.hostname;
  hostname.toLowerCase();
  return (WiFi.localIP() != IPAddress(0, 0, 0, 0) && host == WiFi.localIP().toString()) ||
         host == hostname || host == hostname + ".local";
}

static void handleNotFound() {
  if (server.uri().startsWith("/api/")) {
    sendMessage(404, false, "API endpoint not found.");
  } else if (isStaManagementRequest()) {
    commonHeaders();
    server.send(404, "text/plain; charset=utf-8", "Not found\n");
  } else {
    redirectToPortal();
  }
}

static const char APP_CSS[] PROGMEM = R"CSS(
:root{color-scheme:light;--bg:#f3f6fb;--panel:#fff;--panel2:#f8fafc;--text:#142033;--muted:#64748b;--line:#dbe3ee;--brand:#2563eb;--brand2:#1d4ed8;--good:#047857;--goodbg:#d1fae5;--bad:#b91c1c;--badbg:#fee2e2;--warn:#a16207;--warnbg:#fef3c7;--shadow:0 8px 30px rgba(30,64,175,.07)}
@media(prefers-color-scheme:dark){:root{color-scheme:dark;--bg:#0b1120;--panel:#111c2e;--panel2:#172337;--text:#e5edf8;--muted:#9aabc0;--line:#2b3b52;--brand:#60a5fa;--brand2:#3b82f6;--good:#6ee7b7;--goodbg:#073c35;--bad:#fca5a5;--badbg:#481d27;--warn:#fde047;--warnbg:#41340c;--shadow:none}}
*{box-sizing:border-box}html{scroll-behavior:smooth}body{margin:0;background:var(--bg);color:var(--text);font:15px/1.55 system-ui,-apple-system,BlinkMacSystemFont,"Segoe UI",sans-serif}button,input,select{font:inherit}button,a,input,select{touch-action:manipulation}a{color:var(--brand)}.shell{width:min(1180px,100% - 28px);margin:auto}.topbar{background:var(--panel);border-bottom:1px solid var(--line);position:sticky;top:0;z-index:10}.topbar-inner{min-height:70px;display:flex;align-items:center;justify-content:space-between;gap:14px}.brand{display:flex;align-items:center;gap:10px;text-decoration:none;color:var(--text);font-weight:750;line-height:1.15}.brand small{display:block;color:var(--muted);font-size:10px;letter-spacing:.13em;margin-top:5px}.brand-mark{display:grid;place-items:center;width:42px;height:42px;border-radius:13px;background:var(--brand);color:#fff;font-size:13px;letter-spacing:.03em}.connection{display:flex;align-items:center;gap:7px;color:var(--muted);font-size:12px;white-space:nowrap}.dot{width:9px;height:9px;border-radius:50%;background:var(--warn);display:inline-block;flex:none}.dot.good{background:var(--good)}.dot.bad{background:var(--bad)}nav{display:flex;gap:5px;overflow-x:auto;padding:9px 0;scrollbar-width:thin}nav a{flex:0 0 auto;text-decoration:none;color:var(--muted);font-weight:650;padding:9px 13px;border-radius:9px;min-height:42px}nav a:hover{background:var(--panel2);color:var(--text)}nav a.active{background:var(--brand);color:#fff}main{padding:26px 0 42px}.page-head{display:flex;align-items:flex-start;justify-content:space-between;gap:16px;margin-bottom:20px}h1{font-size:clamp(24px,4vw,34px);line-height:1.15;margin:0 0 7px;letter-spacing:-.025em}h2{font-size:18px;margin:0 0 15px}h3{font-size:15px;margin:22px 0 12px}.subtitle{color:var(--muted);margin:0;max-width:740px}.grid{display:grid;gap:16px}.grid.two{grid-template-columns:repeat(2,minmax(0,1fr))}.grid.three{grid-template-columns:repeat(3,minmax(0,1fr))}.grid.four{grid-template-columns:repeat(4,minmax(0,1fr))}.card{background:var(--panel);border:1px solid var(--line);border-radius:16px;padding:20px;box-shadow:var(--shadow);min-width:0}.card h2 .unit{font-size:12px;color:var(--muted);font-weight:500}.metric{position:relative;overflow:hidden}.metric-label{color:var(--muted);font-size:13px;font-weight:650}.metric-value{display:block;font-size:clamp(27px,4vw,39px);font-weight:760;line-height:1.25;margin:9px 0 6px;letter-spacing:-.035em;overflow-wrap:anywhere}.metric-note{font-size:12px;color:var(--muted);margin:0}.card-link{display:block;text-decoration:none;color:inherit;transition:border-color .15s,transform .15s}.card-link:hover{border-color:var(--brand);transform:translateY(-1px)}.section{margin-top:16px}.badge{display:inline-flex;align-items:center;gap:6px;padding:4px 9px;border-radius:999px;font-size:12px;font-weight:700;background:var(--panel2);color:var(--muted);white-space:nowrap}.badge.good{background:var(--goodbg);color:var(--good)}.badge.bad{background:var(--badbg);color:var(--bad)}.badge.warn{background:var(--warnbg);color:var(--warn)}.details{width:100%;border-collapse:collapse}.details th,.details td{text-align:left;padding:10px 4px;border-bottom:1px solid var(--line);vertical-align:top;overflow-wrap:anywhere}.details th{color:var(--muted);font-weight:550;width:45%}.details tr:last-child th,.details tr:last-child td{border:0}.details td{font-weight:650;text-align:right}.notice{border:1px solid var(--line);border-left:4px solid var(--brand);background:var(--panel2);border-radius:10px;padding:12px 14px;color:var(--muted);margin:14px 0}.notice.warn{border-left-color:var(--warn);background:var(--warnbg);color:var(--text)}.notice.danger{border-left-color:var(--bad);background:var(--badbg);color:var(--text)}form{margin:0}.form-grid{display:grid;grid-template-columns:repeat(2,minmax(0,1fr));gap:15px}.field{display:flex;flex-direction:column;gap:6px;min-width:0}.full{grid-column:1/-1}.field label,.label{font-weight:650;font-size:13px}.help{color:var(--muted);font-size:12px;margin:0}.input-wrap{display:flex;gap:7px}.input-wrap input{flex:1;min-width:0}input,select{width:100%;min-height:44px;border:1px solid var(--line);border-radius:9px;background:var(--panel2);color:var(--text);padding:9px 11px;outline:none}input:focus,select:focus,button:focus-visible,a:focus-visible{outline:3px solid var(--brand);outline:3px solid color-mix(in srgb,var(--brand) 35%,transparent);outline-offset:2px;border-color:var(--brand)}input:disabled,select:disabled{opacity:.55;cursor:not-allowed}.check{display:flex;align-items:flex-start;gap:10px;min-height:44px;padding:10px 0;cursor:pointer}.check input{appearance:none;width:21px;height:21px;min-height:21px;padding:0;margin:1px 0 0;border-radius:6px;display:grid;place-content:center;flex:none}.check input:checked{background:var(--brand);border-color:var(--brand)}.check input:checked::after{content:"✓";color:#fff;font-weight:800;font-size:15px}.check span{font-size:13px}.check small{display:block;color:var(--muted);font-weight:400;margin-top:2px}.actions{display:flex;align-items:center;flex-wrap:wrap;gap:10px;margin-top:20px}.btn{appearance:none;border:1px solid transparent;border-radius:9px;min-height:44px;padding:10px 16px;background:var(--brand);color:#fff;font-weight:700;cursor:pointer;text-decoration:none;display:inline-flex;align-items:center;justify-content:center;gap:7px}.btn:hover{background:var(--brand2)}.btn.secondary{background:var(--panel2);color:var(--text);border-color:var(--line)}.btn.secondary:hover{border-color:var(--brand)}.btn.danger{background:var(--bad);color:var(--badbg)}.btn.danger:hover{filter:brightness(1.1)}.btn.small{padding:6px 11px;min-height:38px;font-size:12px}.btn:disabled{opacity:.55;cursor:wait}.btn-group{display:flex;flex-wrap:wrap;gap:9px}.footer{color:var(--muted);font-size:12px;padding:20px 0 30px;text-align:center}.toast-region{position:fixed;right:16px;bottom:16px;z-index:50;display:grid;gap:8px;width:min(390px,calc(100% - 32px))}.toast{background:var(--text);color:var(--bg);padding:13px 15px;border-radius:11px;box-shadow:0 10px 35px #0004;animation:appear .2s ease-out}.toast.error{background:var(--bad);color:#fff}.toast.success{background:var(--good);color:var(--panel)}@keyframes appear{from{opacity:0;transform:translateY(8px)}}.divider{height:1px;background:var(--line);margin:20px 0}.danger-zone{border-color:var(--bad)}.mono{font-family:ui-monospace,SFMono-Regular,Consolas,monospace;font-size:.94em}.hidden{display:none!important}.restart-banner{display:none;background:var(--warnbg);color:var(--warn);padding:11px;text-align:center;font-weight:700}.restart-pending .restart-banner{display:block}.mobile-only{display:none}
@media(max-width:900px){.grid.four{grid-template-columns:repeat(2,minmax(0,1fr))}.grid.three{grid-template-columns:repeat(2,minmax(0,1fr))}}
@media(max-width:650px){.shell{width:min(100% - 20px,1180px)}.topbar-inner{min-height:62px}.brand small{display:none}.brand-mark{width:36px;height:36px;border-radius:10px}.connection span:last-child{display:none}main{padding-top:20px}.grid.two,.grid.three{grid-template-columns:1fr}.card{padding:16px;border-radius:13px}.form-grid{grid-template-columns:1fr}.field.full{grid-column:auto}.page-head{display:block}.page-head .badge{margin-top:10px}.details th{width:48%}.mobile-only{display:initial}.actions .btn{flex:1}.btn-group .btn{flex:1}.grid.four{gap:10px}.metric{padding:15px}.metric-value{font-size:26px}}
@media(max-width:370px){.grid.four{grid-template-columns:1fr}.brand>span:last-child{display:none}}
@media(prefers-reduced-motion:reduce){*{scroll-behavior:auto!important;animation:none!important;transition:none!important}}
)CSS";

static const char APP_JS[] PROGMEM = R"JS(
"use strict";
const $=(id)=>document.getElementById(id);
const page=document.body.dataset.page;
let latest=null,pollTimer=null,polling=false;
const populated=new Set();
const path=(obj,key)=>key.split(".").reduce((v,k)=>v==null?undefined:v[k],obj);
const text=(id,value)=>{const e=$(id);if(e)e.textContent=value;};
const set=(id,value)=>{const e=$(id);if(e)e.value=value??"";};
const check=(id,value)=>{const e=$(id);if(e)e.checked=!!value;};
const wifiStates={0:"Idle",1:"SSID unavailable",2:"Scan complete",3:"Connected",4:"Connection failed",5:"Connection lost",6:"Disconnected",255:"No Wi-Fi shield"};
function duration(ms){if(ms==null||!Number.isFinite(Number(ms)))return"—";let s=Math.max(0,Math.floor(Number(ms)/1000));const d=Math.floor(s/86400);s%=86400;const h=Math.floor(s/3600);s%=3600;const m=Math.floor(s/60);s%=60;return(d?d+"d ":"")+(d||h?h+"h ":"")+m+"m "+s+"s";}
function age(ms){if(ms==null)return"Never";if(ms<1500)return"Just now";if(ms<60000)return(Math.floor(ms/1000)+" s ago");return duration(ms)+" ago";}
function remaining(ms){if(!ms)return"Now / not pending";if(ms<1000)return"Under 1 s";return duration(ms);}
function bytes(v){if(v==null)return"—";return Number(v).toLocaleString()+" B";}
const formatters={temp:v=>v==null?"—":Number(v).toFixed(2)+" °C",hum:v=>v==null?"—":Number(v).toFixed(2)+" %",pressure:v=>v==null?"—":Number(v).toFixed(2)+" hPa",age,remaining,uptime:duration,bytes,bool:v=>v?"Yes":"No",onoff:v=>v?"ON":"OFF",wifi:v=>wifiStates[v]||("Status "+v),power:v=>(Number(v)/4).toFixed(Number(v)%4?2:0)+" dBm",address:v=>"0x"+Number(v).toString(16).toUpperCase(),chip:v=>v?"0x"+Number(v).toString(16).toUpperCase():"—",relaymode:v=>({0:"Automatic",1:"Manual OFF",2:"Manual ON"}[v]||"Unknown"),source:v=>({0:"Temperature",1:"Humidity",2:"Pressure"}[v]||"Unknown"),rssi:v=>v?Number(v)+" dBm":"—"};
function badge(id,value,label){const e=$(id);if(!e)return;e.textContent=label;e.className="badge "+value;}
function renderBindings(s){document.querySelectorAll("[data-bind]").forEach(e=>{const v=path(s,e.dataset.bind);const f=formatters[e.dataset.format];e.textContent=f?f(v,s):(v==null?"—":String(v));});}
function render(s){
 renderBindings(s);
 badge("sensorBadge",s.sensor.present&&s.sensor.valid?"good":"bad",s.sensor.present?(s.sensor.valid?"Sensor online":"Sensor data invalid"):"Sensor not found");
 badge("relayBadge",s.relay.on?"warn":"good",s.relay.on?"Relay energized":"Relay off");
 badge("apBadge",s.ap.running?"good":"bad",s.ap.running?"AP active":"AP fault");
 const staFault=s.sta.configured&&(!s.sta.ipConfigOk||!s.sta.macApplyOk);
 const staConflict=!!s.sta.subnetConflict;
 const staLabel=staConflict?"STA/AP subnet conflict":(s.sta.status===3?"STA connected":(staFault?"STA configuration fault":(s.sta.configured?"STA reconnecting":"STA not configured")));
 badge("staBadge",staConflict?"bad":(s.sta.status===3?"good":(staFault?"bad":(s.sta.configured?"warn":""))),staLabel);
 badge("storageBadge",s.system.storageOk?(s.system.configDirty?"warn":"good"):"bad",s.system.storageOk?(s.system.configDirty?"Commit pending":"Storage healthy"):"Storage fault");
 text("connectionText","Controller online");$("connectionDot")?.classList.remove("bad");$("connectionDot")?.classList.add("good");
 const sensorTitle=!s.sensor.present?"Check 3.3 V power, GPIO21/SDA, GPIO22/SCL and I²C address.":(s.sensor.valid?"Readings update every second.":"Automatic relay control is OFF until valid data returns.");
 document.querySelectorAll("[data-sensor-note]").forEach(e=>e.textContent=sensorTitle);
 updateUnits();populateForms(s);
}
function populateForms(s){
 if(page==="sensor")populateOnce("sensorForm",()=>{set("address",s.sensor.address);set("temperatureOffset",s.sensor.temperatureOffset);set("humidityOffset",s.sensor.humidityOffset);set("pressureOffset",s.sensor.pressureOffset);set("relayMode",s.relay.mode);set("source",s.relay.source);set("condition",s.relay.condition);set("threshold",s.relay.threshold);set("hysteresis",s.relay.hysteresis);check("activeHigh",s.relay.activeHigh);updateUnits();});
 if(page==="ap")populateOnce("apForm",()=>{set("ssid",s.ap.ssid);set("channel",s.ap.channel);set("maxClients",s.ap.maxClients);set("txPower",s.ap.txPower);check("hidden",s.ap.hidden);check("open",!s.ap.hasPassword);});
 if(page==="station")populateOnce("stationForm",()=>{set("ssid",s.sta.ssid);set("hostname",s.sta.hostname);set("ipMode",s.sta.dhcp?1:0);set("ip",path(s,"sta.configuredIp"));set("gateway",path(s,"sta.configuredGateway"));set("subnet",path(s,"sta.configuredSubnet"));set("dns1",path(s,"sta.configuredDns1"));set("dns2",path(s,"sta.configuredDns2"));set("mac",s.sta.mac);check("open",!s.sta.hasPassword);check("useCustomMac",s.sta.useCustomMac);updateNetworkFields();});
}
function populateOnce(id,fn){if(!populated.has(id)&&$(id)&&!$(id).dataset.dirty){fn();populated.add(id);}}
function toast(message,error=false){const region=$("toastRegion");if(!region)return;const e=document.createElement("div");e.className="toast "+(error?"error":"success");e.setAttribute("role",error?"alert":"status");e.textContent=message;region.appendChild(e);setTimeout(()=>e.remove(),error?8500:5000);}
async function request(url,options={}){
 const controller=new AbortController();const timeout=setTimeout(()=>controller.abort(),options.timeout||7000);
 try{const response=await fetch(url,{cache:"no-store",credentials:"same-origin",...options,signal:controller.signal,headers:{"X-Requested-With":"XMLHttpRequest",...(options.headers||{})}});const body=await response.text();let data;try{data=JSON.parse(body);}catch{throw new Error("Controller returned an unreadable response (HTTP "+response.status+").");}if(!response.ok||!data.ok)throw new Error(data.message||("HTTP "+response.status));return data;}finally{clearTimeout(timeout);}
}
async function poll(){if(polling)return;if(document.hidden){schedule(2500);return;}polling=true;try{const s=await request("/api/state",{timeout:5000});latest=s;render(s);}catch(error){text("connectionText","Controller unreachable");$("connectionDot")?.classList.remove("good");$("connectionDot")?.classList.add("bad");badge("sensorBadge","bad","Status unavailable");}finally{polling=false;schedule(1200);}}
function schedule(delay=200){clearTimeout(pollTimer);pollTimer=setTimeout(poll,delay);}
function updateUnits(){const source=$("source");if(!source)return;const units=["°C","%","hPa"],ranges=[[-40,85,.1,20],[0,100,.1,50],[300,1100,.1,100]];const i=Number(source.value);document.querySelectorAll("[data-threshold-unit]").forEach(e=>e.textContent=units[i]);for(const id of["threshold","hysteresis"]){const e=$(id);if(e){e.min=id==="hysteresis"?0:ranges[i][0];e.max=id==="hysteresis"?ranges[i][3]:ranges[i][1];e.step=ranges[i][2];}}}
function updateNetworkFields(){const dhcp=$("ipMode")?.value==="1";document.querySelectorAll("[data-static-ip]").forEach(e=>{e.disabled=dhcp;e.required=!dhcp;});const custom=$("useCustomMac")?.checked;if($("mac"))$("mac").disabled=!custom;}
async function submitForm(form){const button=form.querySelector("button[type=submit]");button&&(button.disabled=true);try{const body=new URLSearchParams(new FormData(form));const result=await request(form.dataset.endpoint,{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:body.toString(),timeout:9000});delete form.dataset.dirty;populated.delete(form.id);form.querySelectorAll("input[type=password]").forEach(e=>e.value="");toast(result.message);await poll();}catch(error){toast(error.name==="AbortError"?"Request timed out. Changes may have applied; reconnect and verify.":error.message,true);}finally{button&&(button.disabled=false);schedule(300);}}
document.querySelectorAll("form[data-endpoint]").forEach(form=>{form.addEventListener("submit",e=>{e.preventDefault();if(!form.reportValidity())return;submitForm(form);});form.addEventListener("input",()=>form.dataset.dirty="1");});
document.querySelectorAll("[data-relay-mode]").forEach(button=>button.addEventListener("click",async()=>{button.disabled=true;try{const r=await request("/api/relay",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:"relayMode="+button.dataset.relayMode});toast(r.message);await poll();}catch(e){toast(e.message,true);}finally{button.disabled=false;}}));
document.querySelectorAll("[data-toggle-password]").forEach(button=>button.addEventListener("click",()=>{const input=$(button.dataset.togglePassword);if(!input)return;const show=input.type==="password";input.type=show?"text":"password";button.textContent=show?"Hide":"Show";button.setAttribute("aria-label",(show?"Hide":"Show")+" password");}));
$("source")?.addEventListener("change",updateUnits);$("ipMode")?.addEventListener("change",updateNetworkFields);$("useCustomMac")?.addEventListener("change",updateNetworkFields);
document.querySelectorAll(".password-field").forEach(input=>input.addEventListener("input",()=>{if(input.value){const open=$("open");if(open)open.checked=false;}}));
document.querySelectorAll("[data-system-action]").forEach(button=>button.addEventListener("click",async()=>{const factory=button.dataset.systemAction==="factory";const message=factory?"Erase ALL controller settings and restore the default AP, sensor and relay configuration? This cannot be undone.":"Restart the controller now? Active connections will briefly drop.";if(!window.confirm(message))return;button.disabled=true;try{const r=await request(factory?"/api/factory":"/api/reboot",{method:"POST",headers:{"Content-Type":"application/x-www-form-urlencoded"},body:factory?"confirm=FACTORY":""});toast(r.message);document.body.classList.add("restart-pending");}catch(e){toast(e.message,true);button.disabled=false;}}));
document.querySelectorAll("a[href]").forEach(a=>a.addEventListener("click",e=>{const dirty=[...document.querySelectorAll("form[data-dirty]")];if(dirty.length&&!window.confirm("Leave this page and discard unsaved form edits?"))e.preventDefault();}));
window.addEventListener("beforeunload",e=>{if(document.querySelector("form[data-dirty]")){e.preventDefault();e.returnValue="";}});
document.addEventListener("visibilitychange",()=>{if(!document.hidden)schedule(0);});
updateNetworkFields();updateUnits();poll();
)JS";

static const char PAGE_DASHBOARD[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#2563eb"><title>Overview · ESP32 Environment</title><link rel="stylesheet" href="/app.css"><script src="/app.js" defer></script></head><body data-page="dashboard"><div class="restart-banner">Restart pending — reconnect if this page stops responding.</div><header class="topbar"><div class="shell topbar-inner"><a class="brand" href="/"><span class="brand-mark">EC</span><span>ESP32 Environment<small>LOCAL CONTROLLER</small></span></a><div class="connection"><span class="dot" id="connectionDot"></span><span id="connectionText">Connecting…</span></div></div><div class="shell"><nav aria-label="Main navigation"><a class="active" href="/">Overview</a><a href="/sensor">Sensor & relay</a><a href="/ap">Wi-Fi AP</a><a href="/station">Wi-Fi station</a><a href="/system">System</a></nav></div></header><main class="shell"><div class="page-head"><div><h1>Environment overview</h1><p class="subtitle">Live BME280 measurements, relay state and network health. All data stays on this controller.</p></div><span class="badge" id="sensorBadge">Reading status…</span></div><section class="grid three" aria-label="Live sensor readings"><article class="card metric"><span class="metric-label">Temperature</span><strong class="metric-value" data-bind="sensor.temperature" data-format="temp">—</strong><p class="metric-note">Calibrated BME280 reading</p></article><article class="card metric"><span class="metric-label">Relative humidity</span><strong class="metric-value" data-bind="sensor.humidity" data-format="hum">—</strong><p class="metric-note">Calibrated BME280 reading</p></article><article class="card metric"><span class="metric-label">Barometric pressure</span><strong class="metric-value" data-bind="sensor.pressure" data-format="pressure">—</strong><p class="metric-note">Station pressure, not sea-level adjusted</p></article></section><div class="notice" data-sensor-note>Waiting for sensor status…</div><section class="grid two section"><article class="card"><div class="page-head"><div><h2>Relay control · GPIO25</h2><p class="subtitle" data-bind="relay.reason">—</p></div><span class="badge" id="relayBadge">—</span></div><table class="details"><tr><th>Output</th><td data-bind="relay.on" data-format="onoff">—</td></tr><tr><th>Mode</th><td data-bind="relay.mode" data-format="relaymode">—</td></tr><tr><th>Threshold source</th><td data-bind="relay.source" data-format="source">—</td></tr><tr><th>Active level</th><td><span data-bind="relay.activeHigh" data-format="bool">—</span> (Yes = active HIGH)</td></tr></table><div class="btn-group" aria-label="Quick relay mode"><button class="btn secondary" type="button" data-relay-mode="0">Automatic</button><button class="btn secondary" type="button" data-relay-mode="1">Force OFF</button><button class="btn danger" type="button" data-relay-mode="2">Force ON</button></div><div class="actions"><a class="btn secondary" href="/sensor">Threshold settings</a></div></article><article class="card"><h2>Network & controller</h2><table class="details"><tr><th>Access point</th><td><span data-bind="ap.ssid">—</span> · <span data-bind="ap.clients">—</span> clients</td></tr><tr><th>AP address</th><td class="mono" data-bind="ap.ip">—</td></tr><tr><th>Station</th><td data-bind="sta.status" data-format="wifi">—</td></tr><tr><th>Station address</th><td class="mono" data-bind="sta.ip">—</td></tr><tr><th>Uptime</th><td data-bind="uptimeMs" data-format="uptime">—</td></tr><tr><th>Free heap</th><td data-bind="system.heap" data-format="bytes">—</td></tr><tr><th>Storage</th><td><span data-bind="system.validSlots">—</span> valid slot mask · gen <span data-bind="system.generation">—</span></td></tr></table><div class="actions"><a class="btn secondary" href="/ap">Manage AP</a><a class="btn secondary" href="/station">Manage station</a><a class="btn secondary" href="/system">Diagnostics</a></div></article></section></main><footer class="footer">ESP32 Environment Controller · <span data-bind="firmware">—</span> · Last sample: <span data-bind="sensor.ageMs" data-format="age">—</span></footer><div class="toast-region" id="toastRegion" aria-live="polite"></div></body></html>)HTML";

static const char PAGE_SENSOR[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#2563eb"><title>Sensor & relay · ESP32 Environment</title><link rel="stylesheet" href="/app.css"><script src="/app.js" defer></script></head><body data-page="sensor"><div class="restart-banner">Restart pending — reconnect if this page stops responding.</div><header class="topbar"><div class="shell topbar-inner"><a class="brand" href="/"><span class="brand-mark">EC</span><span>ESP32 Environment<small>LOCAL CONTROLLER</small></span></a><div class="connection"><span class="dot" id="connectionDot"></span><span id="connectionText">Connecting…</span></div></div><div class="shell"><nav aria-label="Main navigation"><a href="/">Overview</a><a class="active" href="/sensor">Sensor & relay</a><a href="/ap">Wi-Fi AP</a><a href="/station">Wi-Fi station</a><a href="/system">System</a></nav></div></header><main class="shell"><div class="page-head"><div><h1>BME280 & relay</h1><p class="subtitle">I²C SDA GPIO21 · SCL GPIO22 · 100 kHz. Relay output uses GPIO25 through a suitable driver.</p></div><span class="badge" id="sensorBadge">Reading status…</span></div><section class="grid three"><article class="card metric"><span class="metric-label">Temperature</span><strong class="metric-value" data-bind="sensor.temperature" data-format="temp">—</strong><p class="metric-note">Last sample: <span data-bind="sensor.ageMs" data-format="age">—</span></p></article><article class="card metric"><span class="metric-label">Humidity</span><strong class="metric-value" data-bind="sensor.humidity" data-format="hum">—</strong><p class="metric-note">BME280 chip ID: <span data-bind="sensor.chipId" data-format="chip">—</span></p></article><article class="card metric"><span class="metric-label">Pressure</span><strong class="metric-value" data-bind="sensor.pressure" data-format="pressure">—</strong><p class="metric-note">Read errors: <span data-bind="sensor.errors">—</span></p></article></section><div class="notice" data-sensor-note>Waiting for sensor status…</div><form id="sensorForm" data-endpoint="/api/config/sensor" class="card section"><h2>Sensor calibration</h2><div class="form-grid"><div class="field"><label for="address">I²C address</label><select id="address" name="address"><option value="118">0x76 (SDO to GND)</option><option value="119">0x77 (SDO to 3.3 V)</option></select></div><div class="field"><label for="temperatureOffset">Temperature offset (°C)</label><input id="temperatureOffset" name="temperatureOffset" type="number" min="-20" max="20" step="0.01" required><p class="help">Range −20 to +20 °C; use a trusted reference.</p></div><div class="field"><label for="humidityOffset">Humidity offset (% RH)</label><input id="humidityOffset" name="humidityOffset" type="number" min="-20" max="20" step="0.01" required></div><div class="field"><label for="pressureOffset">Pressure offset (hPa)</label><input id="pressureOffset" name="pressureOffset" type="number" min="-50" max="50" step="0.01" required><p class="help">Offsets are added to each raw reading.</p></div></div><div class="divider"></div><div class="page-head"><div><h2>Relay automation</h2><p class="subtitle">Hysteresis prevents rapid switching near the threshold. Automatic mode always fails OFF if data is invalid or older than five seconds.</p></div><span class="badge" id="relayBadge">—</span></div><div class="form-grid"><div class="field"><label for="relayMode">Operating mode</label><select id="relayMode" name="relayMode"><option value="0">Automatic threshold control</option><option value="1">Manual forced OFF</option><option value="2">Manual forced ON</option></select><p class="help">Mode persists across restart. Use manual ON with appropriate load safety.</p></div><div class="field"><label for="source">Threshold sensor</label><select id="source" name="source"><option value="0">Temperature</option><option value="1">Humidity</option><option value="2">Pressure</option></select></div><div class="field"><label for="condition">Trigger condition</label><select id="condition" name="condition"><option value="0">Energize at/above threshold</option><option value="1">Energize at/below threshold</option></select></div><div class="field"><label for="threshold">Threshold (<span data-threshold-unit>°C</span>)</label><input id="threshold" name="threshold" type="number" step="0.1" required></div><div class="field"><label for="hysteresis">Hysteresis (<span data-threshold-unit>°C</span>)</label><input id="hysteresis" name="hysteresis" type="number" min="0" step="0.1" required><p class="help">Above: releases below threshold − hysteresis. Below: releases above threshold + hysteresis. Exact release-boundary readings hold their current state.</p></div><div class="field"><span class="label">Relay electrical polarity</span><label class="check"><input id="activeHigh" name="activeHigh" type="checkbox" value="1"><span>Active HIGH<small>Checked: HIGH energizes. Unchecked: LOW energizes. Boot always drives the inactive level.</small></span></label></div></div><div class="notice warn"><strong>Current output: <span data-bind="relay.on" data-format="onoff">—</span></strong><br><span data-bind="relay.reason">—</span>. Never connect a relay coil directly to GPIO25; use a transistor/MOSFET or proper relay module, flyback protection and a suitable supply.</div><div class="actions"><button class="btn" type="submit">Save sensor & relay settings</button></div></form></main><footer class="footer">Sensor readings are compensated by the BME280, then adjusted by your offsets.</footer><div class="toast-region" id="toastRegion" aria-live="polite"></div></body></html>)HTML";

static const char PAGE_AP[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#2563eb"><title>Wi-Fi AP · ESP32 Environment</title><link rel="stylesheet" href="/app.css"><script src="/app.js" defer></script></head><body data-page="ap"><div class="restart-banner">Restart pending — reconnect if this page stops responding.</div><header class="topbar"><div class="shell topbar-inner"><a class="brand" href="/"><span class="brand-mark">EC</span><span>ESP32 Environment<small>LOCAL CONTROLLER</small></span></a><div class="connection"><span class="dot" id="connectionDot"></span><span id="connectionText">Connecting…</span></div></div><div class="shell"><nav aria-label="Main navigation"><a href="/">Overview</a><a href="/sensor">Sensor & relay</a><a class="active" href="/ap">Wi-Fi AP</a><a href="/station">Wi-Fi station</a><a href="/system">System</a></nav></div></header><main class="shell"><div class="page-head"><div><h1>Wi-Fi access point</h1><p class="subtitle">The AP runs concurrently with station mode and is supervised every ten seconds. Captive DNS and HTTP detection route to this dashboard.</p></div><span class="badge" id="apBadge">—</span></div><section class="grid two"><article class="card"><h2>Live AP status</h2><table class="details"><tr><th>SSID</th><td data-bind="ap.ssid">—</td></tr><tr><th>Portal address</th><td class="mono" data-bind="ap.ip">—</td></tr><tr><th>Configured / actual channel</th><td><span data-bind="ap.channel">—</span> / <span data-bind="ap.actualChannel">—</span></td></tr><tr><th>TX power configured / actual</th><td><span data-bind="ap.txPower" data-format="power">—</span> / <span data-bind="ap.txPowerActual" data-format="power">—</span></td></tr><tr><th>TX power accepted</th><td data-bind="ap.txPowerApplyOk" data-format="bool">—</td></tr><tr><th>Connected clients</th><td><span data-bind="ap.clients">—</span> / <span data-bind="ap.maxClients">—</span></td></tr><tr><th>AP MAC</th><td class="mono" data-bind="ap.mac">—</td></tr><tr><th>Hidden SSID</th><td data-bind="ap.hidden" data-format="bool">—</td></tr><tr><th>Captive DNS</th><td data-bind="system.dnsActive" data-format="bool">—</td></tr></table></article><article class="card"><h2>Important radio behavior</h2><div class="notice">AP and STA share one 2.4 GHz radio. When the station connects on another channel, the AP follows the station channel. Keep both on one channel for predictable AP visibility.</div><div class="notice warn">A hidden SSID cannot be discovered from a normal network list. Join it manually before captive detection can work. Hiding an SSID is not a security control.</div><p class="help">The captive portal supports common Android, Apple, Windows and Firefox probes. VPNs, HTTPS-only apps and cached connectivity state can require opening <span class="mono">http://192.168.4.1</span> manually.</p></article></section><form id="apForm" data-endpoint="/api/config/ap" class="card section"><h2>AP configuration</h2><div class="form-grid"><div class="field full"><label for="ssid">AP SSID</label><input id="ssid" name="ssid" type="text" maxlength="32" required autocomplete="off"><p class="help">1–32 bytes. Emoji/non-ASCII characters use multiple UTF-8 bytes; the server enforces the byte limit.</p></div><div class="field full"><label for="password">New WPA2 password</label><div class="input-wrap"><input class="password-field" id="password" name="password" type="password" minlength="8" maxlength="63" autocomplete="new-password" placeholder="Leave blank to keep current password"><button class="btn secondary" type="button" data-toggle-password="password">Show</button></div><p class="help">8–63 printable ASCII characters. Existing passwords are never returned by the API.</p></div><label class="check full"><input id="open" name="open" type="checkbox" value="1"><span>Open network / clear AP password<small>Not recommended on an untrusted network. Checking this overrides the password field.</small></span></label><div class="field"><label for="channel">2.4 GHz channel</label><select id="channel" name="channel"><option value="1">1 · 2412 MHz</option><option value="2">2 · 2417 MHz</option><option value="3">3 · 2422 MHz</option><option value="4">4 · 2427 MHz</option><option value="5">5 · 2432 MHz</option><option value="6">6 · 2437 MHz</option><option value="7">7 · 2442 MHz</option><option value="8">8 · 2447 MHz</option><option value="9">9 · 2452 MHz</option><option value="10">10 · 2457 MHz</option><option value="11">11 · 2462 MHz</option><option value="12">12 · 2467 MHz</option><option value="13">13 · 2472 MHz</option></select><p class="help">Channels 12–13 depend on local regulatory support.</p></div><div class="field"><label for="txPower">Maximum TX power</label><select id="txPower" name="txPower"><option value="-4">−1 dBm</option><option value="8">2 dBm</option><option value="20">5 dBm</option><option value="28">7 dBm</option><option value="34">8.5 dBm</option><option value="44">11 dBm</option><option value="52">13 dBm</option><option value="60">15 dBm</option><option value="68">17 dBm</option><option value="74">18.5 dBm</option><option value="76">19 dBm</option><option value="78">19.5 dBm</option></select><p class="help">ESP32 radio-wide maximum; actual output varies with hardware and rate.</p></div><div class="field"><label for="maxClients">Maximum AP clients</label><select id="maxClients" name="maxClients"><option value="1">1 client</option><option value="2">2 clients</option><option value="3">3 clients</option><option value="4">4 clients</option></select></div><div class="field"><span class="label">SSID visibility</span><label class="check"><input id="hidden" name="hidden" type="checkbox" value="1"><span>Hide SSID<small>Clients must enter the exact SSID manually.</small></span></label></div></div><div class="notice warn">Saving AP settings briefly disconnects AP clients. If the SSID/password changes, reconnect with the new credentials and open http://192.168.4.1/.</div><div class="actions"><button class="btn" type="submit">Save & apply AP settings</button></div></form></main><footer class="footer">Default AP: ESP32-ENV-XXXX · password: configure123 · address: 192.168.4.1</footer><div class="toast-region" id="toastRegion" aria-live="polite"></div></body></html>)HTML";

static const char PAGE_STATION[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#2563eb"><title>Wi-Fi station · ESP32 Environment</title><link rel="stylesheet" href="/app.css"><script src="/app.js" defer></script></head><body data-page="station"><div class="restart-banner">Restart pending — reconnect if this page stops responding.</div><header class="topbar"><div class="shell topbar-inner"><a class="brand" href="/"><span class="brand-mark">EC</span><span>ESP32 Environment<small>LOCAL CONTROLLER</small></span></a><div class="connection"><span class="dot" id="connectionDot"></span><span id="connectionText">Connecting…</span></div></div><div class="shell"><nav aria-label="Main navigation"><a href="/">Overview</a><a href="/sensor">Sensor & relay</a><a href="/ap">Wi-Fi AP</a><a class="active" href="/station">Wi-Fi station</a><a href="/system">System</a></nav></div></header><main class="shell"><div class="page-head"><div><h1>Wi-Fi station</h1><p class="subtitle">The STA interface remains active in AP+STA mode. Configured connections retry forever with bounded 5–120 second backoff.</p></div><span class="badge" id="staBadge">—</span></div><section class="card"><h2>Live station status</h2><div class="grid two"><table class="details"><tr><th>Connection</th><td data-bind="sta.status" data-format="wifi">—</td></tr><tr><th>Signal</th><td data-bind="sta.rssi" data-format="rssi">—</td></tr><tr><th>Actual IPv4</th><td class="mono" data-bind="sta.ip">—</td></tr><tr><th>Actual gateway</th><td class="mono" data-bind="sta.gateway">—</td></tr><tr><th>Actual subnet</th><td class="mono" data-bind="sta.subnet">—</td></tr></table><table class="details"><tr><th>Active MAC</th><td class="mono" data-bind="sta.actualMac">—</td></tr><tr><th>Hostname</th><td data-bind="sta.hostname">—</td></tr><tr><th>Actual DNS 1</th><td class="mono" data-bind="sta.dns1Actual">—</td></tr><tr><th>Next retry</th><td data-bind="sta.retryMs" data-format="remaining">—</td></tr><tr><th>IP configuration accepted</th><td data-bind="sta.ipConfigOk" data-format="bool">—</td></tr><tr><th>Boot MAC applied</th><td data-bind="sta.macApplyOk" data-format="bool">—</td></tr><tr><th>AP/STA subnet overlap</th><td data-bind="sta.subnetConflict" data-format="bool">—</td></tr></table></div><p class="help">Local hostname access: <span class="mono">http://<span data-bind="sta.hostname">esp32-environment</span>.local/</span> when your client supports mDNS.</p></section><form id="stationForm" data-endpoint="/api/config/station" class="card section"><h2>Upstream network</h2><div class="form-grid"><div class="field full"><label for="ssid">Router / upstream SSID</label><input id="ssid" name="ssid" type="text" maxlength="32" autocomplete="off"><p class="help">Leave SSID empty and select “open/clear password” to stop requesting a STA connection. The STA interface itself stays active.</p></div><div class="field full"><label for="password">New upstream password</label><div class="input-wrap"><input class="password-field" id="password" name="password" type="password" minlength="8" maxlength="63" autocomplete="new-password" placeholder="Leave blank to keep current password"><button class="btn secondary" type="button" data-toggle-password="password">Show</button></div></div><label class="check full"><input id="open" name="open" type="checkbox" value="1"><span>Open network / clear saved password<small>Also use this when clearing the SSID. WPA-Enterprise and 64-character raw PSKs are not supported.</small></span></label><div class="field"><label for="hostname">Device hostname</label><input id="hostname" name="hostname" type="text" maxlength="32" pattern="[A-Za-z0-9]([A-Za-z0-9-]{0,30}[A-Za-z0-9])?" required><p class="help">1–32 letters, digits or hyphens; no leading/trailing hyphen.</p></div><div class="field"><label for="ipMode">IPv4 configuration</label><select id="ipMode" name="ipMode"><option value="1">Dynamic (DHCP)</option><option value="0">Static IPv4</option></select></div></div><h3>Static IPv4 settings</h3><div class="form-grid"><div class="field"><label for="ip">Static IP address</label><input data-static-ip id="ip" name="ip" type="text" inputmode="decimal" placeholder="192.168.1.50" required></div><div class="field"><label for="gateway">Gateway</label><input data-static-ip id="gateway" name="gateway" type="text" inputmode="decimal" placeholder="192.168.1.1" required></div><div class="field"><label for="subnet">Subnet mask</label><input data-static-ip id="subnet" name="subnet" type="text" inputmode="decimal" placeholder="255.255.255.0" required><p class="help">Contiguous /1 through /30; gateway must be in the same subnet.</p></div><div class="field"><label for="dns1">DNS server 1 (optional)</label><input data-static-ip id="dns1" name="dns1" type="text" inputmode="decimal" placeholder="1.1.1.1"></div><div class="field"><label for="dns2">DNS server 2 (optional)</label><input data-static-ip id="dns2" name="dns2" type="text" inputmode="decimal" placeholder="8.8.8.8"></div></div><div class="notice warn">A wrong static IP, mask or gateway can make STA access unreachable. Static subnets overlapping the recovery AP (192.168.4.0/24) are rejected. Avoid IP conflicts and DHCP-reserved addresses. If a DHCP lease overlaps the AP subnet, the live status will report it; change the upstream LAN subnet.</div><h3>Hardware address</h3><label class="check"><input id="useCustomMac" name="useCustomMac" type="checkbox" value="1"><span>Use a custom station MAC address<small>Applied at boot before STA connection. Uncheck to restore the factory MAC after restart.</small></span></label><div class="field"><label for="mac">Custom station MAC</label><input class="mono" id="mac" name="mac" type="text" maxlength="17" pattern="[0-9A-Fa-f]{2}([:-][0-9A-Fa-f]{2}){5}" placeholder="02:00:00:00:00:01"><p class="help">Must be nonzero and unicast. A locally administered address (first octet 02, 06, 0A, 0E, etc.) is recommended and must be unique on your LAN.</p></div><div class="notice">Changing hostname or custom-MAC settings schedules a short restart, which is required by the Arduino Wi-Fi lifecycle. Other changes reconnect STA without intentionally stopping AP.</div><div class="actions"><button class="btn" type="submit">Save & apply station settings</button></div></form></main><footer class="footer">The AP is the recovery path if upstream configuration is incorrect.</footer><div class="toast-region" id="toastRegion" aria-live="polite"></div></body></html>)HTML";

static const char PAGE_SYSTEM[] PROGMEM = R"HTML(<!doctype html><html lang="en"><head><meta charset="utf-8"><meta name="viewport" content="width=device-width,initial-scale=1"><meta name="theme-color" content="#2563eb"><title>System · ESP32 Environment</title><link rel="stylesheet" href="/app.css"><script src="/app.js" defer></script></head><body data-page="system"><div class="restart-banner">Restart pending — reconnect if this page stops responding.</div><header class="topbar"><div class="shell topbar-inner"><a class="brand" href="/"><span class="brand-mark">EC</span><span>ESP32 Environment<small>LOCAL CONTROLLER</small></span></a><div class="connection"><span class="dot" id="connectionDot"></span><span id="connectionText">Connecting…</span></div></div><div class="shell"><nav aria-label="Main navigation"><a href="/">Overview</a><a href="/sensor">Sensor & relay</a><a href="/ap">Wi-Fi AP</a><a href="/station">Wi-Fi station</a><a class="active" href="/system">System</a></nav></div></header><main class="shell"><div class="page-head"><div><h1>System & diagnostics</h1><p class="subtitle">Storage health, memory indicators, uptime and recovery controls.</p></div><span class="badge" id="storageBadge">—</span></div><section class="grid two"><article class="card"><h2>Runtime</h2><table class="details"><tr><th>Firmware</th><td data-bind="firmware">—</td></tr><tr><th>Arduino ESP-IDF SDK</th><td class="mono" data-bind="system.sdk">—</td></tr><tr><th>Chip</th><td><span data-bind="system.chipModel">—</span> rev <span data-bind="system.chipRevision">—</span></td></tr><tr><th>Overflow-extended uptime</th><td data-bind="uptimeMs" data-format="uptime">—</td></tr><tr><th>Reset reason code</th><td data-bind="system.resetReason">—</td></tr><tr><th>Free heap</th><td data-bind="system.heap" data-format="bytes">—</td></tr><tr><th>Minimum free heap since boot</th><td data-bind="system.minHeap" data-format="bytes">—</td></tr><tr><th>Largest allocatable heap block</th><td data-bind="system.maxAllocHeap" data-format="bytes">—</td></tr></table><p class="help">Heap values are diagnostics, not an automatic reboot trigger. Fixed configuration and response buffers, PROGMEM assets and no periodic Arduino String construction reduce application-level fragmentation.</p></article><article class="card"><h2>Configuration storage</h2><table class="details"><tr><th>NVS status</th><td data-bind="system.storageOk" data-format="bool">—</td></tr><tr><th>Preferences ready</th><td data-bind="system.storageReady" data-format="bool">—</td></tr><tr><th>Deferred commit pending</th><td data-bind="system.configDirty" data-format="bool">—</td></tr><tr><th>Valid slots bitmask</th><td data-bind="system.validSlots">—</td></tr><tr><th>Active slot</th><td data-bind="system.activeSlot">—</td></tr><tr><th>Generation</th><td data-bind="system.generation">—</td></tr><tr><th>Successful writes this boot</th><td data-bind="system.saves">—</td></tr><tr><th>Restart pending</th><td data-bind="system.restartPending" data-format="bool">—</td></tr><tr><th>Restart reason</th><td data-bind="system.restartReason">—</td></tr></table><div class="notice">Every save writes only the older/invalid slot, reads it back, verifies its CRC and retains the previous valid slot. Saves occur only after actual changes and are debounced. Wi-Fi SDK persistence is disabled.</div></article></section><section class="card section"><h2>Factory reset button</h2><p>Hold the board’s <strong>BOOT button (GPIO0)</strong> continuously for <strong>five seconds</strong> while firmware is running. Debounce is 40 ms. Release after the device begins restarting. The reset latch prevents repeated NVS erases if GPIO0 remains held across restarts; it clears only after a debounced release.</p><div class="notice warn">Factory reset restores AP password <span class="mono">configure123</span>, DHCP station mode with no upstream SSID, BME280 address 0x76, zero calibration offsets, active-HIGH relay, automatic temperature-above 30 °C with 1 °C hysteresis. Relay output is driven OFF during reset/restart.</div></section><section class="card section danger-zone"><h2>Recovery controls</h2><p class="subtitle">Use these controls only when necessary. Active web and Wi-Fi connections will briefly drop.</p><div class="actions"><button class="btn secondary" type="button" data-system-action="reboot">Restart controller</button><button class="btn danger" type="button" data-system-action="factory">Erase settings & factory reset</button></div></section></main><footer class="footer">No sensor history is written to flash. No cloud connection is used.</footer><div class="toast-region" id="toastRegion" aria-live="polite"></div></body></html>)HTML";

static void setupHttpRoutes() {
  server.on("/", HTTP_GET, [] { sendProgmem(200, "text/html; charset=utf-8", PAGE_DASHBOARD); });
  server.on("/sensor", HTTP_GET, [] { sendProgmem(200, "text/html; charset=utf-8", PAGE_SENSOR); });
  server.on("/ap", HTTP_GET, [] { sendProgmem(200, "text/html; charset=utf-8", PAGE_AP); });
  server.on("/station", HTTP_GET, [] { sendProgmem(200, "text/html; charset=utf-8", PAGE_STATION); });
  server.on("/system", HTTP_GET, [] { sendProgmem(200, "text/html; charset=utf-8", PAGE_SYSTEM); });
  server.on("/app.css", HTTP_GET, [] { sendProgmem(200, "text/css; charset=utf-8", APP_CSS); });
  server.on("/app.js", HTTP_GET, [] { sendProgmem(200, "application/javascript; charset=utf-8", APP_JS); });
  server.on("/api/state", HTTP_GET, buildStateJson);
  server.on("/api/config/sensor", HTTP_POST, handleSensorConfig);
  server.on("/api/relay", HTTP_POST, handleRelayMode);
  server.on("/api/config/ap", HTTP_POST, handleApConfig);
  server.on("/api/config/station", HTTP_POST, handleStaConfig);
  server.on("/api/reboot", HTTP_POST, handleReboot);
  server.on("/api/factory", HTTP_POST, handleFactoryReset);
  const char *captivePaths[] = {"/generate_204", "/gen_204", "/hotspot-detect.html",
    "/library/test/success.html", "/connecttest.txt", "/ncsi.txt", "/fwlink",
    "/canonical.html", "/success.txt", "/check_network_status.txt"};
  for (const char *path : captivePaths) server.on(path, HTTP_GET, redirectToPortal);
  server.onNotFound(handleNotFound);
  static const char *headers[] = {"X-Requested-With", "Content-Length", "Host"};
  server.collectHeaders(headers, 3);
  server.begin();
}

static void performFactoryReset() {
  writeRelay(false, "factory reset safe off", true);
  if (preferencesReady) {
    preferences.clear();
  } else {
    preferencesReady = preferences.begin("env32", false);
    if (preferencesReady) preferences.clear();
  }
  setDefaults();
  const uint32_t factoryBaseGeneration = activeGeneration;
  const bool first = writeSlot(0, cfg, factoryBaseGeneration + 1U);
  const bool second = writeSlot(1, cfg, factoryBaseGeneration + 2U);
  activeSlot = second ? 1 : (first ? 0 : -1);
  activeGeneration = second ? factoryBaseGeneration + 2U :
                     (first ? factoryBaseGeneration + 1U : factoryBaseGeneration);
  validSlotMask = (first ? 1U : 0U) | (second ? 2U : 0U);
  storageOk = first || second;
  configDirty = false;
  factoryPending = false;
}

static void serviceFactoryButton(uint32_t now) {
  const bool raw = digitalRead(PIN_FACTORY);
  if (raw != buttonLastRaw) {
    buttonLastRaw = raw;
    buttonRawChangedMs = now;
  }
  if (raw != buttonStable && elapsed(now, buttonRawChangedMs, FACTORY_DEBOUNCE_MS)) {
    buttonStable = raw;
    if (buttonStable == LOW) buttonPressedMs = now;
    else factoryResetLatched = false;
  }
  if (buttonStable == LOW && !factoryResetLatched && !factoryPending &&
      elapsed(now, buttonPressedMs, FACTORY_HOLD_MS)) {
    factoryResetLatched = true;
    factoryPending = true;
    requestRestart("GPIO0 five-second factory reset", 250);
  }
}

static void serviceLifecycle(uint32_t now) {
  if (factoryPending) performFactoryReset();
  if (restartPending && elapsed(now, restartRequestedMs, restartDelayMs)) {
    writeRelay(false, "restart safe off", true);
    if (configDirty) saveConfigurationNow();
    ESP.restart();
  }
}

void setup() {
  Serial.begin(115200);
  clockLastMillis = millis();
  clockLastMicros = micros();
  pinMode(PIN_FACTORY, INPUT_PULLUP);
  buttonStable = buttonLastRaw = digitalRead(PIN_FACTORY);
  buttonRawChangedMs = buttonPressedMs = clockLastMillis;

  loadConfiguration();
  digitalWrite(PIN_RELAY, cfg.relay.activeHigh ? LOW : HIGH);
  pinMode(PIN_RELAY, OUTPUT);
  relayOn = false;

  Wire.begin(PIN_I2C_SDA, PIN_I2C_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  startSensorInitialization(millis());
  lastSensorSampleMs = millis();

  WiFi.persistent(false);
  WiFi.setHostname(cfg.sta.hostname);
  WiFi.mode(WIFI_AP_STA);
  WiFi.setSleep(false);
  if (cfg.sta.useCustomMac) {
    uint8_t apMac[6] = {0};
    macApplyOk = esp_wifi_get_mac(WIFI_IF_AP, apMac) == ESP_OK &&
                 memcmp(apMac, cfg.sta.mac, sizeof(apMac)) != 0 &&
                 esp_wifi_set_mac(WIFI_IF_STA, cfg.sta.mac) == ESP_OK;
  }
  startAccessPoint();
  applyStation();
  lastSaveAttemptMs = millis() - CONFIG_RETRY_MS;
  dnsActive = dnsServer.start(DNS_PORT, "*", AP_IP);
  setupHttpRoutes();
  if (MDNS.begin(cfg.sta.hostname)) MDNS.addService("http", "tcp", HTTP_PORT);

  Serial.printf("\n%s v%s\nAP: http://%s/  SSID: %s\n",
                DEVICE_NAME, FIRMWARE_VERSION, WiFi.softAPIP().toString().c_str(), cfg.ap.ssid);
}

void loop() {
  updateMonotonicClock();
  dnsServer.processNextRequest();
  server.handleClient();
  updateMonotonicClock();
  const uint32_t now = millis();
  serviceFactoryButton(now);
  serviceSensor(now);
  serviceRelay();
  serviceNetwork(now);
  serviceStorage(now);
  serviceLifecycle(now);
}
