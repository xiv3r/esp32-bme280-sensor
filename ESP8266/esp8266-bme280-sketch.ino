/*
 * =====================================================================================
 * SYSTEM: ESP8266 BME280 ENVIRONMENTAL CONTROLLER & CAPTIVE PORTAL
 * HARDWARE CONFIGURATION:
 *   - ESP8266 Microcontroller Platform (NodeMCU / Wemos D1 Mini / ESP-12)
 *   - BME280 I2C Sensor: GPIO 5 / D1 (SDA), GPIO 4 / D2 (SCL)
 *   - Relay 1 Output:    GPIO 14 / D5 (Unified Mode All-Sensor Relay / Individual Mode Temp)
 *   - Relay 2 Output:    GPIO 12 / D6 (Individual Mode Humidity Relay)
 *   - Relay 3 Output:    GPIO 13 / D7 (Individual Mode Pressure Relay)
 *   - Factory Reset Btn: GPIO 0 / D3  (Onboard FLASH / BOOT Button, Hold >= 5000 ms)
 * =====================================================================================
 */

#include <Arduino.h>
#include <Wire.h>
#include <SPI.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BME280.h>
#include <ESP8266WiFi.h>
#include <ESP8266WebServer.h>
#include <DNSServer.h>
#include <ArduinoJson.h>
#include <EEPROM.h>
#include <math.h>

// =====================================================================================
// SYSTEM HARDWARE & ALGORITHMIC CONSTANTS (64-BIT ENFORCED)
// =====================================================================================
static const uint64_t PIN_I2C_SDA             = 5ULL;
static const uint64_t PIN_I2C_SCL             = 4ULL;
static const uint64_t PIN_RELAY_TEMP_UNIF     = 14ULL;
static const uint64_t PIN_RELAY_HUMIDITY      = 12ULL;
static const uint64_t PIN_RELAY_PRESSURE      = 13ULL;
static const uint64_t PIN_FACTORY_RESET_BTN   = 0ULL;

static const uint64_t CFG_MAGIC               = 0x424D453238305634ULL;
static const uint64_t CFG_VERSION             = 4ULL;
static const uint64_t BME_PRIMARY_ADDR        = 0x76ULL;
static const uint64_t BME_SECONDARY_ADDR      = 0x77ULL;

static const uint64_t FACTORY_RESET_HOLD_MS   = 5000ULL;
static const uint64_t BTN_DEBOUNCE_MS         = 40ULL;
static const uint64_t BME_RECOVERY_RETRY_MS   = 5000ULL;
static const uint64_t BME_WARMUP_DELAY_MS     = 35ULL;
static const uint64_t NVS_COMMIT_DELAY_MS     = 600ULL;
static const uint64_t NVS_MIN_WRITE_GAP_MS    = 2000ULL;
static const uint64_t ACTION_DELAY_MS         = 800ULL;

static const uint64_t DNS_PORT                = 53ULL;
static const uint64_t HTTP_PORT               = 80ULL;
static const uint64_t EEPROM_BANK_A_OFFSET    = 0ULL;
static const uint64_t EEPROM_BANK_B_OFFSET    = 512ULL;
static const uint64_t EEPROM_TOTAL_SIZE       = 1024ULL;

static const uint64_t RELAY_MODE_UNIFIED      = 0ULL;
static const uint64_t RELAY_MODE_INDIVIDUAL   = 1ULL;

static const uint64_t POLARITY_ACTIVE_HIGH    = 0ULL;
static const uint64_t POLARITY_ACTIVE_LOW     = 1ULL;

static const uint64_t FAILSAFE_FORCE_OFF      = 0ULL;
static const uint64_t FAILSAFE_HOLD_LAST      = 1ULL;
static const uint64_t FAILSAFE_FORCE_ON       = 2ULL;

static const uint64_t COND_ABOVE_HIGH         = 0ULL;
static const uint64_t COND_BELOW_LOW          = 1ULL;
static const uint64_t COND_OUTSIDE_RANGE      = 2ULL;
static const uint64_t COND_INSIDE_RANGE       = 3ULL;

static const double ATMOSPHERIC_SEA_LEVEL_HPA = 1013.25;

// =====================================================================================
// DATA STRUCTURE DEFINITIONS (64-BIT / 8-BYTE ALIGNED)
// =====================================================================================
struct alignas(8) SensorChannelConfig {
    uint64_t enabled;
    uint64_t condition;
    double lowThreshold;
    double highThreshold;
    double hysteresis;
    double offset;
};

struct alignas(8) SystemConfig {
    uint64_t magic;
    uint64_t version;
    uint64_t sequence;
    uint64_t crc64;

    uint64_t relayMode;
    uint64_t relayPolarity;
    uint64_t failsafeMode;
    uint64_t minRelaySwitchMs;
    uint64_t sensorPollMs;

    SensorChannelConfig channels[3];

    char apSSID[33];
    char apPassword[65];
    uint64_t apChannel;
    uint64_t apMaxClients;
    uint64_t apHideSSID;
};

struct alignas(8) RelayState {
    uint64_t isEnergized;
    uint64_t lastSwitchMs;
    uint64_t totalSwitchCount;
    uint64_t cooldownActive;
};

struct alignas(8) ChannelRuntimeState {
    uint64_t isLatchedActive;
    double rawValue;
    double calibratedValue;
};

struct alignas(8) RuntimeState {
    uint64_t bootTimeMs;
    uint64_t systemUptimeMs;

    uint64_t sensorHealthy;
    uint64_t i2cInitialized;
    uint64_t activeI2CAddr;
    uint64_t hasValidSample;
    uint64_t lastSensorPollMs;
    uint64_t lastI2CRecoveryMs;
    uint64_t sensorWarmupUntilMs;

    double rawTempC;
    double rawHumPct;
    double rawPresHpa;
    double calTempC;
    double calTempF;
    double calHumPct;
    double calPresHpa;
    double dewPointC;
    double dewPointF;
    double altitudeMeters;

    ChannelRuntimeState channelState[3];
    RelayState relays[3];

    uint64_t nvsDirty;
    uint64_t nvsDirtyTimeMs;
    uint64_t lastNVSWriteMs;
    uint64_t nvsWriteCount;

    uint64_t btnRawState;
    uint64_t btnStableState;
    uint64_t btnLastChangeMs;
    uint64_t buttonPressStartMs;
    uint64_t buttonIsHeld;
    uint64_t factoryResetTriggered;

    uint64_t rebootPending;
    uint64_t rebootTargetMs;
    uint64_t wifiReloadPending;
    uint64_t wifiReloadTargetMs;
};

// =====================================================================================
// GLOBAL OBJECTS & STATIC MEMORY BUFFERS
// =====================================================================================
static SystemConfig       g_config;
static RuntimeState       g_state;
static Adafruit_BME280    g_bme;
static ESP8266WebServer   g_webServer((int)HTTP_PORT);
static DNSServer          g_dnsServer;

static char               g_jsonBuf[3584];

extern const char INDEX_HTML[] PROGMEM;

// =====================================================================================
// 64-BIT TIMEBASE & MATHEMATICAL UTILITIES
// =====================================================================================
static inline uint64_t getMillis64(void) {
    static uint32_t s_lastLow = 0U;
    static uint64_t s_highOrder = 0ULL;
    uint32_t nowLow = millis();
    if (nowLow < s_lastLow) {
        s_highOrder += 0x100000000ULL;
    }
    s_lastLow = nowLow;
    return s_highOrder | (uint64_t)nowLow;
}

static inline uint64_t getMicros64(void) {
    static uint32_t s_lastMicrosLow = 0U;
    static uint64_t s_microsHigh = 0ULL;
    uint32_t nowLow = micros();
    if (nowLow < s_lastMicrosLow) {
        s_microsHigh += 0x100000000ULL;
    }
    s_lastMicrosLow = nowLow;
    return s_microsHigh | (uint64_t)nowLow;
}

static inline double clampDouble(double val, double minVal, double maxVal) {
    if (isnan(val) || isinf(val)) return minVal;
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

static inline uint64_t clampUint64(uint64_t val, uint64_t minVal, uint64_t maxVal) {
    if (val < minVal) return minVal;
    if (val > maxVal) return maxVal;
    return val;
}

static inline double safeJsonDouble(double val) {
    if (isnan(val) || isinf(val)) return 0.0;
    return val;
}

static void escapeJsonString(char* dest, uint64_t destSize, const char* src) {
    if (destSize == 0ULL) return;
    uint64_t dIdx = 0ULL;
    for (uint64_t sIdx = 0ULL; src[sIdx] != '\0'; ++sIdx) {
        char c = src[sIdx];
        if (c == '\"' || c == '\\') {
            if (dIdx + 2ULL >= destSize) break;
            dest[dIdx++] = '\\';
            dest[dIdx++] = c;
        } else if ((uint8_t)c >= 0x20U && (uint8_t)c <= 0x7EU) {
            if (dIdx + 1ULL >= destSize) break;
            dest[dIdx++] = c;
        }
    }
    dest[dIdx] = '\0';
}

// =====================================================================================
// ATMOSPHERIC PHYSICS ENGINE
// =====================================================================================
static double calculateDewPoint(double tempC, double humPct) {
    double validHum  = clampDouble(humPct, 0.01, 100.0);
    double validTemp = clampDouble(tempC, -80.0, 100.0);

    static const double a = 17.27;
    static const double b = 237.7;

    double denomTemp = b + validTemp;
    if (fabs(denomTemp) < 1e-6) return 0.0;

    double alpha = ((a * validTemp) / denomTemp) + log(validHum / 100.0);
    double denominator = a - alpha;
    if (fabs(denominator) < 1e-6) return 0.0;

    double dewPoint = (b * alpha) / denominator;
    return clampDouble(dewPoint, -80.0, 100.0);
}

static double calculateAltitude(double presHpa) {
    if (isnan(presHpa) || isinf(presHpa) || presHpa <= 0.0) return 0.0;
    double alt = 44330.0 * (1.0 - pow(presHpa / ATMOSPHERIC_SEA_LEVEL_HPA, 1.0 / 5.255));
    return clampDouble(alt, -1000.0, 20000.0);
}

// =====================================================================================
// NVS CHECKSUM & CONFIG INTEGRITY VALIDATION
// =====================================================================================
static uint64_t computeConfigChecksum(const SystemConfig* cfg) {
    const uint64_t FNV_OFFSET_BASIS = 0xCBF29CE484222325ULL;
    const uint64_t FNV_PRIME        = 0x100000001B3ULL;

    uint64_t hash = FNV_OFFSET_BASIS;
    const uint8_t* ptr = (const uint8_t*)cfg;
    uint64_t payloadOffset = (uint64_t)offsetof(SystemConfig, relayMode);
    uint64_t payloadSize   = (uint64_t)sizeof(SystemConfig) - payloadOffset;

    for (uint64_t i = 0ULL; i < payloadSize; ++i) {
        hash ^= (uint64_t)ptr[payloadOffset + i];
        hash *= FNV_PRIME;
    }
    return hash;
}

static uint64_t validateConfig(SystemConfig* cfg) {
    if (cfg == NULL) return 0ULL;
    if (cfg->magic != CFG_MAGIC) return 0ULL;
    if (cfg->version != CFG_VERSION) return 0ULL;
    if (cfg->crc64 != computeConfigChecksum(cfg)) return 0ULL;

    if (cfg->relayMode > RELAY_MODE_INDIVIDUAL) return 0ULL;
    if (cfg->relayPolarity > POLARITY_ACTIVE_LOW) return 0ULL;
    if (cfg->failsafeMode > FAILSAFE_FORCE_ON) return 0ULL;
    if (cfg->minRelaySwitchMs < 100ULL || cfg->minRelaySwitchMs > 3600000ULL) return 0ULL;
    if (cfg->sensorPollMs < 250ULL || cfg->sensorPollMs > 300000ULL) return 0ULL;

    for (uint64_t i = 0ULL; i < 3ULL; ++i) {
        SensorChannelConfig* ch = &cfg->channels[i];
        if (ch->enabled > 1ULL) return 0ULL;
        if (ch->condition > COND_INSIDE_RANGE) return 0ULL;
        if (!isfinite(ch->lowThreshold) || !isfinite(ch->highThreshold) ||
            !isfinite(ch->hysteresis)   || !isfinite(ch->offset)) {
            return 0ULL;
        }
        if (ch->lowThreshold > ch->highThreshold) return 0ULL;

        if (i == 0ULL) {
            if (ch->lowThreshold < -40.0 || ch->highThreshold > 85.0) return 0ULL;
            if (ch->hysteresis < 0.05 || ch->hysteresis > 25.0) return 0ULL;
            if (ch->offset < -25.0 || ch->offset > 25.0) return 0ULL;
        } else if (i == 1ULL) {
            if (ch->lowThreshold < 0.0 || ch->highThreshold > 100.0) return 0ULL;
            if (ch->hysteresis < 0.05 || ch->hysteresis > 40.0) return 0ULL;
            if (ch->offset < -40.0 || ch->offset > 40.0) return 0ULL;
        } else {
            if (ch->lowThreshold < 300.0 || ch->highThreshold > 1100.0) return 0ULL;
            if (ch->hysteresis < 0.1 || ch->hysteresis > 100.0) return 0ULL;
            if (ch->offset < -100.0 || ch->offset > 100.0) return 0ULL;
        }
    }

    cfg->apSSID[32] = '\0';
    cfg->apPassword[64] = '\0';

    uint64_t ssidLen = (uint64_t)strlen(cfg->apSSID);
    uint64_t passLen = (uint64_t)strlen(cfg->apPassword);
    if (ssidLen < 1ULL || ssidLen > 32ULL) return 0ULL;
    if (passLen < 8ULL || passLen > 63ULL) return 0ULL;
    if (cfg->apChannel < 1ULL || cfg->apChannel > 13ULL) return 0ULL;
    if (cfg->apMaxClients < 1ULL || cfg->apMaxClients > 8ULL) return 0ULL;
    if (cfg->apHideSSID > 1ULL) return 0ULL;

    return 1ULL;
}

static void sanitizeAndClampConfig(SystemConfig* cfg) {
    cfg->magic            = CFG_MAGIC;
    cfg->version          = CFG_VERSION;
    cfg->relayMode        = (cfg->relayMode > RELAY_MODE_INDIVIDUAL) ? RELAY_MODE_UNIFIED : cfg->relayMode;
    cfg->relayPolarity    = (cfg->relayPolarity > POLARITY_ACTIVE_LOW) ? POLARITY_ACTIVE_HIGH : cfg->relayPolarity;
    cfg->failsafeMode     = (cfg->failsafeMode > FAILSAFE_FORCE_ON) ? FAILSAFE_FORCE_OFF : cfg->failsafeMode;
    cfg->minRelaySwitchMs = clampUint64(cfg->minRelaySwitchMs, 100ULL, 3600000ULL);
    cfg->sensorPollMs     = clampUint64(cfg->sensorPollMs, 250ULL, 300000ULL);

    for (uint64_t i = 0ULL; i < 3ULL; ++i) {
        SensorChannelConfig* ch = &cfg->channels[i];
        ch->enabled   = (ch->enabled != 0ULL) ? 1ULL : 0ULL;
        ch->condition = (ch->condition > COND_INSIDE_RANGE) ? COND_ABOVE_HIGH : ch->condition;

        if (i == 0ULL) {
            ch->lowThreshold  = clampDouble(ch->lowThreshold, -40.0, 85.0);
            ch->highThreshold = clampDouble(ch->highThreshold, -40.0, 85.0);
            ch->hysteresis    = clampDouble(ch->hysteresis, 0.05, 25.0);
            ch->offset        = clampDouble(ch->offset, -25.0, 25.0);
        } else if (i == 1ULL) {
            ch->lowThreshold  = clampDouble(ch->lowThreshold, 0.0, 100.0);
            ch->highThreshold = clampDouble(ch->highThreshold, 0.0, 100.0);
            ch->hysteresis    = clampDouble(ch->hysteresis, 0.05, 40.0);
            ch->offset        = clampDouble(ch->offset, -40.0, 40.0);
        } else {
            ch->lowThreshold  = clampDouble(ch->lowThreshold, 300.0, 1100.0);
            ch->highThreshold = clampDouble(ch->highThreshold, 300.0, 1100.0);
            ch->hysteresis    = clampDouble(ch->hysteresis, 0.1, 100.0);
            ch->offset        = clampDouble(ch->offset, -100.0, 100.0);
        }

        if (ch->lowThreshold > ch->highThreshold) {
            double tmp = ch->lowThreshold;
            ch->lowThreshold  = ch->highThreshold;
            ch->highThreshold = tmp;
        }
    }

    cfg->apSSID[32] = '\0';
    if (strlen(cfg->apSSID) == 0) {
        strncpy(cfg->apSSID, "ESP8266_BME280", sizeof(cfg->apSSID) - 1);
    }

    cfg->apPassword[63] = '\0';
    if (strlen(cfg->apPassword) < 8) {
        strncpy(cfg->apPassword, "12345678", sizeof(cfg->apPassword) - 1);
    }

    cfg->apChannel    = clampUint64(cfg->apChannel, 1ULL, 13ULL);
    cfg->apMaxClients = clampUint64(cfg->apMaxClients, 1ULL, 8ULL);
    cfg->apHideSSID   = (cfg->apHideSSID != 0ULL) ? 1ULL : 0ULL;
    cfg->crc64        = computeConfigChecksum(cfg);
}

static void applyDefaultSettings(SystemConfig* cfg) {
    memset(cfg, 0, sizeof(SystemConfig));
    cfg->magic            = CFG_MAGIC;
    cfg->version          = CFG_VERSION;
    cfg->sequence         = 1ULL;
    cfg->relayMode        = RELAY_MODE_UNIFIED;
    cfg->relayPolarity    = POLARITY_ACTIVE_HIGH;
    cfg->failsafeMode     = FAILSAFE_FORCE_OFF;
    cfg->minRelaySwitchMs = 3000ULL;
    cfg->sensorPollMs     = 1000ULL;

    cfg->channels[0].enabled       = 1ULL;
    cfg->channels[0].condition     = COND_ABOVE_HIGH;
    cfg->channels[0].lowThreshold  = 18.0;
    cfg->channels[0].highThreshold = 30.0;
    cfg->channels[0].hysteresis    = 1.0;
    cfg->channels[0].offset        = 0.0;

    cfg->channels[1].enabled       = 1ULL;
    cfg->channels[1].condition     = COND_ABOVE_HIGH;
    cfg->channels[1].lowThreshold  = 30.0;
    cfg->channels[1].highThreshold = 70.0;
    cfg->channels[1].hysteresis    = 2.0;
    cfg->channels[1].offset        = 0.0;

    cfg->channels[2].enabled       = 1ULL;
    cfg->channels[2].condition     = COND_BELOW_LOW;
    cfg->channels[2].lowThreshold  = 995.0;
    cfg->channels[2].highThreshold = 1030.0;
    cfg->channels[2].hysteresis    = 2.0;
    cfg->channels[2].offset        = 0.0;

    strncpy(cfg->apSSID, "ESP8266_BME280", sizeof(cfg->apSSID) - 1);
    strncpy(cfg->apPassword, "12345678", sizeof(cfg->apPassword) - 1);
    cfg->apChannel    = 1ULL;
    cfg->apMaxClients = 4ULL;
    cfg->apHideSSID   = 0ULL;

    cfg->crc64        = computeConfigChecksum(cfg);
}

// =====================================================================================
// HARDWARE RELAY CONTROL ENGINE
// =====================================================================================
static void writeRelayHardwarePins(uint64_t index, uint64_t isEnergized) {
    uint64_t pin = PIN_RELAY_TEMP_UNIF;
    if (index == 1ULL) pin = PIN_RELAY_HUMIDITY;
    if (index == 2ULL) pin = PIN_RELAY_PRESSURE;

    uint64_t activeHigh = (g_config.relayPolarity == POLARITY_ACTIVE_HIGH) ? 1ULL : 0ULL;
    uint64_t pinLevel   = (uint64_t)LOW;

    if (activeHigh) {
        pinLevel = isEnergized ? (uint64_t)HIGH : (uint64_t)LOW;
    } else {
        pinLevel = isEnergized ? (uint64_t)LOW : (uint64_t)HIGH;
    }
    digitalWrite((uint8_t)pin, (uint8_t)pinLevel);
}

static uint64_t evaluateChannelTrigger(uint64_t chIndex) {
    if (chIndex >= 3ULL) return 0ULL;
    const SensorChannelConfig* ch = &g_config.channels[chIndex];
    ChannelRuntimeState* st       = &g_state.channelState[chIndex];

    if (!ch->enabled) {
        st->isLatchedActive = 0ULL;
        return 0ULL;
    }

    double val  = st->calibratedValue;
    double low  = ch->lowThreshold;
    double high = ch->highThreshold;
    double hyst = ch->hysteresis;

    uint64_t newActiveState = st->isLatchedActive;

    switch (ch->condition) {
        case COND_ABOVE_HIGH:
            if (val >= high) {
                newActiveState = 1ULL;
            } else if (val <= (high - hyst)) {
                newActiveState = 0ULL;
            }
            break;

        case COND_BELOW_LOW:
            if (val <= low) {
                newActiveState = 1ULL;
            } else if (val >= (low + hyst)) {
                newActiveState = 0ULL;
            }
            break;

        case COND_OUTSIDE_RANGE: {
            double span = high - low;
            double effHyst = clampDouble(hyst, 0.01, (span > 0.02 ? span * 0.45 : 0.01));
            if (val >= high || val <= low) {
                newActiveState = 1ULL;
            } else if (val <= (high - effHyst) && val >= (low + effHyst)) {
                newActiveState = 0ULL;
            }
            break;
        }

        case COND_INSIDE_RANGE: {
            double span = high - low;
            double effHyst = clampDouble(hyst, 0.01, (span > 0.02 ? span * 0.45 : 0.01));
            if (val >= low && val <= high) {
                newActiveState = 1ULL;
            } else if (val <= (low - effHyst) || val >= (high + effHyst)) {
                newActiveState = 0ULL;
            }
            break;
        }

        default:
            newActiveState = 0ULL;
            break;
    }

    st->isLatchedActive = newActiveState;
    return newActiveState;
}

static void updateRelaysNonBlocking(uint64_t forceImmediate) {
    uint64_t nowMs = getMillis64();
    uint64_t targetStates[3] = {0ULL, 0ULL, 0ULL};

    if (!g_state.sensorHealthy || !g_state.hasValidSample) {
        for (uint64_t i = 0ULL; i < 3ULL; ++i) {
            g_state.channelState[i].isLatchedActive = 0ULL;
        }
        switch (g_config.failsafeMode) {
            case FAILSAFE_FORCE_OFF:
                targetStates[0] = 0ULL;
                targetStates[1] = 0ULL;
                targetStates[2] = 0ULL;
                break;
            case FAILSAFE_HOLD_LAST:
                targetStates[0] = g_state.relays[0].isEnergized;
                targetStates[1] = (g_config.relayMode == RELAY_MODE_INDIVIDUAL) ? g_state.relays[1].isEnergized : 0ULL;
                targetStates[2] = (g_config.relayMode == RELAY_MODE_INDIVIDUAL) ? g_state.relays[2].isEnergized : 0ULL;
                break;
            case FAILSAFE_FORCE_ON:
                targetStates[0] = 1ULL;
                targetStates[1] = (g_config.relayMode == RELAY_MODE_INDIVIDUAL) ? 1ULL : 0ULL;
                targetStates[2] = (g_config.relayMode == RELAY_MODE_INDIVIDUAL) ? 1ULL : 0ULL;
                break;
            default:
                targetStates[0] = 0ULL;
                targetStates[1] = 0ULL;
                targetStates[2] = 0ULL;
                break;
        }
    } else {
        uint64_t ch0Trigger = evaluateChannelTrigger(0ULL);
        uint64_t ch1Trigger = evaluateChannelTrigger(1ULL);
        uint64_t ch2Trigger = evaluateChannelTrigger(2ULL);

        if (g_config.relayMode == RELAY_MODE_UNIFIED) {
            targetStates[0] = (ch0Trigger || ch1Trigger || ch2Trigger) ? 1ULL : 0ULL;
            targetStates[1] = 0ULL;
            targetStates[2] = 0ULL;
        } else {
            targetStates[0] = ch0Trigger;
            targetStates[1] = ch1Trigger;
            targetStates[2] = ch2Trigger;
        }
    }

    for (uint64_t i = 0ULL; i < 3ULL; ++i) {
        RelayState* r    = &g_state.relays[i];
        uint64_t desired = targetStates[i];
        uint64_t standbyForceOff = (g_config.relayMode == RELAY_MODE_UNIFIED && i > 0ULL) ? 1ULL : 0ULL;

        if (desired != r->isEnergized) {
            uint64_t elapsedMs = (nowMs >= r->lastSwitchMs) ? (nowMs - r->lastSwitchMs) : 0ULL;
            if (forceImmediate || standbyForceOff || r->lastSwitchMs == 0ULL || (elapsedMs >= g_config.minRelaySwitchMs)) {
                r->isEnergized    = desired;
                r->lastSwitchMs   = nowMs;
                r->cooldownActive = 0ULL;
                r->totalSwitchCount++;
                writeRelayHardwarePins(i, r->isEnergized);
            } else {
                r->cooldownActive = 1ULL;
            }
        } else {
            r->cooldownActive = 0ULL;
            writeRelayHardwarePins(i, r->isEnergized);
        }
    }
}

// =====================================================================================
// BME280 HARDWARE RECOVERY & TELEMETRY POLLING ENGINE
// =====================================================================================
static void initOrRecoverI2CBus(uint64_t forceReset) {
    if (g_state.i2cInitialized && !forceReset) return;

    if (g_state.i2cInitialized) {
        g_state.i2cInitialized = 0ULL;
    }

    pinMode((uint8_t)PIN_I2C_SDA, INPUT_PULLUP);
    pinMode((uint8_t)PIN_I2C_SCL, OUTPUT);
    digitalWrite((uint8_t)PIN_I2C_SCL, HIGH);
    delayMicroseconds(10);

    for (uint64_t i = 0ULL; i < 9ULL; ++i) {
        if (digitalRead((uint8_t)PIN_I2C_SDA) == HIGH) break;
        digitalWrite((uint8_t)PIN_I2C_SCL, LOW);
        delayMicroseconds(10);
        digitalWrite((uint8_t)PIN_I2C_SCL, HIGH);
        delayMicroseconds(10);
    }

    pinMode((uint8_t)PIN_I2C_SDA, OUTPUT);
    digitalWrite((uint8_t)PIN_I2C_SDA, LOW);
    delayMicroseconds(10);
    digitalWrite((uint8_t)PIN_I2C_SCL, HIGH);
    delayMicroseconds(10);
    digitalWrite((uint8_t)PIN_I2C_SDA, HIGH);
    delayMicroseconds(10);

    Wire.begin((int)PIN_I2C_SDA, (int)PIN_I2C_SCL);
    Wire.setClock(100000U);
    Wire.setClockStretchLimit(50000);
    g_state.i2cInitialized = 1ULL;
}

static uint64_t tryInitializeBME280(void) {
    initOrRecoverI2CBus(0ULL);

    uint64_t foundAddr = 0ULL;
    if (g_bme.begin((uint8_t)BME_PRIMARY_ADDR, &Wire)) {
        foundAddr = BME_PRIMARY_ADDR;
    } else if (g_bme.begin((uint8_t)BME_SECONDARY_ADDR, &Wire)) {
        foundAddr = BME_SECONDARY_ADDR;
    }

    if (foundAddr != 0ULL) {
        g_bme.setSampling(
            Adafruit_BME280::MODE_NORMAL,
            Adafruit_BME280::SAMPLING_X2,
            Adafruit_BME280::SAMPLING_X16,
            Adafruit_BME280::SAMPLING_X1,
            Adafruit_BME280::FILTER_X16,
            Adafruit_BME280::STANDBY_MS_0_5
        );

        g_state.sensorHealthy       = 1ULL;
        g_state.activeI2CAddr       = foundAddr;
        g_state.sensorWarmupUntilMs = getMillis64() + BME_WARMUP_DELAY_MS;
        return 1ULL;
    }

    g_state.sensorHealthy  = 0ULL;
    g_state.activeI2CAddr  = 0ULL;
    g_state.hasValidSample = 0ULL;
    return 0ULL;
}

static void recomputeCalibratedMetricsFromRaw(void) {
    if (!g_state.hasValidSample) return;

    g_state.calTempC   = clampDouble(g_state.rawTempC + g_config.channels[0].offset, -40.0, 85.0);
    g_state.calTempF   = (g_state.calTempC * 1.8) + 32.0;

    g_state.calHumPct  = clampDouble(g_state.rawHumPct + g_config.channels[1].offset, 0.0, 100.0);
    g_state.calPresHpa = clampDouble(g_state.rawPresHpa + g_config.channels[2].offset, 300.0, 1100.0);

    g_state.dewPointC  = calculateDewPoint(g_state.calTempC, g_state.calHumPct);
    g_state.dewPointF  = (g_state.dewPointC * 1.8) + 32.0;

    g_state.altitudeMeters = calculateAltitude(g_state.calPresHpa);

    g_state.channelState[0].rawValue        = g_state.rawTempC;
    g_state.channelState[0].calibratedValue = g_state.calTempC;

    g_state.channelState[1].rawValue        = g_state.rawHumPct;
    g_state.channelState[1].calibratedValue = g_state.calHumPct;

    g_state.channelState[2].rawValue        = g_state.rawPresHpa;
    g_state.channelState[2].calibratedValue = g_state.calPresHpa;
}

static void pollSensorNonBlocking(void) {
    uint64_t nowMs = getMillis64();

    if (!g_state.sensorHealthy) {
        if ((nowMs - g_state.lastI2CRecoveryMs) >= BME_RECOVERY_RETRY_MS) {
            g_state.lastI2CRecoveryMs = nowMs;
            initOrRecoverI2CBus(1ULL);
            tryInitializeBME280();
            updateRelaysNonBlocking(0ULL);
        }
        return;
    }

    if (nowMs < g_state.sensorWarmupUntilMs) return;

    if (g_state.hasValidSample && ((nowMs - g_state.lastSensorPollMs) < g_config.sensorPollMs)) {
        updateRelaysNonBlocking(0ULL);
        return;
    }
    g_state.lastSensorPollMs = nowMs;

    double rawT = (double)g_bme.readTemperature();
    double rawP = (double)(g_bme.readPressure() / 100.0F);
    double rawH = (double)g_bme.readHumidity();

    uint64_t validRead = 1ULL;
    if (!isfinite(rawT) || !isfinite(rawP) || !isfinite(rawH)) validRead = 0ULL;
    if (rawT < -45.0 || rawT > 90.0) validRead = 0ULL;
    if (rawP < 250.0 || rawP > 1150.0) validRead = 0ULL;
    if (rawH < 0.0 || rawH > 100.0) validRead = 0ULL;

    if (!validRead) {
        g_state.sensorHealthy     = 0ULL;
        g_state.hasValidSample    = 0ULL;
        g_state.lastI2CRecoveryMs = nowMs;
        updateRelaysNonBlocking(1ULL);
        return;
    }

    g_state.rawTempC       = rawT;
    g_state.rawPresHpa     = rawP;
    g_state.rawHumPct      = rawH;
    g_state.hasValidSample = 1ULL;

    recomputeCalibratedMetricsFromRaw();
    updateRelaysNonBlocking(0ULL);
}

// =====================================================================================
// DUAL-BANK CHECKSUMMED NVS FLASH PERSISTENCE ENGINE
// =====================================================================================
static void writeConfigToNvsNow(void) {
    g_config.sequence++;
    g_config.crc64 = computeConfigChecksum(&g_config);

    EEPROM.put((int)EEPROM_BANK_A_OFFSET, g_config);
    EEPROM.commit();

    SystemConfig verifyA;
    memset(&verifyA, 0, sizeof(SystemConfig));
    EEPROM.get((int)EEPROM_BANK_A_OFFSET, verifyA);
    if (validateConfig(&verifyA)) {
        EEPROM.put((int)EEPROM_BANK_B_OFFSET, g_config);
        EEPROM.commit();
    }

    g_state.nvsDirty       = 0ULL;
    g_state.lastNVSWriteMs = getMillis64();
    g_state.nvsWriteCount++;
}

static uint64_t configsEqualPayload(const SystemConfig* a, const SystemConfig* b) {
    uint64_t payloadOffset = (uint64_t)offsetof(SystemConfig, relayMode);
    uint64_t payloadSize   = (uint64_t)sizeof(SystemConfig) - payloadOffset;
    return (memcmp(((const uint8_t*)a) + payloadOffset, ((const uint8_t*)b) + payloadOffset, (size_t)payloadSize) == 0) ? 1ULL : 0ULL;
}

static void stageConfigSave(const SystemConfig* newCfg) {
    uint64_t payloadUnchanged = configsEqualPayload(&g_config, newCfg);

    memcpy(&g_config, newCfg, sizeof(SystemConfig));
    recomputeCalibratedMetricsFromRaw();
    updateRelaysNonBlocking(1ULL);

    if (payloadUnchanged) return;

    g_state.nvsDirty       = 1ULL;
    g_state.nvsDirtyTimeMs = getMillis64();
}

static void processNvsWriteCoalescing(void) {
    if (!g_state.nvsDirty) return;

    uint64_t nowMs              = getMillis64();
    uint64_t timeSinceDirty     = (nowMs >= g_state.nvsDirtyTimeMs) ? (nowMs - g_state.nvsDirtyTimeMs) : 0ULL;
    uint64_t timeSinceLastWrite = (nowMs >= g_state.lastNVSWriteMs) ? (nowMs - g_state.lastNVSWriteMs) : 0ULL;

    if (timeSinceDirty >= NVS_COMMIT_DELAY_MS && timeSinceLastWrite >= NVS_MIN_WRITE_GAP_MS) {
        writeConfigToNvsNow();
    }
}

static void initStorageAndConfig(void) {
    EEPROM.begin((int)EEPROM_TOTAL_SIZE);
    applyDefaultSettings(&g_config);

    SystemConfig bankA;
    SystemConfig bankB;
    memset(&bankA, 0, sizeof(SystemConfig));
    memset(&bankB, 0, sizeof(SystemConfig));

    EEPROM.get((int)EEPROM_BANK_A_OFFSET, bankA);
    EEPROM.get((int)EEPROM_BANK_B_OFFSET, bankB);

    uint64_t validA = validateConfig(&bankA);
    uint64_t validB = validateConfig(&bankB);

    if (validA && validB) {
        if (bankA.sequence >= bankB.sequence) {
            memcpy(&g_config, &bankA, sizeof(SystemConfig));
            if (bankA.sequence != bankB.sequence) {
                EEPROM.put((int)EEPROM_BANK_B_OFFSET, g_config);
                EEPROM.commit();
            }
        } else {
            memcpy(&g_config, &bankB, sizeof(SystemConfig));
            EEPROM.put((int)EEPROM_BANK_A_OFFSET, g_config);
            EEPROM.commit();
        }
    } else if (validA) {
        memcpy(&g_config, &bankA, sizeof(SystemConfig));
        EEPROM.put((int)EEPROM_BANK_B_OFFSET, g_config);
        EEPROM.commit();
    } else if (validB) {
        memcpy(&g_config, &bankB, sizeof(SystemConfig));
        EEPROM.put((int)EEPROM_BANK_A_OFFSET, g_config);
        EEPROM.commit();
    } else {
        applyDefaultSettings(&g_config);
        writeConfigToNvsNow();
    }
}

// =====================================================================================
// HARDWARE BOOT BUTTON DEBOUNCING & 5-SECOND FACTORY RESET ENGINE
// =====================================================================================
static void processFactoryResetButtonNonBlocking(void) {
    uint64_t nowMs = getMillis64();
    uint64_t currentRaw = (digitalRead((uint8_t)PIN_FACTORY_RESET_BTN) == LOW) ? 1ULL : 0ULL;

    if (currentRaw != g_state.btnRawState) {
        g_state.btnRawState     = currentRaw;
        g_state.btnLastChangeMs = nowMs;
    }

    if ((nowMs - g_state.btnLastChangeMs) >= BTN_DEBOUNCE_MS) {
        if (g_state.btnStableState != g_state.btnRawState) {
            g_state.btnStableState = g_state.btnRawState;
            if (g_state.btnStableState == 1ULL) {
                g_state.buttonIsHeld          = 1ULL;
                g_state.buttonPressStartMs    = nowMs;
                g_state.factoryResetTriggered = 0ULL;
            } else {
                g_state.buttonIsHeld          = 0ULL;
                g_state.factoryResetTriggered = 0ULL;
            }
        }
    }

    if (g_state.buttonIsHeld && !g_state.factoryResetTriggered) {
        uint64_t holdDuration = (nowMs >= g_state.buttonPressStartMs) ? (nowMs - g_state.buttonPressStartMs) : 0ULL;
        if (holdDuration >= FACTORY_RESET_HOLD_MS) {
            g_state.factoryResetTriggered = 1ULL;

            for (uint64_t i = 0ULL; i < 3ULL; ++i) {
                g_state.relays[i].isEnergized = 0ULL;
                writeRelayHardwarePins(i, 0ULL);
            }

            applyDefaultSettings(&g_config);
            writeConfigToNvsNow();

            for (uint64_t i = 0ULL; i < 3ULL; ++i) {
                writeRelayHardwarePins(i, 0ULL);
            }

            g_state.rebootPending  = 1ULL;
            g_state.rebootTargetMs = nowMs + ACTION_DELAY_MS;
        }
    }
}

// =====================================================================================
// WI-FI ACCESS POINT & DNS CAPTIVE PORTAL ENGINE
// =====================================================================================
static void initializeAccessPoint(void) {
    WiFi.persistent(false);
    WiFi.mode(WIFI_AP);
    WiFi.setSleepMode(WIFI_NONE_SLEEP);

    IPAddress apIP(192, 168, 4, 1);
    IPAddress netMask(255, 255, 255, 0);
    WiFi.softAPConfig(apIP, apIP, netMask);

    WiFi.softAP(
        g_config.apSSID,
        g_config.apPassword,
        (int)g_config.apChannel,
        (int)g_config.apHideSSID,
        (int)g_config.apMaxClients
    );

    g_dnsServer.setErrorReplyCode(DNSReplyCode::NoError);
    g_dnsServer.start((uint16_t)DNS_PORT, "*", apIP);
}

static void reloadWiFiSettingsNonBlocking(void) {
    if (!g_state.wifiReloadPending) return;

    uint64_t nowMs = getMillis64();
    if (nowMs >= g_state.wifiReloadTargetMs) {
        g_state.wifiReloadPending = 0ULL;
        g_dnsServer.stop();
        WiFi.softAPdisconnect(true);
        initializeAccessPoint();
    }
}

static uint64_t isCaptiveRedirectRequired(void) {
    String host = g_webServer.hostHeader();
    if (host.length() == 0 && g_webServer.hasHeader("Host")) {
        host = g_webServer.header("Host");
    }
    if (host.length() == 0) return 0ULL;
    if (host.equalsIgnoreCase("192.168.4.1") || host.equalsIgnoreCase("192.168.4.1:80")) {
        return 0ULL;
    }
    return 1ULL;
}

static void handleCaptiveRedirect(void) {
    g_webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    g_webServer.sendHeader("Location", "http://192.168.4.1/", true);
    g_webServer.send(302, "text/plain", "");
}

// =====================================================================================
// REST API HTTP ENDPOINTS
// =====================================================================================
static void handleApiStatus(void) {
    uint64_t nowMs = getMillis64();
    g_state.systemUptimeMs = (nowMs >= g_state.bootTimeMs) ? (nowMs - g_state.bootTimeMs) : 0ULL;

    double freeHeap = (double)ESP.getFreeHeap();
    uint32_t heapFrag = (uint32_t)ESP.getHeapFragmentation();

    uint64_t btnHoldMs = 0ULL;
    if (g_state.buttonIsHeld && (nowMs >= g_state.buttonPressStartMs)) {
        btnHoldMs = nowMs - g_state.buttonPressStartMs;
        if (btnHoldMs > FACTORY_RESET_HOLD_MS) btnHoldMs = FACTORY_RESET_HOLD_MS;
    }

    uint64_t clientCount = (uint64_t)WiFi.softAPgetStationNum();
    char escapedSSID[70];
    escapeJsonString(escapedSSID, (uint64_t)sizeof(escapedSSID), g_config.apSSID);

    int64_t written = (int64_t)snprintf(
        g_jsonBuf,
        sizeof(g_jsonBuf),
        "{"
          "\"sensorHealthy\":%llu,"
          "\"hasValidSample\":%llu,"
          "\"activeAddr\":\"0x%02X\","
          "\"tempC\":%.2f,\"tempF\":%.2f,"
          "\"rawTempC\":%.2f,\"rawHumPct\":%.2f,\"rawPresHpa\":%.2f,"
          "\"humPct\":%.2f,\"presHpa\":%.2f,"
          "\"dewPointC\":%.2f,\"dewPointF\":%.2f,"
          "\"altitudeM\":%.2f,"
          "\"relayMode\":%llu,\"relayPolarity\":%llu,\"failsafeMode\":%llu,"
          "\"minRelaySwitchMs\":%llu,\"sensorPollMs\":%llu,"
          "\"relays\":["
            "{\"id\":0,\"pin\":14,\"energized\":%llu,\"switches\":%llu,\"cooldown\":%llu},"
            "{\"id\":1,\"pin\":12,\"energized\":%llu,\"switches\":%llu,\"cooldown\":%llu},"
            "{\"id\":2,\"pin\":13,\"energized\":%llu,\"switches\":%llu,\"cooldown\":%llu}"
          "],"
          "\"channels\":["
            "{\"id\":0,\"enabled\":%llu,\"cond\":%llu,\"latched\":%llu,\"val\":%.2f,\"low\":%.2f,\"high\":%.2f,\"hyst\":%.2f,\"off\":%.2f},"
            "{\"id\":1,\"enabled\":%llu,\"cond\":%llu,\"latched\":%llu,\"val\":%.2f,\"low\":%.2f,\"high\":%.2f,\"hyst\":%.2f,\"off\":%.2f},"
            "{\"id\":2,\"enabled\":%llu,\"cond\":%llu,\"latched\":%llu,\"val\":%.2f,\"low\":%.2f,\"high\":%.2f,\"hyst\":%.2f,\"off\":%.2f}"
          "],"
          "\"heapFree\":%.0f,\"heapFrag\":%u,"
          "\"nvsSeq\":%llu,\"nvsWrites\":%llu,\"nvsDirty\":%llu,"
          "\"btnHeld\":%llu,\"btnHoldMs\":%llu,"
          "\"apSSID\":\"%s\",\"apChannel\":%llu,\"wifiClients\":%llu,\"uptimeMs\":%llu"
        "}",
        (unsigned long long)g_state.sensorHealthy,
        (unsigned long long)g_state.hasValidSample,
        (unsigned int)g_state.activeI2CAddr,
        safeJsonDouble(g_state.calTempC), safeJsonDouble(g_state.calTempF),
        safeJsonDouble(g_state.rawTempC), safeJsonDouble(g_state.rawHumPct), safeJsonDouble(g_state.rawPresHpa),
        safeJsonDouble(g_state.calHumPct), safeJsonDouble(g_state.calPresHpa),
        safeJsonDouble(g_state.dewPointC), safeJsonDouble(g_state.dewPointF),
        safeJsonDouble(g_state.altitudeMeters),
        (unsigned long long)g_config.relayMode,
        (unsigned long long)g_config.relayPolarity,
        (unsigned long long)g_config.failsafeMode,
        (unsigned long long)g_config.minRelaySwitchMs,
        (unsigned long long)g_config.sensorPollMs,
        (unsigned long long)g_state.relays[0].isEnergized, (unsigned long long)g_state.relays[0].totalSwitchCount, (unsigned long long)g_state.relays[0].cooldownActive,
        (unsigned long long)g_state.relays[1].isEnergized, (unsigned long long)g_state.relays[1].totalSwitchCount, (unsigned long long)g_state.relays[1].cooldownActive,
        (unsigned long long)g_state.relays[2].isEnergized, (unsigned long long)g_state.relays[2].totalSwitchCount, (unsigned long long)g_state.relays[2].cooldownActive,
        (unsigned long long)g_config.channels[0].enabled, (unsigned long long)g_config.channels[0].condition, (unsigned long long)g_state.channelState[0].isLatchedActive,
        safeJsonDouble(g_state.channelState[0].calibratedValue), safeJsonDouble(g_config.channels[0].lowThreshold), safeJsonDouble(g_config.channels[0].highThreshold),
        safeJsonDouble(g_config.channels[0].hysteresis), safeJsonDouble(g_config.channels[0].offset),
        (unsigned long long)g_config.channels[1].enabled, (unsigned long long)g_config.channels[1].condition, (unsigned long long)g_state.channelState[1].isLatchedActive,
        safeJsonDouble(g_state.channelState[1].calibratedValue), safeJsonDouble(g_config.channels[1].lowThreshold), safeJsonDouble(g_config.channels[1].highThreshold),
        safeJsonDouble(g_config.channels[1].hysteresis), safeJsonDouble(g_config.channels[1].offset),
        (unsigned long long)g_config.channels[2].enabled, (unsigned long long)g_config.channels[2].condition, (unsigned long long)g_state.channelState[2].isLatchedActive,
        safeJsonDouble(g_state.channelState[2].calibratedValue), safeJsonDouble(g_config.channels[2].lowThreshold), safeJsonDouble(g_config.channels[2].highThreshold),
        safeJsonDouble(g_config.channels[2].hysteresis), safeJsonDouble(g_config.channels[2].offset),
        safeJsonDouble(freeHeap), (unsigned int)heapFrag,
        (unsigned long long)g_config.sequence,
        (unsigned long long)g_state.nvsWriteCount,
        (unsigned long long)g_state.nvsDirty,
        (unsigned long long)g_state.buttonIsHeld,
        (unsigned long long)btnHoldMs,
        escapedSSID,
        (unsigned long long)g_config.apChannel,
        (unsigned long long)clientCount,
        (unsigned long long)g_state.systemUptimeMs
    );

    if (written < 0LL || written >= (int64_t)sizeof(g_jsonBuf)) {
        g_webServer.send(500, "application/json", "{\"error\":\"buffer_overflow\"}");
        return;
    }

    g_webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    g_webServer.send(200, "application/json", g_jsonBuf);
}

static void handleApiGetSettings(void) {
    char escapedSSID[70];
    char escapedPass[136];
    escapeJsonString(escapedSSID, (uint64_t)sizeof(escapedSSID), g_config.apSSID);
    escapeJsonString(escapedPass, (uint64_t)sizeof(escapedPass), g_config.apPassword);

    int64_t written = (int64_t)snprintf(
        g_jsonBuf,
        sizeof(g_jsonBuf),
        "{"
          "\"relayMode\":%llu,\"relayPolarity\":%llu,\"failsafeMode\":%llu,"
          "\"minRelaySwitchMs\":%llu,\"sensorPollMs\":%llu,"
          "\"channels\":["
            "{\"enabled\":%llu,\"cond\":%llu,\"low\":%.2f,\"high\":%.2f,\"hyst\":%.2f,\"off\":%.2f},"
            "{\"enabled\":%llu,\"cond\":%llu,\"low\":%.2f,\"high\":%.2f,\"hyst\":%.2f,\"off\":%.2f},"
            "{\"enabled\":%llu,\"cond\":%llu,\"low\":%.2f,\"high\":%.2f,\"hyst\":%.2f,\"off\":%.2f}"
          "],"
          "\"apSSID\":\"%s\",\"apPassword\":\"%s\","
          "\"apChannel\":%llu,\"apMaxClients\":%llu,\"apHideSSID\":%llu"
        "}",
        (unsigned long long)g_config.relayMode,
        (unsigned long long)g_config.relayPolarity,
        (unsigned long long)g_config.failsafeMode,
        (unsigned long long)g_config.minRelaySwitchMs,
        (unsigned long long)g_config.sensorPollMs,
        (unsigned long long)g_config.channels[0].enabled, (unsigned long long)g_config.channels[0].condition,
        safeJsonDouble(g_config.channels[0].lowThreshold), safeJsonDouble(g_config.channels[0].highThreshold),
        safeJsonDouble(g_config.channels[0].hysteresis), safeJsonDouble(g_config.channels[0].offset),
        (unsigned long long)g_config.channels[1].enabled, (unsigned long long)g_config.channels[1].condition,
        safeJsonDouble(g_config.channels[1].lowThreshold), safeJsonDouble(g_config.channels[1].highThreshold),
        safeJsonDouble(g_config.channels[1].hysteresis), safeJsonDouble(g_config.channels[1].offset),
        (unsigned long long)g_config.channels[2].enabled, (unsigned long long)g_config.channels[2].condition,
        safeJsonDouble(g_config.channels[2].lowThreshold), safeJsonDouble(g_config.channels[2].highThreshold),
        safeJsonDouble(g_config.channels[2].hysteresis), safeJsonDouble(g_config.channels[2].offset),
        escapedSSID,
        escapedPass,
        (unsigned long long)g_config.apChannel,
        (unsigned long long)g_config.apMaxClients,
        (unsigned long long)g_config.apHideSSID
    );

    if (written < 0LL || written >= (int64_t)sizeof(g_jsonBuf)) {
        g_webServer.send(500, "application/json", "{\"error\":\"buffer_overflow\"}");
        return;
    }

    g_webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    g_webServer.send(200, "application/json", g_jsonBuf);
}

static void handleApiSaveSettings(void) {
    if (g_webServer.method() != HTTP_POST) {
        g_webServer.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
        return;
    }

    if (!g_webServer.hasArg("plain")) {
        g_webServer.send(400, "application/json", "{\"error\":\"missing_body\"}");
        return;
    }

#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<1536> doc;
#endif
    DeserializationError err = deserializeJson(doc, g_webServer.arg("plain"));
    if (err) {
        g_webServer.send(400, "application/json", "{\"error\":\"invalid_json\"}");
        return;
    }

    SystemConfig tempCfg;
    memcpy(&tempCfg, &g_config, sizeof(SystemConfig));

    if (!doc["relayMode"].isNull())        tempCfg.relayMode        = doc["relayMode"].as<uint64_t>();
    if (!doc["relayPolarity"].isNull())    tempCfg.relayPolarity    = doc["relayPolarity"].as<uint64_t>();
    if (!doc["failsafeMode"].isNull())     tempCfg.failsafeMode     = doc["failsafeMode"].as<uint64_t>();
    if (!doc["minRelaySwitchMs"].isNull()) tempCfg.minRelaySwitchMs = doc["minRelaySwitchMs"].as<uint64_t>();
    if (!doc["sensorPollMs"].isNull())     tempCfg.sensorPollMs     = doc["sensorPollMs"].as<uint64_t>();

    if (doc["channels"].is<JsonArray>()) {
        JsonArray arr = doc["channels"].as<JsonArray>();
        uint64_t count = (uint64_t)arr.size();
        for (uint64_t i = 0ULL; i < 3ULL && i < count; ++i) {
            JsonObject chObj = arr[(size_t)i];
            SensorChannelConfig* ch = &tempCfg.channels[i];
            if (!chObj["enabled"].isNull()) ch->enabled       = chObj["enabled"].as<uint64_t>();
            if (!chObj["cond"].isNull())    ch->condition     = chObj["cond"].as<uint64_t>();
            if (!chObj["low"].isNull())     ch->lowThreshold  = chObj["low"].as<double>();
            if (!chObj["high"].isNull())    ch->highThreshold = chObj["high"].as<double>();
            if (!chObj["hyst"].isNull())    ch->hysteresis    = chObj["hyst"].as<double>();
            if (!chObj["off"].isNull())     ch->offset        = chObj["off"].as<double>();
        }
    }

    sanitizeAndClampConfig(&tempCfg);

    if (!validateConfig(&tempCfg)) {
        g_webServer.send(400, "application/json", "{\"error\":\"validation_failed\"}");
        return;
    }

    for (uint64_t i = 0ULL; i < 3ULL; ++i) {
        if (tempCfg.channels[i].condition != g_config.channels[i].condition ||
            tempCfg.channels[i].enabled != g_config.channels[i].enabled) {
            g_state.channelState[i].isLatchedActive = 0ULL;
        }
    }

    stageConfigSave(&tempCfg);
    g_webServer.send(200, "application/json", "{\"status\":\"ok\"}");
}

static void handleApiSaveWiFi(void) {
    if (g_webServer.method() != HTTP_POST) {
        g_webServer.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
        return;
    }

    if (!g_webServer.hasArg("plain")) {
        g_webServer.send(400, "application/json", "{\"error\":\"missing_body\"}");
        return;
    }

#if ARDUINOJSON_VERSION_MAJOR >= 7
    JsonDocument doc;
#else
    StaticJsonDocument<512> doc;
#endif
    DeserializationError err = deserializeJson(doc, g_webServer.arg("plain"));
    if (err) {
        g_webServer.send(400, "application/json", "{\"error\":\"invalid_json\"}");
        return;
    }

    const char* ssid = doc["apSSID"] | "";
    const char* pass = doc["apPassword"] | "";
    uint64_t chan    = doc["apChannel"] | g_config.apChannel;
    uint64_t maxC    = doc["apMaxClients"] | g_config.apMaxClients;
    uint64_t hide    = doc["apHideSSID"] | g_config.apHideSSID;

    uint64_t ssidLen = (uint64_t)strlen(ssid);
    uint64_t passLen = (uint64_t)strlen(pass);

    if (ssidLen < 1ULL || ssidLen > 32ULL) {
        g_webServer.send(400, "application/json", "{\"error\":\"invalid_ssid\"}");
        return;
    }

    if (passLen != 0ULL && (passLen < 8ULL || passLen > 63ULL)) {
        g_webServer.send(400, "application/json", "{\"error\":\"invalid_password\"}");
        return;
    }

    SystemConfig tempCfg;
    memcpy(&tempCfg, &g_config, sizeof(SystemConfig));

    memset(tempCfg.apSSID, 0, sizeof(tempCfg.apSSID));
    strncpy(tempCfg.apSSID, ssid, sizeof(tempCfg.apSSID) - 1);

    if (passLen >= 8ULL) {
        memset(tempCfg.apPassword, 0, sizeof(tempCfg.apPassword));
        strncpy(tempCfg.apPassword, pass, sizeof(tempCfg.apPassword) - 1);
    }

    tempCfg.apChannel    = clampUint64(chan, 1ULL, 13ULL);
    tempCfg.apMaxClients = clampUint64(maxC, 1ULL, 8ULL);
    tempCfg.apHideSSID   = (hide != 0ULL) ? 1ULL : 0ULL;
    tempCfg.crc64        = computeConfigChecksum(&tempCfg);

    if (!validateConfig(&tempCfg)) {
        g_webServer.send(400, "application/json", "{\"error\":\"validation_failed\"}");
        return;
    }

    stageConfigSave(&tempCfg);
    writeConfigToNvsNow();

    g_state.wifiReloadPending  = 1ULL;
    g_state.wifiReloadTargetMs = getMillis64() + ACTION_DELAY_MS;

    g_webServer.send(200, "application/json", "{\"status\":\"ok\",\"reloadingWiFi\":true}");
}

static void handleApiReset(void) {
    if (g_webServer.method() != HTTP_POST) {
        g_webServer.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
        return;
    }

    for (uint64_t i = 0ULL; i < 3ULL; ++i) {
        g_state.relays[i].isEnergized = 0ULL;
        writeRelayHardwarePins(i, 0ULL);
    }

    applyDefaultSettings(&g_config);
    writeConfigToNvsNow();

    for (uint64_t i = 0ULL; i < 3ULL; ++i) {
        writeRelayHardwarePins(i, 0ULL);
    }

    g_state.rebootPending  = 1ULL;
    g_state.rebootTargetMs = getMillis64() + ACTION_DELAY_MS;

    g_webServer.send(200, "application/json", "{\"status\":\"ok\",\"rebooting\":true}");
}

static void handleApiReboot(void) {
    if (g_webServer.method() != HTTP_POST) {
        g_webServer.send(405, "application/json", "{\"error\":\"method_not_allowed\"}");
        return;
    }

    if (g_state.nvsDirty) {
        writeConfigToNvsNow();
    }

    g_state.rebootPending  = 1ULL;
    g_state.rebootTargetMs = getMillis64() + ACTION_DELAY_MS;

    g_webServer.send(200, "application/json", "{\"status\":\"ok\",\"rebooting\":true}");
}

static void serveSpaIndex(void) {
    if (isCaptiveRedirectRequired()) {
        handleCaptiveRedirect();
        return;
    }
    g_webServer.sendHeader("Cache-Control", "no-cache, no-store, must-revalidate");
    g_webServer.send_P(200, "text/html", INDEX_HTML);
}

static void registerWebRoutes(void) {
    g_webServer.collectHeaders("Host");

    g_webServer.on("/", HTTP_GET, serveSpaIndex);
    g_webServer.on("/dashboard", HTTP_GET, serveSpaIndex);
    g_webServer.on("/settings", HTTP_GET, serveSpaIndex);
    g_webServer.on("/wifi", HTTP_GET, serveSpaIndex);

    g_webServer.on("/favicon.ico", HTTP_GET, []() {
        g_webServer.send(204, "image/x-icon", "");
    });

    g_webServer.on("/generate_204", HTTP_GET, handleCaptiveRedirect);
    g_webServer.on("/gen_204", HTTP_GET, handleCaptiveRedirect);
    g_webServer.on("/hotspot-detect.html", HTTP_GET, handleCaptiveRedirect);
    g_webServer.on("/library/test/success.html", HTTP_GET, handleCaptiveRedirect);
    g_webServer.on("/ncsi.txt", HTTP_GET, handleCaptiveRedirect);
    g_webServer.on("/connecttest.txt", HTTP_GET, handleCaptiveRedirect);
    g_webServer.on("/redirect", HTTP_GET, handleCaptiveRedirect);
    g_webServer.on("/success.txt", HTTP_GET, handleCaptiveRedirect);
    g_webServer.on("/canonical.html", HTTP_GET, handleCaptiveRedirect);
    g_webServer.on("/check_network_status.txt", HTTP_GET, handleCaptiveRedirect);

    g_webServer.on("/api/status", HTTP_GET, handleApiStatus);
    g_webServer.on("/api/settings", HTTP_GET, handleApiGetSettings);
    g_webServer.on("/api/settings", HTTP_POST, handleApiSaveSettings);
    g_webServer.on("/api/wifi", HTTP_POST, handleApiSaveWiFi);
    g_webServer.on("/api/reset", HTTP_POST, handleApiReset);
    g_webServer.on("/api/reboot", HTTP_POST, handleApiReboot);

    g_webServer.onNotFound([]() {
        handleCaptiveRedirect();
    });

    g_webServer.begin();
}

// =====================================================================================
// SINGLE-PAGE APPLICATION (SPA) RESPONSIVE MOBILE & DESKTOP FRONTEND (PROGMEM)
// =====================================================================================
const char INDEX_HTML[] PROGMEM = R"rawliteral(
<!DOCTYPE html>
<html lang="en">
<head>
<meta charset="UTF-8">
<meta name="viewport" content="width=device-width, initial-scale=1.0, maximum-scale=1.0, user-scalable=no">
<title>ESP8266 BME280 Controller</title>
<style>
:root {
  --bg-base: #090d16;
  --bg-card: #121a2b;
  --bg-card-alt: #172238;
  --border-color: #243454;
  --accent-cyan: #00f2fe;
  --accent-blue: #4facfe;
  --accent-green: #00e676;
  --accent-red: #ff5252;
  --accent-amber: #ffab00;
  --text-main: #f1f5f9;
  --text-muted: #94a3b8;
  --font-stack: -apple-system, BlinkMacSystemFont, "Segoe UI", Roboto, Helvetica, Arial, sans-serif;
}
* { box-sizing: border-box; margin: 0; padding: 0; }
body {
  background-color: var(--bg-base);
  color: var(--text-main);
  font-family: var(--font-stack);
  line-height: 1.5;
  padding-bottom: 40px;
  -webkit-font-smoothing: antialiased;
}
.header {
  background: linear-gradient(135deg, #0d1526 0%, #1b2845 100%);
  border-bottom: 1px solid var(--border-color);
  padding: 14px 20px;
  display: flex;
  justify-content: space-between;
  align-items: center;
  position: sticky;
  top: 0;
  z-index: 100;
  box-shadow: 0 4px 20px rgba(0,0,0,0.45);
}
.brand { display: flex; align-items: center; gap: 12px; }
.brand-logo { width: 30px; height: 30px; fill: var(--accent-cyan); flex-shrink: 0; }
.brand-title {
  font-size: 1.15rem;
  font-weight: 800;
  letter-spacing: 0.5px;
  background: linear-gradient(90deg, var(--accent-cyan), var(--accent-blue));
  -webkit-background-clip: text;
  -webkit-text-fill-color: transparent;
}
.brand-sub { font-size: 0.72rem; color: var(--text-muted); display: block; }
.status-group { display: flex; align-items: center; gap: 10px; }
.status-pill {
  display: inline-flex;
  align-items: center;
  gap: 7px;
  padding: 5px 12px;
  border-radius: 999px;
  font-size: 0.78rem;
  font-weight: 700;
  background: rgba(255,255,255,0.05);
  border: 1px solid var(--border-color);
}
.dot { width: 8px; height: 8px; border-radius: 50%; background-color: var(--text-muted); }
.dot.online { background-color: var(--accent-green); box-shadow: 0 0 8px var(--accent-green); }
.dot.warn { background-color: var(--accent-amber); box-shadow: 0 0 8px var(--accent-amber); }
.dot.offline { background-color: var(--accent-red); box-shadow: 0 0 8px var(--accent-red); }

.nav-tabs {
  display: flex;
  background: #0c1220;
  border-bottom: 1px solid var(--border-color);
  padding: 0 12px;
  overflow-x: auto;
}
.tab-btn {
  background: none;
  border: none;
  color: var(--text-muted);
  padding: 14px 18px;
  font-size: 0.92rem;
  font-weight: 700;
  cursor: pointer;
  border-bottom: 3px solid transparent;
  transition: all 0.2s;
  white-space: nowrap;
}
.tab-btn:hover { color: var(--text-main); }
.tab-btn.active { color: var(--accent-cyan); border-bottom-color: var(--accent-cyan); }

.container { max-width: 1220px; margin: 20px auto; padding: 0 16px; }
.tab-content { display: none; }
.tab-content.active { display: block; }

.grid-3 { display: grid; grid-template-columns: repeat(auto-fit, minmax(310px, 1fr)); gap: 18px; }
.grid-2 { display: grid; grid-template-columns: repeat(auto-fit, minmax(420px, 1fr)); gap: 18px; }

.card {
  background-color: var(--bg-card);
  border: 1px solid var(--border-color);
  border-radius: 12px;
  padding: 20px;
  box-shadow: 0 8px 24px rgba(0,0,0,0.25);
}
.card-title {
  font-size: 0.85rem;
  font-weight: 700;
  text-transform: uppercase;
  letter-spacing: 0.8px;
  color: var(--text-muted);
  margin-bottom: 12px;
  display: flex;
  justify-content: space-between;
  align-items: center;
}
.badge {
  padding: 3px 9px;
  border-radius: 6px;
  font-size: 0.72rem;
  font-weight: 700;
  text-transform: uppercase;
}
.badge-ok { background: rgba(0,230,118,0.12); color: var(--accent-green); border: 1px solid rgba(0,230,118,0.35); }
.badge-trig { background: rgba(255,82,82,0.15); color: var(--accent-red); border: 1px solid rgba(255,82,82,0.4); }
.badge-off { background: rgba(148,163,184,0.12); color: var(--text-muted); border: 1px solid rgba(148,163,184,0.25); }
.badge-info { background: rgba(0,242,254,0.12); color: var(--accent-cyan); border: 1px solid rgba(0,242,254,0.35); }

.metric-val { font-size: 2.65rem; font-weight: 800; color: var(--text-main); line-height: 1.05; }
.metric-unit { font-size: 1.15rem; font-weight: 600; color: var(--text-muted); margin-left: 5px; }
.metric-sub { font-size: 0.85rem; color: var(--text-muted); margin-top: 6px; display: flex; justify-content: space-between; }

.progress-wrap { margin-top: 12px; }
.progress-labels { display: flex; justify-content: space-between; font-size: 0.74rem; color: var(--text-muted); margin-bottom: 4px; }
.progress-bar { width: 100%; height: 6px; background: rgba(255,255,255,0.08); border-radius: 3px; overflow: hidden; }
.progress-fill { height: 100%; background: linear-gradient(90deg, var(--accent-cyan), var(--accent-blue)); width: 0%; transition: width 0.3s ease; }
.progress-fill.trig { background: linear-gradient(90deg, var(--accent-amber), var(--accent-red)); }

.sparkline-container { height: 58px; margin-top: 14px; }
canvas { width: 100% !important; height: 100% !important; display: block; }

.relay-status-grid { display: grid; grid-template-columns: repeat(auto-fit, minmax(150px, 1fr)); gap: 12px; margin-top: 12px; }
.relay-box {
  background: var(--bg-card-alt);
  border: 1px solid var(--border-color);
  border-radius: 10px;
  padding: 14px 12px;
  text-align: center;
  transition: all 0.2s;
}
.relay-box.active {
  border-color: var(--accent-green);
  background: rgba(0,230,118,0.07);
  box-shadow: 0 0 16px rgba(0,230,118,0.12);
}
.relay-title { font-size: 0.78rem; color: var(--text-muted); text-transform: uppercase; font-weight: 700; }
.relay-pin { font-size: 0.72rem; color: var(--accent-cyan); margin-top: 2px; }
.relay-state { font-size: 1.15rem; font-weight: 800; margin-top: 6px; }
.relay-state.ON { color: var(--accent-green); }
.relay-state.OFF { color: var(--text-muted); }

.form-group { margin-bottom: 14px; }
.form-group label { display: block; font-size: 0.82rem; font-weight: 600; color: var(--text-muted); margin-bottom: 5px; }
.form-control {
  width: 100%;
  background: #090d16;
  border: 1px solid var(--border-color);
  border-radius: 7px;
  padding: 10px 12px;
  color: var(--text-main);
  font-size: 0.94rem;
  outline: none;
  transition: border-color 0.2s;
}
.form-control:focus { border-color: var(--accent-cyan); }
.form-row { display: grid; grid-template-columns: 1fr 1fr; gap: 12px; }
.pass-wrap { display: flex; gap: 8px; }
.btn-toggle-pass {
  background: var(--bg-card-alt);
  border: 1px solid var(--border-color);
  color: var(--text-main);
  border-radius: 7px;
  padding: 0 14px;
  font-size: 0.8rem;
  font-weight: 600;
  cursor: pointer;
}

.btn {
  background: linear-gradient(135deg, var(--accent-cyan), var(--accent-blue));
  color: #06101e;
  border: none;
  border-radius: 8px;
  padding: 12px 20px;
  font-size: 0.95rem;
  font-weight: 800;
  cursor: pointer;
  transition: opacity 0.2s, transform 0.1s;
  width: 100%;
}
.btn:hover { opacity: 0.92; }
.btn:active { transform: scale(0.99); }
.btn-danger { background: linear-gradient(135deg, #ff5252, #d50000); color: #fff; }
.btn-warning { background: linear-gradient(135deg, #ffab00, #ff6d00); color: #090d16; }
.btn-secondary { background: var(--bg-card-alt); color: var(--text-main); border: 1px solid var(--border-color); }

.sys-info-table { width: 100%; border-collapse: collapse; margin-top: 6px; }
.sys-info-table td { padding: 8px 0; border-bottom: 1px solid rgba(255,255,255,0.06); font-size: 0.88rem; }
.sys-info-table td:nth-child(2) { text-align: right; font-weight: 700; color: var(--text-main); }
.sys-info-table td:nth-child(1) { color: var(--text-muted); }

.toast {
  position: fixed;
  bottom: 20px;
  right: 20px;
  background: var(--bg-card);
  border: 1px solid var(--accent-cyan);
  color: var(--text-main);
  padding: 13px 20px;
  border-radius: 8px;
  font-size: 0.9rem;
  font-weight: 600;
  box-shadow: 0 10px 30px rgba(0,0,0,0.6);
  display: none;
  z-index: 2000;
}

.modal-backdrop {
  position: fixed;
  inset: 0;
  background: rgba(0,0,0,0.75);
  display: none;
  align-items: center;
  justify-content: center;
  z-index: 1500;
  padding: 16px;
}
.modal-card {
  background: var(--bg-card);
  border: 1px solid var(--border-color);
  border-radius: 12px;
  max-width: 420px;
  width: 100%;
  padding: 22px;
  box-shadow: 0 16px 40px rgba(0,0,0,0.6);
}
.modal-title { font-size: 1.1rem; font-weight: 800; margin-bottom: 8px; }
.modal-desc { font-size: 0.9rem; color: var(--text-muted); margin-bottom: 18px; }
.modal-actions { display: flex; gap: 12px; }

@media (max-width: 768px) {
  .grid-2, .grid-3 { grid-template-columns: 1fr; }
  .form-row { grid-template-columns: 1fr; }
  .header { padding: 12px 14px; }
}
</style>
</head>
<body>

<header class="header">
  <div class="brand">
    <svg class="brand-logo" viewBox="0 0 24 24"><path d="M12 2C6.48 2 2 6.48 2 12s4.48 10 10 10 10-4.48 10-10S17.52 2 12 2zm-1 17.93c-3.95-.49-7-3.85-7-7.93 0-.62.08-1.21.21-1.79L9 15v1c0 1.1.9 2 2 2v1.93zm6.9-2.54c-.26-.81-1-1.39-1.9-1.39h-1v-3c0-.55-.45-1-1-1H8v-2h2c.55 0 1-.45 1-1V7h2c1.1 0 2-.9 2-2v-.41c2.93 1.19 5 4.06 5 7.41 0 2.08-.8 3.97-2.1 5.39z"/></svg>
    <div>
      <div class="brand-title">ESP8266 BME280 CONTROLLER</div>
      <span class="brand-sub">64-Bit Non-Blocking Environmental &amp; Relay Engine</span>
    </div>
  </div>
  <div class="status-group">
    <div class="status-pill">
      <div id="statusDot" class="dot offline"></div>
      <span id="statusText">CONNECTING</span>
    </div>
  </div>
</header>

<nav class="nav-tabs">
  <button class="tab-btn active" id="tabBtn-dashboard" onclick="switchTab('dashboard')">Live Dashboard</button>
  <button class="tab-btn" id="tabBtn-settings" onclick="switchTab('settings')">BME280 &amp; Relay Settings</button>
  <button class="tab-btn" id="tabBtn-wifi" onclick="switchTab('wifi')">Wi-Fi AP &amp; System</button>
</nav>

<div class="container">
  <!-- VIEW 1: LIVE DASHBOARD -->
  <div id="tab-dashboard" class="tab-content active">
    <div class="grid-3">
      <!-- Temperature Card -->
      <div class="card">
        <div class="card-title">
          <span>Temperature (GPIO 14 / D5)</span>
          <span id="chBadge0" class="badge badge-ok">NORMAL</span>
        </div>
        <div>
          <span id="valTempC" class="metric-val">--.-</span><span class="metric-unit">&deg;C</span>
        </div>
        <div class="metric-sub">
          <span><strong id="valTempF">--.-</strong> &deg;F</span>
          <span>Offset: <span id="valOff0">0.00</span> &deg;C</span>
        </div>
        <div class="progress-wrap">
          <div class="progress-labels">
            <span>Low: <span id="lblLow0">--</span>&deg;C</span>
            <span id="lblCond0">Above High</span>
            <span>High: <span id="lblHigh0">--</span>&deg;C</span>
          </div>
          <div class="progress-bar"><div id="progFill0" class="progress-fill"></div></div>
        </div>
        <div class="sparkline-container"><canvas id="sparkTemp"></canvas></div>
      </div>

      <!-- Humidity Card -->
      <div class="card">
        <div class="card-title">
          <span>Humidity (GPIO 12 / D6)</span>
          <span id="chBadge1" class="badge badge-ok">NORMAL</span>
        </div>
        <div>
          <span id="valHum" class="metric-val">--.-</span><span class="metric-unit">%RH</span>
        </div>
        <div class="metric-sub">
          <span>Dew Point: <strong id="valDewC">--.-</strong> &deg;C (<span id="valDewF">--.-</span> &deg;F)</span>
          <span>Offset: <span id="valOff1">0.00</span> %</span>
        </div>
        <div class="progress-wrap">
          <div class="progress-labels">
            <span>Low: <span id="lblLow1">--</span>%</span>
            <span id="lblCond1">Above High</span>
            <span>High: <span id="lblHigh1">--</span>%</span>
          </div>
          <div class="progress-bar"><div id="progFill1" class="progress-fill"></div></div>
        </div>
        <div class="sparkline-container"><canvas id="sparkHum"></canvas></div>
      </div>

      <!-- Pressure Card -->
      <div class="card">
        <div class="card-title">
          <span>Pressure (GPIO 13 / D7)</span>
          <span id="chBadge2" class="badge badge-ok">NORMAL</span>
        </div>
        <div>
          <span id="valPres" class="metric-val">----.-</span><span class="metric-unit">hPa</span>
        </div>
        <div class="metric-sub">
          <span>Est. Altitude: <strong id="valAlt">---</strong> m</span>
          <span>Offset: <span id="valOff2">0.00</span> hPa</span>
        </div>
        <div class="progress-wrap">
          <div class="progress-labels">
            <span>Low: <span id="lblLow2">--</span> hPa</span>
            <span id="lblCond2">Below Low</span>
            <span>High: <span id="lblHigh2">--</span> hPa</span>
          </div>
          <div class="progress-bar"><div id="progFill2" class="progress-fill"></div></div>
        </div>
        <div class="sparkline-container"><canvas id="sparkPres"></canvas></div>
      </div>
    </div>

    <div class="grid-2" style="margin-top: 18px;">
      <!-- Relay Outputs Card -->
      <div class="card">
        <div class="card-title">
          <span>Relay Output Matrix</span>
          <span id="valRelayModeBadge" class="badge badge-info">UNIFIED MODE</span>
        </div>
        <div style="font-size: 0.84rem; color: var(--text-muted); margin-bottom: 10px;">
          Polarity: <strong id="valRelayPolarity" style="color:var(--text-main);">ACTIVE HIGH</strong> &bull;
          Failsafe: <strong id="valFailsafe" style="color:var(--text-main);">FORCE OFF</strong> &bull;
          Hold Guard: <strong id="valHoldTime" style="color:var(--text-main);">3000 ms</strong>
        </div>
        <div class="relay-status-grid">
          <div id="relayBox0" class="relay-box">
            <div class="relay-title" id="relayTitle0">Unified / Temp</div>
            <div class="relay-pin">ESP8266 GPIO 14 (D5)</div>
            <div id="relayState0" class="relay-state OFF">DE-ENERGIZED</div>
            <div style="font-size:0.75rem; color:var(--text-muted); margin-top:6px;">
              Switches: <strong id="relayCnt0">0</strong> <span id="relayWait0"></span>
            </div>
          </div>
          <div id="relayBox1" class="relay-box">
            <div class="relay-title" id="relayTitle1">Humidity Relay</div>
            <div class="relay-pin">ESP8266 GPIO 12 (D6)</div>
            <div id="relayState1" class="relay-state OFF">DE-ENERGIZED</div>
            <div style="font-size:0.75rem; color:var(--text-muted); margin-top:6px;">
              Switches: <strong id="relayCnt1">0</strong> <span id="relayWait1"></span>
            </div>
          </div>
          <div id="relayBox2" class="relay-box">
            <div class="relay-title" id="relayTitle2">Pressure Relay</div>
            <div class="relay-pin">ESP8266 GPIO 13 (D7)</div>
            <div id="relayState2" class="relay-state OFF">DE-ENERGIZED</div>
            <div style="font-size:0.75rem; color:var(--text-muted); margin-top:6px;">
              Switches: <strong id="relayCnt2">0</strong> <span id="relayWait2"></span>
            </div>
          </div>
        </div>
      </div>

      <!-- System Health & Longevity Metrics -->
      <div class="card">
        <div class="card-title">
          <span>System Diagnostics &amp; Longevity</span>
          <span id="sensorHealthBadge" class="badge badge-ok">I2C HEALTHY</span>
        </div>
        <table class="sys-info-table">
          <tr><td>BME280 Bus (GPIO 5 SDA / GPIO 4 SCL)</td><td id="sysI2CAddr">--</td></tr>
          <tr><td>RAM Heap (Free / Minimum)</td><td id="sysHeap">--</td></tr>
          <tr><td>Dual-Bank EEPROM Storage</td><td id="sysNVS">--</td></tr>
          <tr><td>GPIO 0 Factory Reset Button (5s Hold)</td><td id="sysBtn">Released (0 ms)</td></tr>
          <tr><td>Active Wi-Fi AP SSID / Clients</td><td id="sysClients">--</td></tr>
          <tr><td>Continuous 64-Bit System Uptime</td><td id="sysUptime">0d 00:00:00</td></tr>
        </table>
      </div>
    </div>
  </div>

  <!-- VIEW 2: BME280 & RELAY SETTINGS -->
  <div id="tab-settings" class="tab-content">
    <form id="formSettings" onsubmit="saveSettings(event)">
      <div class="grid-2">
        <!-- Global Relay Engine Settings -->
        <div class="card">
          <div class="card-title"><span>Relay Control Architecture</span><span class="badge badge-info">GPIO 14 / 12 / 13 (D5 / D6 / D7)</span></div>
          <div class="form-group">
            <label>Relay Operating Mode</label>
            <select id="cfgRelayMode" class="form-control">
              <option value="0">Unified Mode (GPIO 14 / D5 Triggers on Any Sensor Threshold)</option>
              <option value="1">Individual Mode (GPIO 14 / D5 Temp, GPIO 12 / D6 Humidity, GPIO 13 / D7 Pressure)</option>
            </select>
          </div>
          <div class="form-row">
            <div class="form-group">
              <label>Relay Output Polarity</label>
              <select id="cfgRelayPolarity" class="form-control">
                <option value="0">Active HIGH (GPIO HIGH = Relay ON)</option>
                <option value="1">Active LOW (GPIO LOW = Relay ON)</option>
              </select>
            </div>
            <div class="form-group">
              <label>Sensor Fault Failsafe Behavior</label>
              <select id="cfgFailsafeMode" class="form-control">
                <option value="0">Force OFF All Relays</option>
                <option value="1">Hold Last Relay State</option>
                <option value="2">Force ON Active Relays</option>
              </select>
            </div>
          </div>
          <div class="form-row">
            <div class="form-group">
              <label>Min Relay Anti-Short-Cycle Hold (100 - 3600000 ms)</label>
              <input type="number" id="cfgMinSwitchMs" class="form-control" min="100" max="3600000" step="1" required>
            </div>
            <div class="form-group">
              <label>BME280 Polling Interval (250 - 300000 ms)</label>
              <input type="number" id="cfgPollMs" class="form-control" min="250" max="300000" step="1" required>
            </div>
          </div>
        </div>

        <!-- Channel 0: Temperature -->
        <div class="card">
          <div class="card-title"><span>Temperature Thresholds &amp; Calibration</span><span class="badge badge-info">GPIO 14 (D5)</span></div>
          <div class="form-row">
            <div class="form-group">
              <label>Temperature Trigger Enable</label>
              <select id="ch0_enabled" class="form-control">
                <option value="1">Enabled</option>
                <option value="0">Disabled</option>
              </select>
            </div>
            <div class="form-group">
              <label>Trigger Condition</label>
              <select id="ch0_cond" class="form-control">
                <option value="0">Above High Threshold (&ge; High)</option>
                <option value="1">Below Low Threshold (&le; Low)</option>
                <option value="2">Outside Range (&le; Low or &ge; High)</option>
                <option value="3">Inside Range (&ge; Low and &le; High)</option>
              </select>
            </div>
          </div>
          <div class="form-row">
            <div class="form-group">
              <label>Low Threshold (-40.0 to 85.0 &deg;C)</label>
              <input type="number" id="ch0_low" class="form-control" min="-40" max="85" step="0.01" required>
            </div>
            <div class="form-group">
              <label>High Threshold (-40.0 to 85.0 &deg;C)</label>
              <input type="number" id="ch0_high" class="form-control" min="-40" max="85" step="0.01" required>
            </div>
          </div>
          <div class="form-row">
            <div class="form-group">
              <label>Hysteresis Deadband (0.05 to 25.0 &deg;C)</label>
              <input type="number" id="ch0_hyst" class="form-control" min="0.05" max="25" step="0.01" required>
            </div>
            <div class="form-group">
              <label>Calibration Offset (-25.0 to +25.0 &deg;C)</label>
              <input type="number" id="ch0_off" class="form-control" min="-25" max="25" step="0.01" required>
            </div>
          </div>
        </div>

        <!-- Channel 1: Humidity -->
        <div class="card">
          <div class="card-title"><span>Humidity Thresholds &amp; Calibration</span><span class="badge badge-info">GPIO 14 / 12 (D5/D6)</span></div>
          <div class="form-row">
            <div class="form-group">
              <label>Humidity Trigger Enable</label>
              <select id="ch1_enabled" class="form-control">
                <option value="1">Enabled</option>
                <option value="0">Disabled</option>
              </select>
            </div>
            <div class="form-group">
              <label>Trigger Condition</label>
              <select id="ch1_cond" class="form-control">
                <option value="0">Above High Threshold (&ge; High)</option>
                <option value="1">Below Low Threshold (&le; Low)</option>
                <option value="2">Outside Range (&le; Low or &ge; High)</option>
                <option value="3">Inside Range (&ge; Low and &le; High)</option>
              </select>
            </div>
          </div>
          <div class="form-row">
            <div class="form-group">
              <label>Low Threshold (0.0 to 100.0 %RH)</label>
              <input type="number" id="ch1_low" class="form-control" min="0" max="100" step="0.01" required>
            </div>
            <div class="form-group">
              <label>High Threshold (0.0 to 100.0 %RH)</label>
              <input type="number" id="ch1_high" class="form-control" min="0" max="100" step="0.01" required>
            </div>
          </div>
          <div class="form-row">
            <div class="form-group">
              <label>Hysteresis Deadband (0.05 to 40.0 %RH)</label>
              <input type="number" id="ch1_hyst" class="form-control" min="0.05" max="40" step="0.01" required>
            </div>
            <div class="form-group">
              <label>Calibration Offset (-40.0 to +40.0 %RH)</label>
              <input type="number" id="ch1_off" class="form-control" min="-40" max="40" step="0.01" required>
            </div>
          </div>
        </div>

        <!-- Channel 2: Pressure -->
        <div class="card">
          <div class="card-title"><span>Pressure Thresholds &amp; Calibration</span><span class="badge badge-info">GPIO 14 / 13 (D5/D7)</span></div>
          <div class="form-row">
            <div class="form-group">
              <label>Pressure Trigger Enable</label>
              <select id="ch2_enabled" class="form-control">
                <option value="1">Enabled</option>
                <option value="0">Disabled</option>
              </select>
            </div>
            <div class="form-group">
              <label>Trigger Condition</label>
              <select id="ch2_cond" class="form-control">
                <option value="0">Above High Threshold (&ge; High)</option>
                <option value="1">Below Low Threshold (&le; Low)</option>
                <option value="2">Outside Range (&le; Low or &ge; High)</option>
                <option value="3">Inside Range (&ge; Low and &le; High)</option>
              </select>
            </div>
          </div>
          <div class="form-row">
            <div class="form-group">
              <label>Low Threshold (300.0 to 1100.0 hPa)</label>
              <input type="number" id="ch2_low" class="form-control" min="300" max="1100" step="0.01" required>
            </div>
            <div class="form-group">
              <label>High Threshold (300.0 to 1100.0 hPa)</label>
              <input type="number" id="ch2_high" class="form-control" min="300" max="1100" step="0.01" required>
            </div>
          </div>
          <div class="form-row">
            <div class="form-group">
              <label>Hysteresis Deadband (0.1 to 100.0 hPa)</label>
              <input type="number" id="ch2_hyst" class="form-control" min="0.1" max="100" step="0.01" required>
            </div>
            <div class="form-group">
              <label>Calibration Offset (-100.0 to +100.0 hPa)</label>
              <input type="number" id="ch2_off" class="form-control" min="-100" max="100" step="0.01" required>
            </div>
          </div>
        </div>
      </div>
      <div style="margin-top: 18px;">
        <button type="submit" class="btn">Save &amp; Apply Sensor / Relay Configuration</button>
      </div>
    </form>
  </div>

  <!-- VIEW 3: WI-FI AP & SYSTEM MANAGEMENT -->
  <div id="tab-wifi" class="tab-content">
    <div class="grid-2">
      <!-- Wi-Fi AP Settings -->
      <div class="card">
        <div class="card-title"><span>Wi-Fi Captive Portal Access Point</span><span class="badge badge-info">192.168.4.1</span></div>
        <form id="formWiFi" onsubmit="saveWiFi(event)">
          <div class="form-group">
            <label>Access Point SSID (1 - 32 Characters)</label>
            <input type="text" id="wifiSSID" class="form-control" minlength="1" maxlength="32" required>
          </div>
          <div class="form-group">
            <label>WPA2 Password (8 - 63 Characters)</label>
            <div class="pass-wrap">
              <input type="password" id="wifiPass" class="form-control" minlength="8" maxlength="63" required>
              <button type="button" class="btn-toggle-pass" onclick="togglePassVisibility()">SHOW</button>
            </div>
          </div>
          <div class="form-row">
            <div class="form-group">
              <label>Wi-Fi RF Channel (1 - 13)</label>
              <input type="number" id="wifiChan" class="form-control" min="1" max="13" step="1" required>
            </div>
            <div class="form-group">
              <label>Max Simultaneous Clients (1 - 8)</label>
              <input type="number" id="wifiMax" class="form-control" min="1" max="8" step="1" required>
            </div>
          </div>
          <div class="form-group">
            <label>SSID Broadcast Visibility</label>
            <select id="wifiHide" class="form-control">
              <option value="0">Broadcast SSID Publicly (Visible)</option>
              <option value="1">Hide SSID Broadcast (Hidden)</option>
            </select>
          </div>
          <button type="submit" class="btn">Save &amp; Reload Wi-Fi Access Point</button>
        </form>
      </div>

      <!-- System Maintenance & Hardware Reset -->
      <div class="card">
        <div class="card-title"><span>System Maintenance &amp; Factory Reset</span><span class="badge badge-info">GPIO 0 / D3 BOOT</span></div>
        <div style="margin-bottom: 18px;">
          <p style="font-size: 0.88rem; color: var(--text-muted); margin-bottom: 12px;">
            Commit any pending EEPROM settings to flash and perform a clean, non-blocking microcontroller reboot.
          </p>
          <button type="button" class="btn btn-warning" onclick="openActionModal('reboot')">Reboot ESP8266 Controller</button>
        </div>
        <hr style="border:0; border-top:1px solid var(--border-color); margin: 18px 0;">
        <div>
          <p style="font-size: 0.88rem; color: var(--text-muted); margin-bottom: 12px;">
            Restore all sensor thresholds, relay modes, and Wi-Fi AP credentials to factory defaults (SSID: <strong>ESP8266_BME280</strong>, Pass: <strong>12345678</strong>). You can also trigger this at any time by holding the onboard <strong>GPIO 0 / D3 BOOT</strong> button for 5 seconds.
          </p>
          <button type="button" class="btn btn-danger" onclick="openActionModal('reset')">Restore Factory Defaults</button>
        </div>
      </div>
    </div>
  </div>
</div>

<!-- NON-BLOCKING CONFIRMATION MODAL -->
<div id="actionModal" class="modal-backdrop">
  <div class="modal-card">
    <div id="modalTitle" class="modal-title">Confirm Action</div>
    <div id="modalDesc" class="modal-desc">Are you sure you want to proceed?</div>
    <div class="modal-actions">
      <button type="button" class="btn btn-secondary" onclick="closeActionModal()">Cancel</button>
      <button type="button" id="modalConfirmBtn" class="btn btn-danger" onclick="executeModalAction()">Confirm</button>
    </div>
  </div>
</div>

<div id="toast" class="toast">Configuration updated</div>

<script>
let tempHistory = [];
let humHistory = [];
let presHistory = [];
const MAX_HIST = 36;
let pendingModalAction = '';

const COND_NAMES = ['Above High', 'Below Low', 'Outside Range', 'Inside Range'];
const FAILSAFE_NAMES = ['FORCE OFF', 'HOLD LAST', 'FORCE ON'];

function switchTab(tabId) {
  document.querySelectorAll('.tab-btn').forEach(b => b.classList.remove('active'));
  document.querySelectorAll('.tab-content').forEach(c => c.classList.remove('active'));

  const btn = document.getElementById('tabBtn-' + tabId);
  if (btn) btn.classList.add('active');

  const pane = document.getElementById('tab-' + tabId);
  if (pane) pane.classList.add('active');

  if (tabId === 'settings' || tabId === 'wifi') {
    loadSettings();
  }
}

function showToast(msg, isErr) {
  const t = document.getElementById('toast');
  t.innerText = msg;
  t.style.borderColor = isErr ? 'var(--accent-red)' : 'var(--accent-cyan)';
  t.style.display = 'block';
  clearTimeout(window._toastTimer);
  window._toastTimer = setTimeout(() => { t.style.display = 'none'; }, 3200);
}

function togglePassVisibility() {
  const inp = document.getElementById('wifiPass');
  const btn = event.currentTarget;
  if (inp.type === 'password') {
    inp.type = 'text';
    btn.innerText = 'HIDE';
  } else {
    inp.type = 'password';
    btn.innerText = 'SHOW';
  }
}

function formatUptime(ms) {
  let totalSec = Math.floor(ms / 1000);
  let days = Math.floor(totalSec / 86400);
  totalSec %= 86400;
  let hrs = Math.floor(totalSec / 3600);
  totalSec %= 3600;
  let mins = Math.floor(totalSec / 60);
  let secs = totalSec % 60;
  return days + 'd ' + String(hrs).padStart(2,'0') + ':' + String(mins).padStart(2,'0') + ':' + String(secs).padStart(2,'0');
}

function drawSparkline(canvasId, data, strokeColor) {
  const canvas = document.getElementById(canvasId);
  if (!canvas) return;
  const rect = canvas.getBoundingClientRect();
  if (rect.width === 0 || rect.height === 0) return;

  const dpr = window.devicePixelRatio || 1;
  canvas.width = rect.width * dpr;
  canvas.height = rect.height * dpr;

  const ctx = canvas.getContext('2d');
  ctx.scale(dpr, dpr);
  ctx.clearRect(0, 0, rect.width, rect.height);

  if (data.length < 2) return;

  let min = Math.min(...data);
  let max = Math.max(...data);
  if (Math.abs(max - min) < 0.2) { max += 0.5; min -= 0.5; }

  ctx.beginPath();
  ctx.strokeStyle = strokeColor;
  ctx.lineWidth = 2;
  ctx.lineJoin = 'round';

  const step = rect.width / (MAX_HIST - 1);
  for (let i = 0; i < data.length; i++) {
    let x = i * step;
    let norm = (data[i] - min) / (max - min);
    let y = rect.height - (norm * (rect.height - 10) + 5);
    if (i === 0) ctx.moveTo(x, y);
    else ctx.lineTo(x, y);
  }
  ctx.stroke();
}

function updateChannelUI(idx, ch) {
  const badge = document.getElementById('chBadge' + idx);
  const fill = document.getElementById('progFill' + idx);
  document.getElementById('lblLow' + idx).innerText = ch.low.toFixed(1);
  document.getElementById('lblHigh' + idx).innerText = ch.high.toFixed(1);
  document.getElementById('lblCond' + idx).innerText = COND_NAMES[ch.cond] || 'Above High';
  document.getElementById('valOff' + idx).innerText = (ch.off >= 0 ? '+' : '') + ch.off.toFixed(2);

  if (!ch.enabled) {
    badge.className = 'badge badge-off';
    badge.innerText = 'DISABLED';
    fill.className = 'progress-fill';
  } else if (ch.latched) {
    badge.className = 'badge badge-trig';
    badge.innerText = 'TRIGGERED';
    fill.className = 'progress-fill trig';
  } else {
    badge.className = 'badge badge-ok';
    badge.innerText = 'NORMAL';
    fill.className = 'progress-fill';
  }

  let span = ch.high - ch.low;
  let pct = 50;
  if (span > 0.0001) {
    pct = ((ch.val - ch.low) / span) * 100.0;
  }
  pct = Math.max(0, Math.min(100, pct));
  fill.style.width = pct.toFixed(1) + '%';
}

function pollStatus() {
  fetch('/api/status')
    .then(r => r.json())
    .then(d => {
      const dot = document.getElementById('statusDot');
      const txt = document.getElementById('statusText');
      const sBadge = document.getElementById('sensorHealthBadge');

      if (d.sensorHealthy && d.hasValidSample) {
        dot.className = 'dot online';
        txt.innerText = 'ONLINE';
        sBadge.className = 'badge badge-ok';
        sBadge.innerText = 'BME280 OK (' + d.activeAddr + ')';

        document.getElementById('valTempC').innerText = d.tempC.toFixed(2);
        document.getElementById('valTempF').innerText = d.tempF.toFixed(2);
        document.getElementById('valHum').innerText = d.humPct.toFixed(2);
        document.getElementById('valDewC').innerText = d.dewPointC.toFixed(2);
        document.getElementById('valDewF').innerText = d.dewPointF.toFixed(2);
        document.getElementById('valPres').innerText = d.presHpa.toFixed(2);
        document.getElementById('valAlt').innerText = d.altitudeM.toFixed(1);

        tempHistory.push(d.tempC); if (tempHistory.length > MAX_HIST) tempHistory.shift();
        humHistory.push(d.humPct); if (humHistory.length > MAX_HIST) humHistory.shift();
        presHistory.push(d.presHpa); if (presHistory.length > MAX_HIST) presHistory.shift();

        drawSparkline('sparkTemp', tempHistory, '#00f2fe');
        drawSparkline('sparkHum', humHistory, '#00e676');
        drawSparkline('sparkPres', presHistory, '#ffab00');
      } else {
        dot.className = 'dot warn';
        txt.innerText = d.sensorHealthy ? 'WARMING UP' : 'SENSOR FAULT';
        sBadge.className = 'badge badge-trig';
        sBadge.innerText = d.sensorHealthy ? 'WARMUP GUARD' : 'I2C FAULT - RETRYING';
      }

      for (let i = 0; i < 3; i++) {
        if (d.channels && d.channels[i]) updateChannelUI(i, d.channels[i]);
      }

      const modeBadge = document.getElementById('valRelayModeBadge');
      if (d.relayMode === 0) {
        modeBadge.innerText = 'UNIFIED MODE (GPIO 14 / D5)';
        document.getElementById('relayTitle0').innerText = 'Unified Relay (All)';
        document.getElementById('relayTitle1').innerText = 'Humidity (Standby)';
        document.getElementById('relayTitle2').innerText = 'Pressure (Standby)';
      } else {
        modeBadge.innerText = 'INDIVIDUAL (D5 / D6 / D7)';
        document.getElementById('relayTitle0').innerText = 'Temp Relay';
        document.getElementById('relayTitle1').innerText = 'Humidity Relay';
        document.getElementById('relayTitle2').innerText = 'Pressure Relay';
      }

      document.getElementById('valRelayPolarity').innerText = (d.relayPolarity === 0) ? 'ACTIVE HIGH' : 'ACTIVE LOW';
      document.getElementById('valFailsafe').innerText = FAILSAFE_NAMES[d.failsafeMode] || 'FORCE OFF';
      document.getElementById('valHoldTime').innerText = d.minRelaySwitchMs + ' ms';

      for (let i = 0; i < 3; i++) {
        let rBox = document.getElementById('relayBox' + i);
        let rState = document.getElementById('relayState' + i);
        let rCnt = document.getElementById('relayCnt' + i);
        let rWait = document.getElementById('relayWait' + i);
        let r = d.relays[i];

        if (r.energized === 1) {
          rBox.classList.add('active');
          rState.className = 'relay-state ON';
          rState.innerText = 'ENERGIZED';
        } else {
          rBox.classList.remove('active');
          rState.className = 'relay-state OFF';
          rState.innerText = 'DE-ENERGIZED';
        }
        rCnt.innerText = r.switches;
        rWait.innerText = r.cooldown ? '(Hold Guard)' : '';
      }

      document.getElementById('sysI2CAddr').innerText = d.sensorHealthy ? ('Connected at ' + d.activeAddr + ' (SDA: D1 / SCL: D2)') : 'Fault (Auto-Recovering)';
      document.getElementById('sysHeap').innerText = Math.round(d.heapFree / 1024) + ' KB Free (' + d.heapFrag + '% Frag)';
      document.getElementById('sysNVS').innerText = 'Seq #' + d.nvsSeq + ' (' + d.nvsWrites + ' writes)' + (d.nvsDirty ? ' [Pending]' : ' [Synced]');
      document.getElementById('sysBtn').innerText = d.btnHeld ? ('HELD: ' + d.btnHoldMs + ' / 5000 ms (GPIO0)') : 'Released (GPIO0 / D3)';
      document.getElementById('sysClients').innerText = d.apSSID + ' (Ch ' + d.apChannel + ') - ' + d.wifiClients + ' client(s)';
      document.getElementById('sysUptime').innerText = formatUptime(d.uptimeMs);
    })
    .catch(() => {
      document.getElementById('statusDot').className = 'dot offline';
      document.getElementById('statusText').innerText = 'OFFLINE';
    });
}

function loadSettings() {
  fetch('/api/settings')
    .then(r => r.json())
    .then(d => {
      document.getElementById('cfgRelayMode').value = d.relayMode;
      document.getElementById('cfgRelayPolarity').value = d.relayPolarity;
      document.getElementById('cfgFailsafeMode').value = d.failsafeMode;
      document.getElementById('cfgMinSwitchMs').value = d.minRelaySwitchMs;
      document.getElementById('cfgPollMs').value = d.sensorPollMs;

      for (let i = 0; i < 3; i++) {
        let ch = d.channels[i];
        document.getElementById('ch' + i + '_enabled').value = ch.enabled;
        document.getElementById('ch' + i + '_cond').value = ch.cond;
        document.getElementById('ch' + i + '_low').value = ch.low.toFixed(2);
        document.getElementById('ch' + i + '_high').value = ch.high.toFixed(2);
        document.getElementById('ch' + i + '_hyst').value = ch.hyst.toFixed(2);
        document.getElementById('ch' + i + '_off').value = ch.off.toFixed(2);
      }

      document.getElementById('wifiSSID').value = d.apSSID;
      document.getElementById('wifiPass').value = d.apPassword;
      document.getElementById('wifiChan').value = d.apChannel;
      document.getElementById('wifiMax').value = d.apMaxClients;
      document.getElementById('wifiHide').value = d.apHideSSID;
    });
}

function saveSettings(e) {
  e.preventDefault();
  let channels = [];
  for (let i = 0; i < 3; i++) {
    let lowVal = parseFloat(document.getElementById('ch' + i + '_low').value);
    let highVal = parseFloat(document.getElementById('ch' + i + '_high').value);
    if (lowVal > highVal) {
      showToast('Channel ' + i + ' Low Threshold cannot exceed High Threshold', true);
      return;
    }
    channels.push({
      enabled: parseInt(document.getElementById('ch' + i + '_enabled').value, 10),
      cond: parseInt(document.getElementById('ch' + i + '_cond').value, 10),
      low: lowVal,
      high: highVal,
      hyst: parseFloat(document.getElementById('ch' + i + '_hyst').value),
      off: parseFloat(document.getElementById('ch' + i + '_off').value)
    });
  }

  let payload = {
    relayMode: parseInt(document.getElementById('cfgRelayMode').value, 10),
    relayPolarity: parseInt(document.getElementById('cfgRelayPolarity').value, 10),
    failsafeMode: parseInt(document.getElementById('cfgFailsafeMode').value, 10),
    minRelaySwitchMs: parseInt(document.getElementById('cfgMinSwitchMs').value, 10),
    sensorPollMs: parseInt(document.getElementById('cfgPollMs').value, 10),
    channels: channels
  };

  fetch('/api/settings', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload)
  })
  .then(r => r.json())
  .then(d => {
    if (d.status === 'ok') {
      showToast('Sensor & Relay settings applied');
      pollStatus();
    } else {
      showToast('Failed to save settings', true);
    }
  })
  .catch(() => showToast('Network error saving settings', true));
}

function saveWiFi(e) {
  e.preventDefault();
  let ssid = document.getElementById('wifiSSID').value.trim();
  let pass = document.getElementById('wifiPass').value;
  if (ssid.length < 1 || ssid.length > 32) {
    showToast('SSID must be 1 to 32 characters', true);
    return;
  }
  if (pass.length < 8 || pass.length > 63) {
    showToast('Password must be 8 to 63 characters', true);
    return;
  }

  let payload = {
    apSSID: ssid,
    apPassword: pass,
    apChannel: parseInt(document.getElementById('wifiChan').value, 10),
    apMaxClients: parseInt(document.getElementById('wifiMax').value, 10),
    apHideSSID: parseInt(document.getElementById('wifiHide').value, 10)
  };

  fetch('/api/wifi', {
    method: 'POST',
    headers: {'Content-Type': 'application/json'},
    body: JSON.stringify(payload)
  })
  .then(r => r.json())
  .then(d => {
    if (d.status === 'ok') {
      showToast('Wi-Fi AP updated. Reconnect to ' + ssid);
    } else {
      showToast('Invalid Wi-Fi parameters', true);
    }
  })
  .catch(() => showToast('Network error updating Wi-Fi', true));
}

function openActionModal(actionType) {
  pendingModalAction = actionType;
  const modal = document.getElementById('actionModal');
  const title = document.getElementById('modalTitle');
  const desc = document.getElementById('modalDesc');
  const btn = document.getElementById('modalConfirmBtn');

  if (actionType === 'reboot') {
    title.innerText = 'Reboot ESP8266 Controller';
    desc.innerText = 'All pending settings will be committed to EEPROM flash before restarting the processor.';
    btn.className = 'btn btn-warning';
    btn.innerText = 'Reboot Now';
  } else {
    title.innerText = 'Factory Reset Controller';
    desc.innerText = 'This will de-energize all relays and restore all thresholds and Wi-Fi AP settings (ESP8266_BME280 / 12345678) to defaults.';
    btn.className = 'btn btn-danger';
    btn.innerText = 'Factory Reset';
  }
  modal.style.display = 'flex';
}

function closeActionModal() {
  pendingModalAction = '';
  document.getElementById('actionModal').style.display = 'none';
}

function executeModalAction() {
  const act = pendingModalAction;
  closeActionModal();
  if (act === 'reboot') {
    fetch('/api/reboot', {method: 'POST'})
      .then(r => r.json())
      .then(() => showToast('Controller rebooting...'));
  } else if (act === 'reset') {
    fetch('/api/reset', {method: 'POST'})
      .then(r => r.json())
      .then(() => showToast('Factory reset complete. Rebooting...'));
  }
}

setInterval(pollStatus, 1000);
pollStatus();
</script>
</body>
</html>
)rawliteral";

// =====================================================================================
// ARDUINO SETUP & MAIN NON-BLOCKING EXECUTION LOOP
// =====================================================================================
void setup(void) {
    memset(&g_state, 0, sizeof(RuntimeState));
    g_state.bootTimeMs = getMillis64();

    initStorageAndConfig();

    for (uint64_t i = 0ULL; i < 3ULL; ++i) {
        g_state.relays[i].isEnergized = 0ULL;
        writeRelayHardwarePins(i, 0ULL);
    }

    pinMode((uint8_t)PIN_RELAY_TEMP_UNIF, OUTPUT);
    pinMode((uint8_t)PIN_RELAY_HUMIDITY, OUTPUT);
    pinMode((uint8_t)PIN_RELAY_PRESSURE, OUTPUT);
    pinMode((uint8_t)PIN_FACTORY_RESET_BTN, INPUT_PULLUP);

    for (uint64_t i = 0ULL; i < 3ULL; ++i) {
        writeRelayHardwarePins(i, 0ULL);
    }

    tryInitializeBME280();
    initializeAccessPoint();
    registerWebRoutes();

    ESP.wdtEnable(8000);
}

void loop(void) {
    ESP.wdtFeed();

    g_dnsServer.processNextRequest();
    g_webServer.handleClient();

    pollSensorNonBlocking();
    processNvsWriteCoalescing();
    processFactoryResetButtonNonBlocking();
    reloadWiFiSettingsNonBlocking();

    if (g_state.rebootPending) {
        if (getMillis64() >= g_state.rebootTargetMs) {
            g_state.rebootPending = 0ULL;
            ESP.restart();
        }
    }
}
