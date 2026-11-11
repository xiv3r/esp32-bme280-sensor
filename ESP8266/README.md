# Deployment Instructions

## 1. Hardware wiring

| BME280 | NodeMCU | Notes |
|---|---|---|
| VIN / VCC | 3V3 | **Not 5V** — BME280 is 3.3 V only |
| GND | GND | |
| SDA | **D1** (GPIO5) | |
| SCL | **D2** (GPIO4) | |

| Relay module | NodeMCU | Notes |
|---|---|---|
| IN | **D12** (GPIO12) | 3.3 V logic; use a module with an opto-isolator or transistor driver |
| VCC | 5V (or 3V3 per module) | |
| GND | GND | |

**GPIO12 strapping:** ensure the relay's IN pin is **LOW at power-on** (add a 10 kΩ pull-down from D12 to GND if the module pulls high at boot). Otherwise the ESP8266 may fail to enter flash-boot mode.

**BOOT button (GPIO0):** the onboard FLASH button on the NodeMCU. No wiring needed.

## 2. Software

1. Install **Arduino IDE** with the **ESP8266 board package** (Board Manager → `esp8266` by ESP8266 Community, version 3.x).
2. Install these libraries via Library Manager:
   - **Adafruit BME280 Library**
   - **Adafruit Unified Sensor**
3. Board settings: **NodeMCU 1.0 (ESP-12E Module)**, Flash Size **4MB**, CPU **80 MHz**, Upload Speed **115200**.
4. Paste the code, compile, upload.

## 3. First boot

1. On a phone or PC, scan WiFi and connect to:
   - **SSID:** `ESP8266_BME280`
   - **Password:** `12345678`
2. The captive portal should open automatically. If not, browse to **http://192.168.4.1/**.
3. Dashboard shows live temperature, humidity, pressure, relay state, uptime, heap, RSSI.

## 4. Using the pages

- **Dashboard** — live readings; **Force ON / Force OFF / Auto** buttons control the relay. In Auto mode, the relay turns ON whenever any reading is outside the configured thresholds; it turns OFF when all readings are back inside range, or on sensor failure.
- **Thresholds** — set Low/High for each of temperature, humidity, and pressure. **Low must be less than High.** Settings are saved to EEPROM after a 60-second debounce.
- **WiFi** — connect the ESP8266 to your home router (optional): enter SSID and password, tap **Connect**. The AP stays up at `192.168.4.1` either way. Tap **Disconnect** to leave station mode.

## 5. Factory reset

Either method restores defaults and erases stored thresholds:

- **Hold the onboard FLASH button for 5 seconds** until the board reboots.
- Or open **Thresholds → Factory Reset**.

The AP credentials (`ESP8266_BME280` / `12345678`) do not change on reset.

## 6. Normal operation

- Sensor updates once per second.
- If three consecutive reads fail, the sensor enters a 30-second cooldown; the dashboard shows **COOLDOWN** instead of **OK**. Readings resume automatically.
- In Auto mode, the relay stays **OFF** during cooldown and on sensor failure (fail-safe).
- Settings changes are written to flash at most once per minute to protect NVS endurance.

## 7. Troubleshooting

| Symptom | Fix |
|---|---|
| Boot loop / won't flash | Add pull-down on D12; disconnect relay during upload |
| Sensor shows ERROR | Check D1/D2 wiring, 3.3 V supply, and that the module is a BME280 (not BMP280 — no humidity) |
| Captive portal doesn't pop up | Browse to `192.168.4.1` manually |
| Relay triggers immediately in Auto | Verify threshold values in **Thresholds** page and that pressure thresholds are in **hPa** (e.g., 950–1050, not 95000) |
| Dashboard shows `--` for all readings | Sensor cooldown active (wait 30 s) or BME280 not detected |
