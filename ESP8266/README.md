## What it does
A local-only Wi-Fi access point that reads a BME280 sensor and controls up to three relays based on configurable temperature, humidity, and pressure thresholds. No cloud connection.

## Hardware wiring
| Signal | ESP8266 pin | GPIO |
|---|---|---|
| BME280 SDA | D1 | 5 |
| BME280 SCL | D2 | 4 |
| Relay 1 (temp/unified) | D5 | 14 |
| Relay 2 (humidity) | D6 | 12 |
| Relay 3 (pressure) | D7 | 13 |
| Factory reset button | D3 | 0 (to GND) |

- Power the BME280 from **3.3 V** (not 5 V).
- Default relay polarity is **active LOW** (typical relay modules). Change it in the UI if your board is active HIGH.
- Do **not** hold the D3/GPIO0 button while powering on — it must be HIGH at boot for normal startup.

## First-time use
1. Flash the sketch and open the Serial Monitor at 115200 baud (optional).
2. On your phone/PC, connect to Wi-Fi network **ESP8266_BME280** with password **12345678**.
3. Open **http://192.168.4.1** in a browser.
4. Go to **Wi-Fi access point** and change the default password immediately.

## Pages
- **Overview** — live readings, relay state, device health, and a "Re-scan sensor" button.
- **Sensor & relay** — set relay mode (unified vs. individual), polarity, fault behavior, sampling interval (1–60 s), and per-sensor alarm bands with hysteresis.
- **Wi-Fi access point** — change SSID, channel (1–13), and WPA2 password. Saving restarts the AP.

## Relay modes
- **Unified** (default): any enabled alarm drives D5 only; D6/D7 stay off.
- **Individual**: temperature → D5, humidity → D6, pressure → D7.

## Factory reset
While the device is running, hold the D3/GPIO0 button for **5 seconds**. Defaults are restored and the AP restarts.

## Troubleshooting
- **Sensor not detected**: check 3.3 V power, SDA/SCL wiring, and that the module is at address 0x76 or 0x77. Use "Re-scan sensor" after fixing.
- **Relays behave inverted**: switch "Relay input polarity" in Sensor & relay settings.
- **Settings won't save**: LittleFS may be unavailable (wrong flash partition scheme). Select a partition scheme with SPIFFS/LittleFS in the Arduino IDE.
- **Captive portal doesn't open automatically**: some HTTPS pages can't be redirected; navigate directly to http://192.168.4.1.

## Safety note
This is **not** a safety-rated fail-safe controller. Choose the "when BME280 data is unavailable" behavior based on your external circuit, and use a proper relay driver board — do not drive relay coils directly from GPIOs.
