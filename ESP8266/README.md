## Hardware
- **BME280**: SDA → GPIO5 (D1), SCL → GPIO4 (D2), VCC → 3.3V, GND → GND
- **Relay**: signal → GPIO12 (D6), plus VCC/GND per module
- **Boot/Reset button**: built-in FLASH button on GPIO0 (hold 5 s → factory reset)

## First Use
1. Power the board (USB or 3.3V supply).
2. On phone/PC, connect to Wi-Fi:
   - **SSID:** `ESP8266_BME280`
   - **Password:** `12345678`
3. A captive-portal page should open automatically. If not, browse to **http://192.168.4.1**

## Dashboard
- **Live Readings** — temperature, humidity, pressure, relay state (updates every 2 s)
- **Thresholds** — set high/low limits, then tap **Save Settings**
- **Relay Mode**:
  - `Off` — relay always off
  - `Auto` — relay ON when any reading is above its high limit or below its low limit
  - `On` — relay always on
- **System** — uptime, free heap, BME280 status

## Factory Reset
- **From UI:** tap **Factory Reset** → confirm
- **From hardware:** hold the FLASH button ~5 s

Defaults: Temp 10–30 °C, Humidity 30–70 %, Pressure 980–1020 hPa, Relay = Auto.

## Notes
- Values are stored in ×100 units internally (e.g. 30.00 °C = 3000); the UI handles this.
- Config is saved to EEPROM and survives reboot.
- If **BME280** shows `FAIL`, check wiring/address (0x76 or 0x77). In Auto mode with no sensor, relay will turn **ON** — set mode to **Off** if this is undesirable.
- Only AP mode is used; no station (router) connection.
