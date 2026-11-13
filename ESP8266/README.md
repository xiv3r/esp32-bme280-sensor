## 1. Arduino IDE Setup

**Board Manager:**
- Install **ESP8266** core: `File → Preferences → Additional Boards Manager URLs` → add:
  ```
  http://arduino.esp8266.com/stable/package_esp8266com_index.json
  ```
- `Tools → Board → Boards Manager` → search `esp8266` → install **ESP8266 by ESP8266 Community** (v3.x)

**Libraries** (`Tools → Manage Libraries`):
- **Adafruit BME280 Library** (by Adafruit)
- **Adafruit Unified Sensor** (dependency — IDE will prompt, install it)

## 2. Board Settings (`Tools` menu)

| Setting | Value |
|---|---|
| Board | **NodeMCU 1.0 (ESP-12E Module)** (or your board) |
| Flash Size | **4MB (FS:none OTA:~1019KB)** — required for OTA |
| CPU Frequency | 80 MHz |
| Upload Speed | 115200 |
| Port | your COM/tty port |

> OTA needs ≥1 MB free for the update partition. `4MB (FS:none OTA:~1019KB)` is the standard choice.

## 3. Wiring

| BME280 | NodeMCU |
|---|---|
| VCC | 3V3 |
| GND | GND |
| SDA | **D2** (GPIO4) |
| SCL | **D1** (GPIO5) |
| SDO | GND → `0x76`, or 3V3 → `0x77` |

| Relay | NodeMCU |
|---|---|
| IN | **D6** (GPIO12) |
| VCC | 3V3 or 5V (per module) |
| GND | GND |

> **BOOT button (GPIO0)** is on-board — no wiring needed. Hold 5 s to factory reset.

## 4. First Flash (USB)

1. Paste the full code into a new `.ino` sketch (name it `ESP8266_BME280.ino`).
2. `Sketch → Verify` — expect **no errors**.
3. Connect board via USB, select the correct **Port**.
4. Click **Upload**. If it fails with `espcomm_upload_mem failed`, hold **BOOT** while the upload starts, release after "Connecting...".
5. Open **Serial Monitor @ 115200** — after boot you'll see the AP come up.

## 5. First Connection

1. On phone/PC, scan WiFi → join **`ESP8266_BME280`** / password **`12345678`**.
2. A captive-portal page opens automatically. If not, browse to **`http://192.168.4.1`**.
3. You should see the dashboard with live temp/hum/pressure.

## 6. Configuration

**`/settings`** — relay mode, thresholds, sample interval, I2C address.
- Changes persist **immediately** to flash.
- Changing I2C address re-initializes the sensor automatically.

**`/wifi`** — AP SSID, password (min 8 chars or empty = open), channel, hidden, max clients.
- Saving **reboots the device**. Reconnect with the new SSID/password.

## 7. Factory Reset

- **Button:** Hold on-board **BOOT** for 5 s → device resets to defaults and reboots.
- **Web:** `/settings` → **Factory Reset** button.

Defaults: SSID `ESP8266_BME280`, pass `12345678`, mode Manual, relay ON.

## 8. OTA Firmware Update

Once connected to the AP:

1. Browse to **`http://192.168.4.1/update`**
2. Select a compiled `.bin` file (`Sketch → Export Compiled Binary` creates it).
3. Click **Update** → device reboots into new firmware.

> OTA requires the **4MB OTA** flash layout from step 2.

## 9. Verifying It Works

- **Dashboard** (`/`) refreshes every 2 s.
- **Sensor: OK** in status = sensor responding and data valid.
- **Sensor: FAIL** = sensor not found or unplugged (relay modes 1–4 will hold relay OFF after ~15 s).
- **`/api/status`** returns JSON — useful for `curl`/Home Assistant:
  ```bash
  curl http://192.168.4.1/api/status
  ```

## 10. Common Issues

| Symptom | Fix |
|---|---|
| Can't upload | Hold **BOOT** during "Connecting..." |
| Sensor reads `--` | Check wiring; try I2C address `0x77` in `/settings` |
| Sensor: FAIL forever | BME280 not powered (3.3V only) or SDA/SCL swapped |
| Relay never clicks | Ensure mode 0 (Manual) or thresholds crossed; relay is **active HIGH** |
| Can't reach `192.168.4.1` | Reconnect WiFi; DNS portal may need manual IP |
| Settings reset after power cycle | Verify flash layout is OTA 4MB, then re-save once |
| OTA fails | Confirm flash layout allows OTA (see step 2) |

## 11. Quick Reference

| URL | Purpose |
|---|---|
| `/` | Dashboard |
| `/settings` | Sensor/relay config |
| `/wifi` | AP config (reboots on save) |
| `/update` | OTA firmware upload |
| `/api/status` | JSON status |
| `/api/settings` POST | Programmatic settings |
| `/api/wifi` POST | Programmatic AP config |
| `/api/factoryreset` POST | Factory reset |
