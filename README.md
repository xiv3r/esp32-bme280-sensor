<img src="https://github.com/xiv3r/esp32-bme280-sensor/blob/main/esp32-bme280.png">

## 1. What you need

**Hardware**
- ESP32 dev board
- BME280 sensor (I2C)
- 1-channel relay module (with driver — most modules have one)
- Jumper wires
- 10 kΩ resistor (important, see step 6)

**Software**
- Arduino IDE (or PlatformIO)
- ESP32 board package installed
- Libraries: `Adafruit BME280 Library`, `Adafruit Unified Sensor`

---

## 2. Wiring

### BME280 → ESP32

| BME280 | ESP32 |
|---|---|
| VIN / VCC | 3V3 |
| GND | GND |
| SDA | GPIO 21 |
| SCL | GPIO 22 |

### Relay module → ESP32

| Relay module | ESP32 |
|---|---|
| VCC | 5V (or 3V3 — check your module) |
| GND | GND |
| IN | **GPIO 25** |

**Important:** add a **10 kΩ resistor from GPIO 25 to GND**. This keeps the relay OFF during boot/reset. Without it, the relay may click on for a moment when the ESP32 powers up.

---

## 3. Flash the firmware

1. Open `v3-esp32-gpio25.txt` in Arduino IDE (save it as `.ino` first if needed).
2. Select your board: **Tools → Board → ESP32 Dev Module**.
3. Select the correct COM port.
4. Click **Upload**.

> ⚠️ **First boot after upgrade from v2:** WiFi settings are reset to defaults. You will need to reconfigure WiFi. This is normal.

---

## 4. First-time setup

1. Power the ESP32.
2. On your phone or laptop, connect to WiFi **`ESP32-BME280`** (password **`12345678`**).
3. A captive-portal page should open. If not, open a browser and go to **`http://192.168.4.1`**.
4. Log in with:
   - **Username:** `admin`
   - **Password:** `12345678`
5. Go to the **WiFi STA** tab → enter your home WiFi name and password → **Apply STA** → **Reboot**.

Now the ESP32 is on your home network. You can reach it at its new IP address, or reconnect to `ESP32-BME280` if you prefer.

---

## 5. Configure the thresholds

1. Open the **Thresholds** tab.
2. Set min/max for **Temperature**, **Humidity**, **Pressure**.
3. Click **Save Thresholds**.
4. The dashboard cards turn **yellow** when a value is out of range.

---

## 6. Configure the relay

Still on the **Thresholds** tab, scroll to **Relay (GPIO 25)**:

| Option | What it does |
|---|---|
| **Relay enabled** | Turn the relay feature on/off |
| **Logic** | **OR** = relay ON if *any* checked metric is out of range. **AND** = relay ON only if *all* checked metrics are out of range |
| **React to temperature / humidity / pressure** | Pick which metrics drive the relay |
| **Active-low relay** | Set to **Yes** if your relay module turns ON when the pin is LOW (most cheap modules do). Set to **No** if it turns ON when the pin is HIGH |

Click **Save Relay Settings**.

> **Tip:** If you don't know whether your relay is active-low or active-high, enable it with temperature selected and set a threshold you can easily cross (e.g. temp max = 25 °C). Touch the BME280 with your finger — if the relay clicks ON when the dashboard shows an alert, your invert setting is correct.

---

## 7. How the relay behaves

- Turns **ON** the moment a selected sensor value crosses a threshold (matches the dashboard alert exactly).
- Turns **OFF** only when the value comes back **inside** the range by a small safety margin (0.5 °C / 1 %RH / 0.5 hPa). This prevents clicking on/off when the value hovers right at the threshold.
- If the sensor is unplugged, the relay **holds its last state** (it does not toggle randomly).
- The relay is **independent of the "Alerts" enabled switch**. You can have the relay working with dashboard warnings turned off.

---

## 8. Common tasks

### Change the admin password
**System** tab → **Change Admin Password** → enter new password (min 8 characters) → **Change Password** → reload the page and log in with the new password.

### Reboot the ESP32
**System** tab → **Reboot** button.

### Factory reset (erase everything)
Two ways:
- **System** tab → **Factory Reset** → type `RESET` → confirm.
- Or hold the **BOOT button** on the ESP32 for **5 seconds**.

After factory reset, the ESP32 returns to `ESP32-BME280` / `12345678` and admin `admin` / `12345678`.

---

## 9. Quick troubleshooting

| Problem | Fix |
|---|---|
| Can't see `ESP32-BME280` WiFi | Wait 30 s after power on, or check USB power is enough |
| Web page doesn't load | Try `http://192.168.4.1` directly. Make sure you're connected to the ESP32's WiFi |
| Dashboard shows "Sensor not detected" | Check BME280 wiring: SDA→21, SCL→22, VCC→3V3, GND→GND |
| Relay clicks ON at boot then OFF | Normal — add the 10 kΩ pull-down resistor (step 2) |
| Relay never turns ON | Check **Relay enabled = Yes**, at least one metric selected, and the invert setting matches your module |
| Relay ON when value looks OK | You have the invert setting wrong — try the opposite |
| Relay flickering at threshold | Increase the gap between min and max thresholds slightly |
| Forgot admin password | Factory reset (step 8), then log in with default `admin` / `12345678` |

---

## 10. Safety note

GPIO 25 is **3.3 V logic**. If your relay module expects 5 V on the IN pin, confirm it accepts 3.3 V — most modules with an opto-isolator do. If yours does not, add a level shifter.

**Never** connect the relay coil directly to GPIO 25. Use a relay **module** (which has a transistor driver), never a bare relay coil.
