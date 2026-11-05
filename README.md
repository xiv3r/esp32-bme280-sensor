## Components

- **ESP32 DevKit** (30-pin or 38-pin, any ESP32-WROOM / WROVER board)
- **BME280 breakout** (I²C version — the one with 4 pins: VIN, GND, SCL, SDA)
- **BOOT button** — already on the board (GPIO0, no wiring needed)
- **LED** — already on the board (GPIO2, no wiring needed)
- **4 jumper wires**

---

## Wiring Table

| BME280 pin | ESP32 pin | Notes |
|------------|-----------|-------|
| **VIN / VCC** | **3V3** | ⚠️ Use **3.3 V**, not 5 V. BME280 is 3.3 V only. |
| **GND** | **GND** | Any GND pin works. |
| **SCL** | **GPIO 22** | I²C clock (`PIN_I2C_SCL` in firmware). |
| **SDA** | **GPIO 21** | I²C data (`PIN_I2C_SDA` in firmware). |

---

## Notes

### Power
- The BME280 module runs on **3.3 V**. Some breakout boards have a 5 V-tolerant regulator + level shifter, but the bare chip is 3.3 V only — **always use 3V3** to be safe.
- The ESP32's onboard 3.3 V regulator easily handles the BME280's ~0.7 mA.

### I²C address
- The firmware tries **0x76** first, then **0x77**.
- Most modules ship at **0x76** (SDO pin pulled low).
- If your module has an SDO pin, leave it floating or tie to GND for 0x76, tie to 3V3 for 0x77. The firmware auto-detects either.

### Pull-ups
- Most BME280 breakouts have **onboard 4.7 kΩ pull-ups** on SCL and SDA. **No external resistors needed.**
- If you're using a bare BME280 chip (not a breakout), add 4.7 kΩ from SDA→3V3 and SCL→3V3.

### No external components needed
- **BOOT button** (GPIO0) is already on the board — hold 5 s to factory reset (per `FACTORY_HOLD_MS`).
- **Status LED** (GPIO2) is already on the board — blinks once per second.
- **No buttons, no LEDs, no resistors** to wire externally.

### Cable length
- Keep I²C wires **short** (< 30 cm) to avoid noise. For longer runs, lower the clock — the firmware already uses 100 kHz (`Wire.setClock(100000)`).

---

## Quick Test After Wiring

1. Connect USB → flash firmware.
2. The board creates a WiFi AP named **`ESP32-Config`** with password **`12345678`** (defaults).
3. Connect your phone/PC to that AP.
4. Open **http://192.168.4.1** → log in with **`admin` / `esp32admin`**.
5. Dashboard should show **"Sensor OK at 0x76"** (or 0x77) and live temperature / humidity / pressure.

If it says **"Sensor not detected"**, check:
- VIN is on **3V3**, not 5 V or VIN.
- SDA is on **GPIO21**, SCL on **GPIO22** (not swapped).
- Wires are firmly seated.

---

## Bill of Materials

| Item | Qty |
|------|-----|
| ESP32 DevKit | 1 |
| BME280 I²C breakout | 1 |
| Jumper wires (female–female) | 4 |
| USB cable | 1 |

**Total external wiring: 4 wires.**
