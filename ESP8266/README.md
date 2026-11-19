## 1. Hardware Wiring

| Device / Module | Pin Name | ESP8266 GPIO | Board Pin (NodeMCU / Wemos D1) | Notes |
| :--- | :--- | :--- | :--- | :--- |
| **BME280 Sensor** | **SDA** | **GPIO 5** | **D1** | I2C Data line |
| **BME280 Sensor** | **SCL** | **GPIO 4** | **D2** | I2C Clock line |
| **BME280 Sensor** | **VCC / GND** | — | **3.3V / GND** | Sensor power ($3.3\text{V}$ only) |
| **Relay 1** | **IN1** | **GPIO 14** | **D5** | **Unified Mode** (all sensors) or **Temp** |
| **Relay 2** | **IN2** | **GPIO 12** | **D6** | **Individual Mode** (Humidity) |
| **Relay 3** | **IN3** | **GPIO 13** | **D7** | **Individual Mode** (Pressure) |
| **Reset Button** | **BOOT / FLASH**| **GPIO 0** | **D3** | Onboard button (Hold $\ge 5\text{s}$ for factory reset) |

---

## 2. Arduino IDE Setup & Libraries

1. **Install ESP8266 Board Package:**
   - In Arduino IDE, go to **Settings** $\rightarrow$ **Additional Board Manager URLs**:  
     `https://arduino.esp8266.com/stable/package_esp8266com_index.json`
   - Open **Tools** $\rightarrow$ **Board Manager**, search for **`esp8266`**, and install.
2. **Install Required Libraries** (via **Tools** $\rightarrow$ **Manage Libraries...**):
   - **`Adafruit BME280 Library`**
   - **`Adafruit Unified Sensor`**
   - **`ArduinoJson`** (v6 or v7)
3. **Board Configuration:**
   - **Board:** *NodeMCU 1.0 (ESP-12E Module)* or *LOLIN(WEMOS) D1 R2 & mini*
   - **CPU Frequency:** *80 MHz* or *160 MHz*
   - **Flash Size:** *4MB (FS:none OTA:~1019KB)* or standard *4MB (FS:2MB)*
   - **Upload Speed:** *921600* (or *115200*)

---

## 3. Flash the Firmware

- Open **`/home/user/esp8266-bme280-controller.ino`** in the Arduino IDE.
- Connect your ESP8266 via USB, select the corresponding COM/TTY port, and click **Upload**.

---

## 4. Connect & Access Dashboard

1. **Connect to Wi-Fi Access Point:**
   - **SSID:** `ESP8266_BME280`
   - **Password:** `12345678`
2. **Access Web Interface:**
   - A captive portal popup will appear automatically on your phone or PC.
   - If it does not auto-open, navigate to: **`http://192.168.4.1/`**

---

## 5. Daily Operation

- **Live Dashboard:** Displays real-time Temperature (°C/°F), Humidity (%RH), Dew Point, Pressure (hPa), Altitude, rolling trend graphs, and relay states.
- **Relay Modes (Settings Tab):**
  - **Unified Mode:** Relay 1 (GPIO 14 / D5) activates whenever **any** threshold condition triggers.
  - **Individual Mode:** Relay 1 (D5) triggers on Temperature, Relay 2 (D6) on Humidity, Relay 3 (D7) on Pressure.
  - **Polarity:** Select *Active HIGH* or *Active LOW* to match your relay board.
- **Hardware Factory Reset:**
  - Hold the onboard **FLASH / BOOT** button (GPIO 0 / D3) for **5 seconds**. All relays will de-energize and settings will revert to defaults (`ESP8266_BME280` / `12345678`).
