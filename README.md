# 🚗 ESP32 Real-Time Accident Detection System

<div align="center">

![ESP32](https://img.shields.io/badge/ESP32-Microcontroller-blue?style=for-the-badge&logo=espressif)
![Arduino](https://img.shields.io/badge/Arduino-IDE-teal?style=for-the-badge&logo=arduino)
![C++](https://img.shields.io/badge/C++-Embedded-orange?style=for-the-badge&logo=cplusplus)
![License: MIT](https://img.shields.io/badge/License-MIT-yellow?style=for-the-badge)

**A compact IoT system for real-time crash detection, automatic fuel cutoff, and emergency SMS alerts.**

</div>

---

## 📖 Project Overview

The **ESP32 Real-Time Accident Detection System** is an embedded IoT solution designed to detect vehicular accidents as they happen and respond automatically — without human intervention. It uses a combination of physics-based sensors to analyze motion, jerk, rotation, and driver presence, then instantly triggers safety actions including fuel cutoff and SMS emergency alerts with GPS location.

The system is built in two units:
- 🎽 **Wearable Unit** — worn by the driver; detects motion and biometric signals
- 🚘 **Vehicle Unit** — mounted in the vehicle; controls buzzer and MOSFET fuel cutoff

### ✨ Key Features

| Feature | Description |
|--------|-------------|
| 🧠 Physics-based crash detection | Uses acceleration (`g`), jerk (`g/s`), and angular velocity (`°/s`) |
| 📍 Real-time GPS location | Sends coordinates on crash via Twilio SMS |
| ⛽ Fuel cutoff simulation | MOSFET-based hardware safety action |
| 🔔 Buzzer alert | Immediate audible on-site alarm |
| 🪑 Dual-layer validation | Piezo sensor confirms driver presence before triggering |
| 📊 3-tier severity classification | Minor / Moderate / Severe crash levels |

---

## 🧩 Prerequisites

Before getting started, make sure you have the following:

- Basic familiarity with Arduino IDE
- An ESP32 development board (e.g. ESP32 DevKit V1)
- The components listed in [Hardware Requirements](#-hardware-requirements)
- A [Twilio account](https://www.twilio.com/) for SMS alerts (free trial works)
- A USB cable for flashing firmware
- A stable Wi-Fi connection (for SMS via Twilio)

---

## 🔧 Hardware Requirements

### Components List

| # | Component | Quantity | Notes |
|---|-----------|----------|-------|
| 1 | ESP32 DevKit V1 | 1 | Main microcontroller |
| 2 | MPU6050 (GY-521) | 1 | 6-axis IMU (accelerometer + gyroscope) |
| 3 | NEO-6M GPS Module | 1 | UART GPS with antenna |
| 4 | Piezoelectric Disc Sensor | 1 | Detects seat vibration / driver presence |
| 5 | Active Buzzer | 1 | 5V buzzer for alarm |
| 6 | IRFZ44N N-Channel MOSFET | 1 | Simulates fuel pump cutoff |
| 7 | LED (any color) | 1 | Represents fuel cutoff load |
| 8 | 10kΩ Resistor | 1 | Pull-down for MOSFET gate |
| 9 | 1kΩ Resistor | 1 | Gate resistor for MOSFET |
| 10 | Breadboard + Jumper Wires | — | Prototyping |
| 11 | USB Cable (Micro-USB) | 1 | For programming and power |
| 12 | 5V Power Source | 1 | Power bank or USB adapter |

---

## 💻 Software Requirements

### 1. Arduino IDE

Download and install Arduino IDE (v1.8.x or v2.x):
👉 https://www.arduino.cc/en/software

### 2. ESP32 Board Package

Add ESP32 support to Arduino IDE:

1. Open Arduino IDE → `File` → `Preferences`
2. In **Additional Board Manager URLs**, paste:
   ```
   https://raw.githubusercontent.com/espressif/arduino-esp32/gh-pages/package_esp32_index.json
   ```
3. Go to `Tools` → `Board` → `Boards Manager`
4. Search for `esp32` by Espressif Systems → click **Install**

### 3. Required Libraries

Install these via `Sketch` → `Include Library` → `Manage Libraries`:

| Library | Purpose | Search Term |
|---------|---------|-------------|
| `Adafruit MPU6050` | IMU sensor driver | `Adafruit MPU6050` |
| `Adafruit Unified Sensor` | Dependency for Adafruit libs | `Adafruit Unified Sensor` |
| `TinyGPS++` | GPS NMEA parsing | `TinyGPSPlus` |
| `Wire` | I2C communication | *(built-in)* |
| `WiFi` | Wi-Fi connectivity | *(built-in for ESP32)* |
| `HTTPClient` | HTTP requests for Twilio | *(built-in for ESP32)* |

---

## 🔌 Wiring & Pin Configuration

### ESP32 Pin Map

| Component | ESP32 Pin | Signal | Notes |
|-----------|-----------|--------|-------|
| **MPU6050** SDA | GPIO 21 | I2C Data | Use 3.3V power |
| **MPU6050** SCL | GPIO 22 | I2C Clock | Use 3.3V power |
| **MPU6050** VCC | 3.3V | Power | — |
| **MPU6050** GND | GND | Ground | — |
| **NEO-6M GPS** TX | GPIO 16 (RX2) | UART RX | Cross-connect TX→RX |
| **NEO-6M GPS** RX | GPIO 17 (TX2) | UART TX | Cross-connect RX→TX |
| **NEO-6M GPS** VCC | 3.3V | Power | — |
| **Piezo Sensor** | GPIO 34 | Analog Input | ADC-only pin |
| **Buzzer** (+) | GPIO 18 | Digital Output | Add transistor if needed |
| **MOSFET** Gate | GPIO 23 | Digital Output | Via 1kΩ resistor |
| **MOSFET** Source | GND | — | — |
| **MOSFET** Drain | LED → 5V | Load | LED simulates fuel pump |

### Wiring Diagram Notes

```
ESP32 GPIO 21 ──────── MPU6050 SDA
ESP32 GPIO 22 ──────── MPU6050 SCL
ESP32 3.3V    ──────── MPU6050 VCC
ESP32 GND     ──────── MPU6050 GND

ESP32 GPIO 16 (RX2) ── NEO-6M TX
ESP32 GPIO 17 (TX2) ── NEO-6M RX
ESP32 3.3V          ── NEO-6M VCC

ESP32 GPIO 34 ──────── Piezo Sensor Signal
ESP32 GPIO 18 ──────── Buzzer (+)
ESP32 GPIO 23 ──[1kΩ]── IRFZ44N Gate
                         IRFZ44N Drain ──[LED]── 5V
                         IRFZ44N Source ── GND
                    [10kΩ pull-down between Gate and GND]
```

---

## ⚙️ Core Logic

### Crash Detection Algorithm

1. **MPU6050** samples acceleration and rotation at ~200 Hz
2. Gravity vector is subtracted to isolate **pure linear acceleration**
3. **Piezo plate** validates driver presence before processing
4. A crash is triggered if **any** threshold is exceeded:

```
a     > 2 g       → acceleration threshold
jerk  > 8 g/s     → rate of change of acceleration
gyro  > 300 °/s   → angular velocity
```

5. Crash severity is classified as:

| Severity | Threshold | Action |
|----------|-----------|--------|
| ⚪ Minor | ≥ 2 g | Log event |
| 🟡 Moderate | ≥ 3 g | Buzzer + SMS |
| 🔴 Severe | ≥ 5 g | Buzzer + Fuel Cutoff + SMS |

6. On confirmed crash → **Buzzer ON** + **MOSFET fuel cutoff** + **Twilio SMS with GPS coordinates**

---

## 📤 Firmware Upload Instructions

### Step-by-Step

1. **Clone the repository:**
   ```bash
   git clone https://github.com/asifa1510/AccidentIoT.git
   cd AccidentIoT
   ```

2. **Open the firmware in Arduino IDE:**
   ```
   File → Open → code.ino
   ```

3. **Configure your credentials** in `code.ino`:
   ```cpp
   // Wi-Fi Credentials
   const char* ssid     = "YOUR_WIFI_SSID";
   const char* password = "YOUR_WIFI_PASSWORD";

   // Twilio Credentials
   const char* account_sid   = "YOUR_TWILIO_SID";
   const char* auth_token    = "YOUR_TWILIO_AUTH_TOKEN";
   const char* from_number   = "+1XXXXXXXXXX";  // Twilio number
   const char* to_number     = "+91XXXXXXXXXX"; // Recipient number
   ```

4. **Select the correct board:**
   ```
   Tools → Board → ESP32 Arduino → ESP32 Dev Module
   ```

5. **Select the correct port:**
   ```
   Tools → Port → COM3 (Windows) or /dev/ttyUSB0 (Linux/Mac)
   ```

6. **Upload the firmware:**
   - Click the ➡️ **Upload** button, or press `Ctrl + U`
   - Wait for `Done uploading.` message

7. **Open Serial Monitor** to verify output:
   ```
   Tools → Serial Monitor → Baud Rate: 115200
   ```

---

## 📱 Twilio SMS Setup

1. **Create a free Twilio account** at https://www.twilio.com/

2. **Get a Twilio phone number** (free trial gives you one)

3. **Locate your credentials** in the Twilio Console:
   - `Account SID` — found on the dashboard
   - `Auth Token` — found on the dashboard (click to reveal)

4. **Update the firmware** with your credentials (see Step 3 above)

5. **Verify the recipient number** if using a trial account:
   ```
   Twilio Console → Phone Numbers → Verified Caller IDs
   ```

> ⚠️ **Note:** Free trial accounts can only send SMS to verified numbers. Upgrade for unrestricted sending.

### Example SMS format sent on crash:
```
🚨 ACCIDENT DETECTED!
Severity: SEVERE
Location: 28.6139° N, 77.2090° E
Google Maps: https://maps.google.com/?q=28.6139,77.2090
Speed: 0 km/h
Time: 2026-05-13 10:43
```

---

## 🖼️ Hardware Photos

### Version 1 — Prototype

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/fc6608b9-73db-4ae1-88b8-f4aab9f4d7e8" />
<img width="600" height="600" alt="MOSFET" src="https://github.com/user-attachments/assets/7bf03624-39d5-435e-bf6e-73b8a9550ae6" />

---

### Version 2 — Improved Build

<img width="3000" height="1350" alt="image" src="https://github.com/user-attachments/assets/bfa911a2-ca79-457f-a5c7-a8aacc752c2a" />
<img width="1178" height="395" alt="image" src="https://github.com/user-attachments/assets/e7b9b8a2-8e9a-40e8-91c6-8a49a4513e1d" />
<img width="977" height="538" alt="image" src="https://github.com/user-attachments/assets/a6ecff5e-60b0-46a1-a9df-a04577d39ecd" />

#### 🎽 Wearable Unit
<img width="389" height="490" alt="image" src="https://github.com/user-attachments/assets/9c37a5ca-9808-4d31-94d9-4871f6a2daa6" />
<img width="358" height="431" alt="image" src="https://github.com/user-attachments/assets/4fd3b98b-d381-499b-8f7a-25cc4e231b42" />

#### 🚘 Vehicle Unit
<img width="1080" height="601" alt="Vehicle Unit" src="https://github.com/user-attachments/assets/b1b089ba-d245-4061-a7b0-5287b8ce84e9" />

---

## 🔬 Novelty & Technical Highlights

- **Dual-layer crash validation** — motion data + seat-presence sensing (piezo sensor) prevents false positives
- **Physics-based crash modeling** — acceleration, jerk, and angular rotation analyzed together
- **Gravity-compensated IMU processing** at 200 Hz for true linear acceleration extraction
- **MOSFET-based fuel cutoff simulation** — real hardware safety action, not just a software flag
- **Two-unit architecture** — separates wearable biometric detection from vehicle safety actuation

---

## 🛠️ Troubleshooting

| Problem | Likely Cause | Fix |
|---------|-------------|-----|
| No Serial output | Wrong baud rate | Set Serial Monitor to `115200` |
| `GPS: No Fix` | Poor signal or indoor use | Move near a window or outdoors |
| MPU6050 not detected | Wiring issue | Check SDA/SCL connections & power (3.3V) |
| SMS not sending | Wrong Twilio credentials | Double-check Account SID, Auth Token, phone numbers |
| False crash triggers | Low thresholds or vibration | Tune `ACCEL_THRESHOLD`, `JERK_THRESHOLD` in code |
| Upload fails | Wrong COM port or board | Verify under `Tools → Board` and `Tools → Port` |
| Buzzer always ON | GPIO state on boot | Add pull-down resistor or check firmware logic |
| Wi-Fi not connecting | Wrong SSID/password | Check credentials; ESP32 only supports 2.4 GHz |

---

## 📁 Repository Structure

```
AccidentIoT/
├── code.ino          # Main firmware (Version 1)
├── version2/         # Version 2 firmware and schematics
│   └── ...
├── README.md         # Project documentation
└── LICENSE           # MIT License
```

---

## 🛣️ Working Summary

The system comprises a **wearable unit** (driver-worn) and a **vehicle-mounted unit**. Sensors including MPU6050, ADXL345, and piezo transducers continuously capture motion and impact data at high frequency. The ESP32 processes this data in real-time using gravity-compensated IMU fusion to detect crash events and estimate severity. Upon detection of a critical event, the system simultaneously triggers the buzzer, activates MOSFET-based fuel cutoff, and transmits the GPS coordinates via Twilio SMS to a preconfigured emergency contact.

---

## 🛠️ Tech Stack

| Layer | Technology |
|-------|-----------|
| Microcontroller | ESP32 (Xtensa LX6, dual-core, 240 MHz) |
| Sensors | MPU6050, ADXL345, NEO-6M GPS, Piezo |
| Language | Embedded C++ (Arduino framework) |
| Communication | Wi-Fi + Twilio REST API (SMS) |
| Safety Actuator | IRFZ44N MOSFET (fuel cutoff) |

---

## 🚀 Future Improvements

- ☁️ Cloud dashboard for real-time crash event logging
- 📱 Mobile app integration for live monitoring
- 🤖 ML-based crash classification for improved accuracy
- 🔋 Low-power sleep mode to extend battery life
- 🩺 Heart rate sensor integration for driver health monitoring
- 🌐 Multi-unit fleet tracking support

---

## 🤝 Contributing

Contributions are welcome! Here's how to get started:

1. **Fork** this repository
2. Create a new branch:
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. Make your changes and commit:
   ```bash
   git commit -m "Add: brief description of change"
   ```
4. Push to your fork:
   ```bash
   git push origin feature/your-feature-name
   ```
5. Open a **Pull Request** with a clear description of your changes

Please open an [Issue](https://github.com/asifa1510/AccidentIoT/issues) before working on major changes.

---

## 📄 License

This project is licensed under the [MIT License](LICENSE).

---

<div align="center">
Made with ❤️ for road safety · <a href="https://github.com/asifa1510/AccidentIoT">AccidentIoT</a>
</div>