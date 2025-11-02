# 🚗 ESP32 Real-Time Accident Detection System

A compact IoT project using **ESP32**, **MPU6050**, **GPS**, and **Piezo sensor** for **real-time crash detection** with automatic **fuel cutoff**, **buzzer alert**, and **Twilio SMS** for alert reporting.

---

## Hardware Setup
| Component | ESP32 Pin / Power | Purpose |
|------------|------------------|----------|
| **MPU6050** | SDA → 21, SCL → 22 | Detects acceleration, jerk, tilt |
| **NEO-6M GPS** | TX → 16 (RX2), RX → 17 (TX2) | Location & speed |
| **Piezo Sensor** | Signal → 34 | Detects seat vibration / presence |
| **Buzzer** | + → 18 | Sounds alarm |
| **IRFZ44N MOSFET + LED** | Gate → 23 | Simulates fuel cutoff |
| **ESP32** | USB | Main controller & data processor |

---

## Core Logic
1. **MPU6050** samples acceleration & rotation at ~200 Hz.  
2. Removes gravity to extract **pure linear motion**.  
3. **Piezo plate** confirms driver presence before triggering.  
4. Accident triggers if:
   - `a > 2 g`, `jerk > 8 g/s`, or `gyro > 300 °/s`.  
5. Severity levels:
   - **Minor:** ≥ 2 g  
   - **Moderate:** ≥ 3 g  
   - **Severe:** ≥ 5 g  
6. On crash → buzzer ON, fuel OFF, SMS.

---

##  What Makes This Approach Unique
- **Hybrid sensing:** Combines *MPU6050 motion data* and *piezoelectric presence detection* to eliminate false alarms.  
- **True dynamic severity estimation:** Evaluates acceleration, jerk, and angular velocity instead of a single threshold — mimicking real crash dynamics.  
- **Hardware-level safety cut-off:** MOSFET-controlled “fuel” LED demonstrates instant power isolation, not just software alerting.  
- **Standalone intelligence:** Works without external servers or apps; real-time response is entirely local.  
- **Demo-accurate tuning:** Can simulate genuine crash signatures through precise thresholding rather than random vibration.

---

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/fc6608b9-73db-4ae1-88b8-f4aab9f4d7e8" />


<img width="600" height="600" alt="MOSFET" src="https://github.com/user-attachments/assets/7bf03624-39d5-435e-bf6e-73b8a9550ae6" />

