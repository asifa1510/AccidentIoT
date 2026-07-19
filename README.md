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

## NOVELTY
- Dual-layer crash validation using motion + seat-presence sensing (piezo sensor) 
- Physics-based crash modeling using acceleration, jerk, and angular rotation together  
- Gravity-compensated IMU processing at 200 Hz for true linear acceleration  
- Actual hardware safety action via **MOSFET-based fuel cutoff simulation**
 

---

<img width="500" height="500" alt="image" src="https://github.com/user-attachments/assets/fc6608b9-73db-4ae1-88b8-f4aab9f4d7e8" />

<img width="600" height="600" alt="MOSFET" src="https://github.com/user-attachments/assets/7bf03624-39d5-435e-bf6e-73b8a9550ae6" />



# VERSION 2:

<img width="3000" height="1350" alt="image" src="https://github.com/user-attachments/assets/bfa911a2-ca79-457f-a5c7-a8aacc752c2a" />


<img width="1178" height="395" alt="image" src="https://github.com/user-attachments/assets/e7b9b8a2-8e9a-40e8-91c6-8a49a4513e1d" />


<img width="977" height="538" alt="image" src="https://github.com/user-attachments/assets/a6ecff5e-60b0-46a1-a9df-a04577d39ecd" />


### WEARABLE UNIT:

<img width="389" height="490" alt="image" src="https://github.com/user-attachments/assets/9c37a5ca-9808-4d31-94d9-4871f6a2daa6" />

<img width="358" height="431" alt="image" src="https://github.com/user-attachments/assets/4fd3b98b-d381-499b-8f7a-25cc4e231b42" />


### VEHICLE UNIT:
<img width="1080" height="601" alt="WhatsApp Image 2026-05-01 at 2 53 38 PM" src="https://github.com/user-attachments/assets/b1b089ba-d245-4061-a7b0-5287b8ce84e9" />

---
## Working
The system consists of a wearable unit and a vehicle-mounted unit. Sensors like MPU6050, ADXL345, and piezo sensors capture motion and impact data. This data is processed using ESP32/Arduino to detect crash events and estimate severity. If a critical event is detected, GPS location is sent via alerts and a fuel cutoff mechanism is triggered.

## Tech Stack
- ESP32 / Arduino
- MPU6050, ADXL345, GPS, Piezo Sensors
- Embedded C++

## How to Run
1. Connect sensors to ESP32/Arduino
2. Upload the firmware code
3. Power the system
4. Monitor output via serial console or alert system

## Future Improvements
- Cloud-based alert system
- Mobile app integration
- Improved crash classification models

Contributions are welcome! Feel free to open issues or submit pull requests to improve the project.

## ✨ README Improvement Notes

### 📌 Formatting Enhancements Needed
- Improve heading hierarchy for better readability
- Ensure consistent spacing between sections
- Use proper Markdown formatting for code blocks and lists
- Align all installation and usage steps properly

### 🚀 Suggested Structure Upgrade
- Introduction
- Features
- Tech Stack
- Installation
- Usage
- Project Structure
- Contribution Guidelines
- License

### 🛠️ Documentation Improvements
- Add badges (optional): build, license, contributors
- Add screenshots for better UI understanding
- Standardize code blocks for commands

### 🎯 Goal
Improve onboarding experience for new contributors and users by making README more structured, readable, and professional.

