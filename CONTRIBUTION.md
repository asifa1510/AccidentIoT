# Contributing to AccidentIoT

Thank you for your interest in contributing to **AccidentIoT**! 🚗

AccidentIoT is an ESP32/Arduino-based real-time accident detection system that combines motion sensors, GPS, piezo sensing, and a MOSFET-based fuel cutoff simulation to detect crashes and provide severity-based alerts. Contributions that improve crash detection, sensor integration, firmware reliability, alert handling, and documentation are welcome.

---

## Ways to Contribute

You can contribute by:

- Improving crash detection and severity classification.
- Enhancing MPU6050 or ADXL345 sensor processing.
- Improving GPS integration and location handling.
- Enhancing piezo-based driver-presence detection.
- Improving alert and SMS functionality.
- Improving the fuel cutoff simulation.
- Optimizing ESP32/Arduino firmware.
- Adding tests and improving reliability.
- Improving documentation and setup instructions.
- Working on future features such as cloud alerts or mobile app integration.

---

## Getting Started

1. Fork the repository.
2. Clone your fork locally.
3. Connect the required hardware according to the documented pin configuration.
4. Create a new branch for your changes.
5. Make your changes.
6. Upload the firmware to the ESP32/Arduino.
7. Test the changes using the serial console or available alert system.
8. Commit your changes and open a Pull Request.

---

## Hardware and Development Guidelines

When working with hardware-related features:

- Follow the documented ESP32 pin configuration.
- Test sensor changes carefully before using them with the complete system.
- Avoid introducing changes that could cause unexpected hardware behavior.
- Keep sensor processing and threshold logic clearly documented.
- Clearly document any new hardware components or wiring requirements.

---

## Pull Request Guidelines

Before submitting a Pull Request:

- Clearly describe what was changed.
- Explain any changes to sensor logic or crash thresholds.
- Mention hardware components affected by the change.
- Test the firmware on the relevant hardware whenever possible.
- Keep Pull Requests focused on a single issue or feature.
- Include screenshots, serial output, or other evidence when useful.

Example:

```text
Fixes #123
```

Testing

Before submitting your changes:

Verify that the firmware builds successfully.
Upload and test the firmware on the relevant ESP32/Arduino hardware.
Check sensor readings through the serial console.
Test the relevant crash-detection conditions.
Verify that alerts and connected components behave as expected.
Ensure existing functionality is not broken.
Reporting Issues

When reporting an issue, please include:

A clear description of the problem.
Steps to reproduce it.
Expected and actual behavior.
Hardware components being used.
ESP32/Arduino board information.
Relevant serial console output or screenshots.
Any firmware or configuration details that may help reproduce the issue.
Safety

This project interacts with hardware and includes a fuel-cutoff simulation. Contributors should test changes carefully and avoid connecting experimental firmware directly to real vehicle fuel systems.

The documented MOSFET-based fuel cutoff is intended as a simulation and testing mechanism.

Community Standards

By participating in this project, you agree to follow the project's Code of Conduct.

Please review CODE_OF_CONDUCT.md before contributing.

Thank you for contributing to AccidentIoT and helping improve real-time accident detection and vehicle safety technology!🚗
