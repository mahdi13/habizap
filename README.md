# habizap

Habizap is a hobby project that explores gesture detection using an ESP32 board, an MPU6050 IMU (accelerometer +
gyroscope), and a lightweight machine learning model.

⚠️ Disclaimer:
This is an experimental, in-development project for personal and educational purposes only. It is provided as-is. Use,
modification, or extension of this code is entirely your own responsibility.

---

✨ Features
• Collects accelerometer and gyroscope data from an MPU6050 (training.c).
• Uses Edge Impulse or TensorFlow Lite Micro for real-time gesture classification (inference.c).
• Runs on ESP32 with FreeRTOS.
• Can optionally trigger external feedback devices (e.g., vibration motor).

