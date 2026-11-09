# Autonomous Optical Flow UAV Prototype

Exploratory autonomous quadcopter prototype utilizing optical flow navigation and a custom ESP32 flight controller, benchmarked against ISRO competition specifications.

## Flight Demo & Hardware


## System Architecture
Instead of relying on an off-the-shelf flight controller, this UAV uses a custom dual-core **ESP32** architecture paired with a chassis-mounted **smartphone acting as an onboard companion computer**:
* **Companion Smartphone:** Runs optical flow algorithms using the onboard camera, receives manual overrides via UDP from a ground-station phone, and streams attitude/thrust setpoints (`M,pitch,roll,throttle`) to the ESP32 over a wired USB Serial connection. *(Note: This repository strictly contains the ESP32 firmware; the companion smartphone codebase is not included).*
* **ESP32 Flight Controller:** Executes a deterministic 250Hz Cascaded PID loop (Angle + Rate) on Core 1 while parsing incoming high-speed UART serial commands on Core 0.

## Repository Contents
* `flight_controller_esp.ino` — Primary flight control firmware featuring a 250Hz Cascaded PID controller, complementary IMU filter, and zero-allocation USB Serial parser pinned across dual FreeRTOS cores.
* `pid_cal.ino` — Wireless tethered tuning utility hosting an asynchronous web interface on Core 0 to tune Angle and Rate PID gains in real time.
* `wifi_power_test.ino` — Hardware bring-up utility for verifying motor spin directions, ESC calibration, and live MPU6050 I2C telemetry.

## Engineering Log: Challenges & Solutions

* **No RF Transmitter / RC Hardware:** 
  * *Problem:* Lacked a dedicated radio transmitter and receiver for manual flight testing and override control.
  * *Solution:* Engineered a companion-computer relay. Built a ground-station mobile interface with an on-screen joystick that transmits commands over UDP to the drone-mounted smartphone, which translates and forwards them to the ESP32 via USB Serial.

* **WiFi Latency & Heap Fragmentation During Tuning:** 
  * *Problem:* Running an HTTP web server for PID tuning alongside the flight loop caused memory fragmentation (due to Arduino `String` allocations) and timing jitter.
  * *Solution:* Isolated the communication stack to Core 0 and locked the 250Hz PID loop to Core 1. Replaced dynamic strings with fixed `char` buffers (`snprintf` / `sscanf`) and transitioned the final flight controller to a hardwired 115200 baud UART link.
