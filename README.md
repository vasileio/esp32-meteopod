# esp32-rtos-envnode 🚧 (WIP)

A FreeRTOS-based environmental sensing node for ESP32.  
This work-in-progress project samples orientation and particulate matter (PM) data and publishes it via MQTT.

## Features

- Deterministic sensor polling using `esp_timer` and task notifications
- MQTT-based telemetry from SDS011 (air quality) and IMU sensors
- FreeRTOS task isolation for acquisition, logging, and communication
- Designed with real-time constraints and system modularity in mind

## Platform

- ESP32 (ESP-IDF v5+)
- FreeRTOS
- MQTT publishing

## Project Layout

- `main.c`: Task scheduler and system setup
- `/components`: Sensor interface code (to be added)
- `/docs`: Architecture notes and diagrams

## Status

Early development — core structure and task model in place.  
Sensor integration and message formatting in progress.

---

