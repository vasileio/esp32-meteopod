# esp32-meteopod

An ESP32 RTOS-style embedded project that periodically reads weather-related sensor data and can publish it via MQTT.\
The project is in early development and aims to demonstrate real-time scheduling, ISR-triggered tasks, and sensor interfacing under FreeRTOS.

## Current Features

- FreeRTOS-based task scheduling
- Interrupt-safe sensor reading triggers
- MQTT-ready architecture
- Modular structure for Wi-Fi and sensor logic
- HTTP OTA firmware update via MQTT

## Build & Flash

```bash
idf.py build
idf.py flash monitor
```

Ensure the ESP-IDF environment is initialised with:

```bash
. $IDF_PATH/export.sh
```

Or run `export.ps1` on Windows (ensure PowerShell scripts are allowed to run).

---

## Wi-Fi Configuration

To set the Wi-Fi SSID and password without hardcoding them into source files:

1. **Create **``\
   If it doesn't exist, create the file and add:

   ```kconfig
   menu "Wi-Fi Configuration"

   config WIFI_SSID
       string "Wi-Fi SSID"
       default "myssid"

   config WIFI_PASS
       string "Wi-Fi Password"
       default "mypassword"

   endmenu
   ```

2. **Set credentials via **``

   Run:

   ```bash
   idf.py menuconfig
   ```

   Then navigate to:

   ```
   → Wi-Fi Configuration
     → Wi-Fi SSID
     → Wi-Fi Password
   ```

   Enter your Wi-Fi network details and save.

---

## OTA (Over-the-Air) Firmware Updates

This project supports HTTP OTA firmware updates triggered by MQTT.

### Hosting the Firmware

You can host the `.bin` firmware file using:

- A simple HTTP server like Python’s `http.server`
- A local or remote MinIO bucket with public access
- Any HTTP-accessible web server

Make sure your ESP32 can reach the server over the same network.

### Setting the OTA Firmware URL

Add the following to your `sdkconfig.defaults` or via `idf.py menuconfig → OTA Firmware Update`:

```ini
CONFIG_OTA_FIRMWARE_URL="http://<your-ip>:<port>/esp32-firmware/esp32_meteopod.bin"
CONFIG_PARTITION_TABLE_CUSTOM=y
CONFIG_PARTITION_TABLE_CUSTOM_FILENAME="partitions.csv"
CONFIG_BOOTLOADER_APP_ROLLBACK_ENABLE=y
CONFIG_BOOTLOADER_APP_ELF_SHA256=y
CONFIG_OTA_DATA_ERASE=y
```

If you want to use HTTP instead of HTTPS, make sure this is also set:

```ini
CONFIG_ESP_HTTPS_OTA_ALLOW_HTTP=y
```

### Triggering OTA via MQTT

Once the device is running, you can trigger an OTA update by publishing this MQTT message:

- **Topic:** `meteopod/<DEVICE_MAC>/ota/cmd`
- **Payload:** `update`

The device will attempt to fetch the firmware from the configured URL and update itself.

Status is published on:

- **Topic:** `meteopod/<DEVICE_MAC>/ota/status`

Payloads include:

- `updating`
- `success`
- `failure`

---

## Status

> 🧪 Work in Progress — MQTT publishing and sensor queueing coming soon.

## 🧰 ESP-IDF Installation

To build and flash this project, you need to install the [Espressif IoT Development Framework (ESP-IDF)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).

### 🔧 Install on Windows (Recommended via Installer)

1. Download the ESP-IDF Tools Installer from the official site:\
   👉 [https://dl.espressif.com/dl/esp-idf/](https://dl.espressif.com/dl/esp-idf/)

2. Run the installer:

   - Choose your ESP-IDF version (e.g. v5.4.1)
   - Let it install Git, Python, and toolchains automatically

3. Launch the **ESP-IDF PowerShell Environment** from Start Menu

4. Clone this project and build:

   ```bash
   git clone https://github.com/vasileio/esp32-meteopod.git
   cd esp32-meteopod
   idf.py set-target esp32
   idf.py build
   ```

### 🧪 Verify your setup

```bash
idf.py --version
```

This should show the installed ESP-IDF version.

See the full guide:\
📚 [https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/)

---

## 🐧 ESP-IDF Installation on Linux/macOS

1. Clone the ESP-IDF repository:

   ```bash
   git clone --recursive https://github.com/espressif/esp-idf.git
   cd esp-idf
   ./install.sh esp32
   ```

2. Add the environment setup script to your shell profile:

   ```bash
   echo 'source $HOME/esp/esp-idf/export.sh' >> ~/.bashrc
   source ~/.bashrc
   ```

3. Clone and build this project:

   ```bash
   git clone https://github.com/vasileio/esp32-meteopod.git
   cd esp32-meteopod
   idf.py set-target esp32
   idf.py build
   ```

> You can also use `install.fish` or `install.ps1` depending on your shell.

---

## 🔁 Development Workflow Diagram

```text
+------------------------+
|   VS Code / CLI IDE   |
+-----------+------------+
            |
            v
+------------------------+
|       Edit Code        |
+-----------+------------+
            |
            v
+------------------------+
|    idf.py build        |
|    idf.py flash        |
+-----------+------------+
            |
            v
+------------------------+
|    ESP32 Device        |
|  (runs firmware, logs) |
+------------------------+
```

Use `idf.py monitor` to view logs in real time from the serial port.

---

