# ESP32 Meteopod

A comprehensive ESP32-based weather monitoring station that reads data from multiple environmental sensors and seamlessly integrates with Home Assistant via MQTT Discovery. This embedded project demonstrates professional IoT development practices using FreeRTOS, robust error handling, and comprehensive testing.

## Features

### 🌤️ Weather Monitoring
- **Temperature & Humidity**: BME280 (internal/case) and SHT31 (ambient/external) sensors
- **Atmospheric Pressure**: BME280 barometric sensor with altitude compensation
- **Wind Measurement**: Directional wind sensor with speed calculation
- **Light Sensing**: BH1750 digital illuminance sensor (lux measurement)
- **Lightning Detection**: AS3935 lightning sensor with distance and energy estimation
- **Rainfall Monitoring**: Tipping bucket rain gauge with cumulative and hourly totals

### 📊 Motion & Orientation
- **6-Axis IMU**: MPU6050 accelerometer and gyroscope for device positioning and vibration monitoring

### 🏠 Home Assistant Integration  
- **MQTT Discovery**: Automatic sensor discovery and configuration
- **Real-time Data**: Live sensor readings with configurable intervals
- **Device Management**: Unified device representation with diagnostic sensors
- **Conditional Publishing**: Only initialized sensors appear in Home Assistant

### 🔧 System Features
- **FreeRTOS Architecture**: Task-based design with proper scheduling and resource management
- **Over-the-Air Updates**: HTTP-based firmware updates triggered via MQTT
- **System Monitoring**: Uptime tracking, Wi-Fi signal strength, and IP address reporting
- **Error Recovery**: Graceful sensor failure handling with initialization status tracking
- **Comprehensive Testing**: 53.5% code coverage with Unity test framework

## Quick Start

### Prerequisites

Ensure ESP-IDF v5.0+ is installed and environment is set up:

```bash
# Linux/macOS
. $IDF_PATH/export.sh

# Windows PowerShell
%IDF_PATH%\export.ps1
```

### Build and Flash

```bash
# Clone the repository
git clone https://github.com/vasileio/esp32-meteopod.git
cd esp32-meteopod

# Configure Wi-Fi credentials (required)
idf.py menuconfig
# Navigate to: Wi-Fi Configuration → Set SSID and Password

# Build and flash
idf.py build
idf.py flash monitor
```

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

## Current Sensor Readings

The device publishes sensor data to the following MQTT topics:

- `meteopod/<MAC>/sensor/bme280` - Case temperature, humidity, pressure (diagnostic)
- `meteopod/<MAC>/sensor/sht31` - Ambient temperature and humidity 
- `meteopod/<MAC>/sensor/wind` - Wind direction and speed
- `meteopod/<MAC>/sensor/light` - Illuminance in lux
- `meteopod/<MAC>/sensor/mpu6050` - 6-axis acceleration and gyroscope data (diagnostic)
- `meteopod/<MAC>/sensor/as3935` - Lightning detection and distance
- `meteopod/<MAC>/sensor/rainfall` - Rainfall measurements and bucket tip counts
- `meteopod/<MAC>/metrics` - System metrics (uptime, RSSI, IP address)

### Example Sensor Data

```json
// BME280 (Case sensors)
{"temperature": 23.45, "humidity": 65.2, "pressure": 1013.25}

// SHT31 (Ambient sensors) 
{"temperature": 22.1, "humidity": 68.5}

// Wind measurements
{"direction": "NE", "speed": 2.3}

// Lightning detection
{"distance_km": 15, "strike_energy": 12450}

// Rainfall data
{"cumulative": 2.5, "rainfall_1h": 0.8, "raw_count": 125}
```

## 🧰 ESP-IDF Installation

To build and flash this project, you need to install the [Espressif IoT Development Framework (ESP-IDF)](https://docs.espressif.com/projects/esp-idf/en/latest/esp32/get-started/).

### Windows Installation

1. **Download ESP-IDF Tools Installer** from the official Espressif website:
   - [ESP-IDF Tools Installer](https://dl.espressif.com/dl/esp-idf/)

2. **Run the installer**:
   - Select ESP-IDF version v5.0 or later (recommended: v5.4.1+)
   - Allow automatic installation of Git, Python, and required toolchains

3. **Launch development environment**:
   - Open **ESP-IDF PowerShell Environment** from Start Menu

4. **Clone and build**:
   ```bash
   git clone https://github.com/vasileio/esp32-meteopod.git
   cd esp32-meteopod
   idf.py set-target esp32
   idf.py menuconfig  # Configure Wi-Fi credentials
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

### Linux/macOS Installation

1. **Clone ESP-IDF repository**:
   ```bash
   git clone --recursive https://github.com/espressif/esp-idf.git
   cd esp-idf
   ./install.sh esp32
   ```

2. **Set up environment** (add to shell profile for persistence):
   ```bash
   # For bash
   echo 'source $HOME/esp/esp-idf/export.sh' >> ~/.bashrc
   source ~/.bashrc
   
   # For zsh
   echo 'source $HOME/esp/esp-idf/export.sh' >> ~/.zshrc
   source ~/.zshrc
   ```

3. **Clone and build this project**:
   ```bash
   git clone https://github.com/vasileio/esp32-meteopod.git
   cd esp32-meteopod
   idf.py set-target esp32
   idf.py menuconfig  # Configure Wi-Fi credentials
   idf.py build
   ```

**Alternative shells**: Use `install.fish` or `install.ps1` as appropriate for your environment.

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

## Testing & Development

### Running Tests

The project includes comprehensive unit tests using the Unity framework:

```bash
# Build and run all tests
cd unity-app
idf.py -DTESTS_ALL=1 build

# Build and flash tests to device
idf.py -DTESTS_ALL=1 flash monitor
```

**Test Coverage**: 53.5% (1,809 of 3,380 lines tested)

### Tested Components

- ✅ BME280 temperature/pressure/humidity sensor
- ✅ SHT31 temperature/humidity sensor  
- ✅ BH1750 light sensor
- ✅ MPU6050 6-axis IMU
- ✅ Wind direction/speed sensors
- ✅ AS3935 lightning detector
- ✅ DFRobot rainfall sensor
- ✅ System monitoring utilities
- ✅ I2C test stubs and mocking

### Development Notes

- All sensor drivers include comprehensive error handling
- FreeRTOS mutex protection for I2C bus access
- Graceful degradation when sensors fail to initialize
- Extensive logging for debugging and monitoring
- Memory leak prevention with proper resource cleanup

---

