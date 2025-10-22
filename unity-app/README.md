# Unity Test Application for ESP32 Meteopod

This directory contains the Unity test framework application for comprehensive unit testing of ESP32 Meteopod components. The test application builds a separate firmware that runs all sensor driver tests on the actual ESP32 hardware.

## Overview

The Unity test app provides:
- **Hardware-in-the-loop testing** on real ESP32 devices
- **Automated test execution** with CI/CD integration
- **JUnit XML output** for test reporting
- **Comprehensive mocking** of I2C and hardware interfaces
- **Coverage reporting** for sensor drivers and utilities

## Project Structure

```
unity-app/
├── CMakeLists.txt              # Unity test app build configuration
├── README.md                   # This documentation
├── conftest.py                 # pytest configuration for ESP-IDF
├── main/
│   ├── CMakeLists.txt          # Main component build config
│   └── test_app_main.c         # Unity test app entry point
├── sdkconfig                   # ESP32 configuration (auto-generated)
├── sdkconfig.defaults          # Default ESP32 settings for tests
└── test_unit_case.py           # pytest test case runner
```

## Building Tests

### Prerequisites

Ensure ESP-IDF environment is set up:

```bash
# Linux/macOS
. $IDF_PATH/export.sh

# Windows
%IDF_PATH%\export.ps1
```

### Build Commands

```bash
# Build all tests (from project root)
idf.py -C unity-app -DTESTS_ALL=1 build

# Clean build
idf.py -C unity-app -DTESTS_ALL=1 fullclean

# Build and flash to device
idf.py -C unity-app -DTESTS_ALL=1 flash

# Build, flash, and monitor
idf.py -C unity-app -DTESTS_ALL=1 flash monitor
```

### Alternative Build (from unity-app directory)

```bash
cd unity-app

# Build all tests
idf.py -DTESTS_ALL=1 build

# Flash to device
idf.py -DTESTS_ALL=1 flash monitor
```

## Running Tests

### Method 1: Manual Execution

Flash the test firmware and connect manually:

```bash
# Flash test firmware
idf.py -C unity-app -DTESTS_ALL=1 flash

# Connect to serial monitor
idf.py -C unity-app monitor

# In the Unity menu, type '*' and press Enter to run all tests
```

### Method 2: Automated Test Runner

Use the automated test runner for CI/CD integration:

```bash
# Run all tests with automated output capture
python3 test_runner.py \
    --port /dev/ttyUSB0 \
    --firmware unity-app/build/esp32_meteopod_test_app.bin \
    --timeout 300 \
    --output results.xml
```

#### Test Runner Options

```bash
python3 test_runner.py --help

Options:
  --port PORT              Serial port (e.g., /dev/ttyUSB0, COM3)
  --firmware FIRMWARE      Path to test firmware binary
  --output OUTPUT          JUnit XML output file (default: results.xml)
  --timeout TIMEOUT        Test timeout in seconds (default: 120)
  --baud BAUD             Serial baud rate (default: 115200)
  --skip-flash            Skip firmware flashing (use existing firmware)
  --diagnostic            Run connection diagnostic first
  --unity-cmd CMD         Unity command to run (default: *)
  --no-auto-trigger       Disable automatic test triggering
```

### Method 3: pytest Integration

For advanced test scenarios using pytest-embedded:

```bash
# Run specific test group
pytest --embedded-services esp,idf --test-name="TEST_GROUP=sensors"

# Run all tests
pytest --embedded-services esp,idf --test-name="TEST_ALL"
```

## Test Components

The Unity app tests the following components:

### Sensor Drivers (✅ Tested)
- **BME280**: Temperature, humidity, pressure sensor
- **SHT31**: Temperature and humidity sensor
- **BH1750**: Digital light sensor (lux measurement)
- **MPU6050**: 6-axis accelerometer and gyroscope
- **Wind Sensor**: ADC-based wind direction and speed
- **AS3935**: Lightning detection sensor
- **DFRobot Rainfall**: Tipping bucket rain gauge

### Utilities (✅ Tested)
- **System Monitor**: Uptime, memory, Wi-Fi monitoring
- **Utils**: String manipulation and helper functions

### Test Infrastructure
- **I2C Test Stubs**: Mock I2C communication for isolated testing
- **ADC Mocking**: Simulated ADC readings for analog sensors
- **FreeRTOS Stubs**: Minimal RTOS compatibility for test environment

## Test Configuration

### Unity Configuration

The test app includes these Unity components:
- **Core Unity Framework**: Basic assertion and test runner
- **Unity Fixture**: Advanced test organization and setup/teardown
- **Unity Memory**: Memory leak detection

### ESP32 Configuration

Key settings in `sdkconfig.defaults`:
```ini
# Unity test framework
CONFIG_UNITY_ENABLE_FIXTURE=y
CONFIG_UNITY_ENABLE_MEMORY=y

# Test-specific settings
CONFIG_TESTS_ALL=1
CONFIG_FREERTOS_HZ=1000

# Debug settings for testing
CONFIG_ESP_SYSTEM_PANIC_PRINT_HALT=y
CONFIG_ESP_DEBUG_OCDAWARE=y
```

## Output Format

### Console Output

Tests produce structured Unity output:
```
ESP32 Meteopod Test Suite
Running 90 tests...

components/sensors/test/test_bme280.c:45:test_bme280_init:PASS
components/sensors/test/test_sht31.c:67:test_sht31_read_data:PASS
components/sensors/test/test_bh1750.c:23:test_bh1750_init:PASS
...

-----------------------
90 Tests 0 Failures 0 Ignored
OK
```

### JUnit XML Output

The test runner generates JUnit-compatible XML for CI/CD integration:

```xml
<?xml version="1.0" ?>
<testsuites>
  <testsuite name="ESP32_Unity_Tests" tests="90" failures="0" skipped="0">
    <testcase classname="test_bme280.c" name="test_bme280_init" time="0.0"/>
    <testcase classname="test_sht31.c" name="test_sht31_read_data" time="0.0"/>
    <!-- ... more test cases ... -->
  </testsuite>
</testsuites>
```

## CI/CD Integration

### Example GitHub Actions

```yaml
name: ESP32 Tests
on: [push, pull_request]

jobs:
  test:
    runs-on: self-hosted  # With ESP32 hardware
    steps:
      - uses: actions/checkout@v3
      
      - name: Setup ESP-IDF
        run: |
          . $IDF_PATH/export.sh
          
      - name: Build Tests
        run: |
          idf.py -C unity-app -DTESTS_ALL=1 build
          
      - name: Run Tests
        run: |
          python3 test_runner.py \
            --port /dev/ttyUSB0 \
            --firmware unity-app/build/esp32_meteopod_test_app.bin \
            --timeout 300 \
            --output test-results.xml
            
      - name: Publish Test Results
        uses: dorny/test-reporter@v1
        if: always()
        with:
          name: ESP32 Tests
          path: test-results.xml
          reporter: java-junit
```

### Example Jenkins Pipeline

```groovy
pipeline {
    agent { label 'esp32-runner' }
    stages {
        stage('Build Tests') {
            steps {
                sh '''
                    . $IDF_PATH/export.sh
                    idf.py -C unity-app -DTESTS_ALL=1 build
                '''
            }
        }
        stage('Run Tests') {
            steps {
                sh '''
                    python3 test_runner.py \
                        --port /dev/ttyUSB0 \
                        --firmware unity-app/build/esp32_meteopod_test_app.bin \
                        --timeout 300 \
                        --output results.xml
                '''
            }
            post {
                always {
                    junit 'results.xml'
                }
            }
        }
    }
}
```

## Troubleshooting

### Common Issues

**No test output / timeout**:
```bash
# Check ESP32 connection
python3 test_runner.py --port /dev/ttyUSB0 --diagnostic

# Verify correct serial port
ls /dev/ttyUSB*  # Linux
ls /dev/tty.usbserial*  # macOS

# Test manual connection
screen /dev/ttyUSB0 115200
```

**Build failures**:
```bash
# Clean and rebuild
idf.py -C unity-app -DTESTS_ALL=1 fullclean
idf.py -C unity-app -DTESTS_ALL=1 build

# Check ESP-IDF version
idf.py --version  # Should be v5.0+
```

**Flash failures**:
```bash
# Hold ESP32 BOOT button during flash
idf.py -C unity-app -DTESTS_ALL=1 flash

# Try different baud rate
idf.py -C unity-app -DTESTS_ALL=1 -b 460800 flash
```

### Debug Mode

Enable verbose output for debugging:

```bash
# Build with debug symbols
idf.py -C unity-app -DTESTS_ALL=1 -DCMAKE_BUILD_TYPE=Debug build

# Run with detailed logging
python3 test_runner.py \
    --port /dev/ttyUSB0 \
    --firmware unity-app/build/esp32_meteopod_test_app.bin \
    --timeout 600 \
    --diagnostic
```

## Test Coverage

Current coverage: **53.5%** (1,809 of 3,380 lines tested)

**Covered components**:
- BME280: Temperature/pressure/humidity (✅ 95% coverage)
- SHT31: Temperature/humidity (✅ 92% coverage)  
- BH1750: Light sensor (✅ 88% coverage)
- MPU6050: 6-axis IMU (✅ 90% coverage)
- Wind sensors: Direction/speed (✅ 85% coverage)
- AS3935: Lightning detection (✅ 87% coverage)
- Rainfall sensor: Precipitation monitoring (✅ 89% coverage)
- System utilities: Monitoring and helpers (✅ 91% coverage)

**Areas for improvement**:
- Wi-Fi driver testing
- MQTT client testing
- OTA update testing
- Error recovery scenarios

The Unity test framework provides robust validation of sensor drivers and core functionality, ensuring reliable operation across different hardware configurations and environmental conditions.