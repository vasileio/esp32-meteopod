# Adding a New Sensor to ESP32 Meteopod

This comprehensive guide walks through all the steps required to integrate a new sensor into the ESP32 Meteopod firmware and expose it to Home Assistant via MQTT Discovery. The example uses a DHT22 temperature & humidity sensor to demonstrate the complete integration process.

---

## 1. Wire up your DHT22 and driver

1. **Connect** the DHT22 data pin to a free GPIO on your ESP32.  
2. **Install** your DHT22 driver (e.g. [`dht.h` / `dht.c`], or the ESP-IDF `dht` component).

   ```c
   #include "dht.h"
   ```

3. **Extend** the `sensor_readings_t` structure in `components/sensors/sensors.h` to hold DHT22 data:

   ```c
   typedef struct {
       bme280_data_t       bme280_readings;
       sht31_data_t        sht31_readings;
       wind_data_t         wind_readings;
       mpu6050_data_t      mpu6050_readings;
       float               light_lux;
       lightning_data_t    lightning_readings;
       bool                lightning_detected;
       float               rainfall_cumulative_mm;
       float               rainfall_1h_mm;
       uint32_t            rainfall_raw_count;
       // Add your new sensor data
       float               dht22_temperature;
       float               dht22_humidity;
       sensor_status_t     sensor_status;
   } sensor_readings_t;
   ```

4. **Update sensor initialization** in `components/sensors/sensors.c`:

   Add to `sensor_status_t` structure in `sensors.h`:
   ```c
   typedef struct {
       bool bme280_ok;
       bool sht31_ok;
       bool wind_ok;
       bool light_ok;
       bool mpu6050_ok;
       bool lightning_ok;
       bool rainfall_ok;
       bool dht22_ok;     // Add this line
   } sensor_status_t;
   ```

   Add DHT22 initialization in `sensors_init()`:
   ```c
   // Initialize DHT22
   esp_err_t dht22_err = dht22_init(GPIO_NUM_XX);
   sensor_status.dht22_ok = (dht22_err == ESP_OK);
   if (sensor_status.dht22_ok) {
       ESP_LOGI(TAG, "DHT22 initialized successfully");
       initialized_count++;
   } else {
       ESP_LOGE(TAG, "DHT22 initialization failed: %s", esp_err_to_name(dht22_err));
   }
   ```

5. **Sample** the DHT22 in `sensors_task()` (in `components/sensors/sensors.c`):

   ```c
   // DHT22 readings (add to sensor reading section)
   if (sensor_status.dht22_ok) {
       float temp, hum;
       if (dht_read_float_data(DHT_TYPE_DHT22, GPIO_NUM_XX, &hum, &temp) == ESP_OK) {
           readings.dht22_temperature = temp;
           readings.dht22_humidity = hum;
           ESP_LOGI(TAG, "DHT22: %.1f°C, %.1f%%", temp, hum);
       } else {
           readings.dht22_temperature = NAN;
           readings.dht22_humidity = NAN;
           ESP_LOGW(TAG, "Failed to read DHT22 sensor");
       }
   } else {
       readings.dht22_temperature = NAN;
       readings.dht22_humidity = NAN;
   }
   ```

---

## 2. Build a DHT22 MQTT topic

In `components/drivers/mqtt.c`, update `mqtt_build_all_topics()`:

```c
// Add after existing sensor topics
// DHT22 subtopic
err = utils_build_topic(ctx->sensor_topic, 
                        "dht22", 
                        ctx->sensor_dht22_topic, 
                        sizeof(ctx->sensor_dht22_topic));
if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to build DHT22 topic: %s", esp_err_to_name(err));
    return err;
}
```

And add a new field to your `app_ctx_t` structure in `main/include/app_context.h`:

```c
typedef struct {
    // ... existing fields ...
    char sensor_bme280_topic[128];
    char sensor_sht31_topic[128];
    char sensor_wind_topic[128];
    char sensor_light_topic[128];
    char sensor_mpu6050_topic[128];
    char sensor_as3935_topic[128];
    char sensor_rainfall_topic[128];
    char sensor_dht22_topic[128];        // Add this line
    char metrics_topic[128];
    // ... other fields ...
} app_ctx_t;
```

---

## 3. Expose via Home Assistant

### 3.1 Add to `ha_sensors[]`

In `components/drivers/mqtt.c`, add to the `ha_sensors[]` array:

```c
static const ha_sensor_config_t ha_sensors[] = {
    // ... existing entries (metrics, bme280, sht31, wind, light, mpu6050, as3935, rainfall) ...
    
    // Add DHT22 sensors
    { "dht22_temperature",      "DHT22 Temperature",        "°C",  "{{ value_json.dht22_temperature }}", "temperature",      NULL,         NULL },
    { "dht22_humidity",         "DHT22 Humidity",           "%",   "{{ value_json.dht22_humidity }}",    "humidity",         NULL,         NULL },
};
```

**Note**: Set `entity_category` to `NULL` for primary sensors or `"diagnostic"` for internal/diagnostic sensors.

### 3.2 Map suffix → topic

In `components/drivers/mqtt.c`, update the `get_topic_for_suffix()` function:

```c
static const char *get_topic_for_suffix(const char *suffix, const app_ctx_t *ctx) {
    // ... existing mappings ...
    
    // DHT22 sensors on their own subtopic
    if (strncmp(suffix, "dht22", 5) == 0) {
        return ctx->sensor_dht22_topic;
    }
    
    // ... other mappings ...
    return NULL;  // Unknown suffix
}
```

### 3.3 Add sensor status checking

Update the `should_publish_sensor()` function to include DHT22 status checking:

```c
static bool should_publish_sensor(const char *suffix, const app_ctx_t *ctx) {
    // ... existing checks ...
    
    // DHT22 sensors
    if (strncmp(suffix, "dht22", 5) == 0) {
        return ctx->sensor_status.dht22_ok;
    }
    
    // ... other checks ...
    return true;  // Default for system sensors
}
```

---

## 4. Publish DHT22 readings

In `components/drivers/mqtt.c`, add to the `MSG_SENSOR` case in `mqtt_task()`:

```c
case MSG_SENSOR: {
    sensor_readings_t *m = &req.data.sensor;
    char payload[512];
    int msg_id;
    
    // ... existing sensor publications (BME280, SHT31, wind, light, etc.) ...
    
    // DHT22 readings
    if (ctx->sensor_status.dht22_ok) {
        wipe_payload(payload, sizeof(payload));
        snprintf(payload, sizeof(payload),
            "{"
            "\"dht22_temperature\":%.2f,"
            "\"dht22_humidity\":%.2f"
            "}",
            m->dht22_temperature,
            m->dht22_humidity);

        msg_id = esp_mqtt_client_publish(
            ctx->mqtt_client,
            ctx->sensor_dht22_topic,
            payload,
            0, 1,  /* QoS 1 */
            0      /* not retained */
        );
        ESP_LOGI(TAG, "Published DHT22: %s (mid=%d)", payload, msg_id);
    }
    
    // ... continue with other sensors ...
} break;
```

**Important**: Only publish sensor data if the sensor initialized successfully (`ctx->sensor_status.dht22_ok`).

---

## 5. Testing and Validation

### 5.1 Add Unit Tests

Create a test file `components/sensors/test/test_dht22.c`:

```c
#include "unity.h"
#include "dht22.h"
#include "i2c_test_stubs.h"

void setUp(void) {
    reset_i2c_stubs();
}

void tearDown(void) {
    // Cleanup if needed
}

void test_dht22_initialization(void) {
    esp_err_t err = dht22_init(GPIO_NUM_4);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

void test_dht22_read_valid_data(void) {
    float temperature, humidity;
    
    // Set up mock data for successful read
    esp_err_t err = dht_read_float_data(DHT_TYPE_DHT22, GPIO_NUM_4, &humidity, &temperature);
    
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 22.5, temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.1, 65.0, humidity);
}
```

Update `components/sensors/test/CMakeLists.txt` to include the new test:

```cmake
set(COMPONENT_SRCS 
    "test_dht22.c"
    # ... other test files ...
)
```

### 5.2 Build and Flash

```bash
# Set ESP-IDF environment
. $IDF_PATH/export.sh

# Build the project
idf.py build

# Flash to device
idf.py flash monitor
```

### 5.3 Monitor Device Logs

Watch for these log messages indicating successful integration:

```
I (1234) SENSORS: DHT22 initialized successfully
I (5678) SENSORS: DHT22: 22.5°C, 65.0%
I (5690) MQTT: Published DHT22: {"dht22_temperature":22.50,"dht22_humidity":65.00} (mid=42)
```

---

## 6. Verify in Home Assistant

### 6.1 Check MQTT Discovery

1. Go to **Settings → Devices & Services → MQTT**
2. Click **⋮ → Reload Entities** to refresh the entity list
3. Look for the new entities:
   - `sensor.meteopod_<MAC>_dht22_temperature`
   - `sensor.meteopod_<MAC>_dht22_humidity`

### 6.2 Monitor MQTT Topics

Use an MQTT client to verify data publication:

```bash
# Subscribe to DHT22 data
mosquitto_sub -h <broker_ip> -t "meteopod/+/sensor/dht22"

# Subscribe to discovery configs
mosquitto_sub -h <broker_ip> -t "homeassistant/sensor/meteopod_+/dht22_+/config"
```

Expected data format:
```json
{
  "dht22_temperature": 22.50,
  "dht22_humidity": 65.00
}
```

### 6.3 Home Assistant Dashboard

The new sensors will appear in Home Assistant and can be added to dashboards. They update every 5 seconds (configurable via `SENSOR_READ_INTERVAL_MS`).

---

## Summary

Following this pattern, you can add any new sensor to ESP32 Meteopod:

1. **Hardware Setup**: Wire the sensor and install its driver
2. **Data Structure**: Extend `sensor_readings_t` and `sensor_status_t`
3. **Initialization**: Add sensor init in `sensors_init()`
4. **Data Collection**: Read sensor data in `sensors_task()`
5. **MQTT Topics**: Add topic building in `mqtt_build_all_topics()`
6. **Home Assistant**: Add to `ha_sensors[]` array and topic mapping
7. **Publishing**: Add sensor data publishing in MQTT task
8. **Testing**: Create unit tests and validate functionality
9. **Verification**: Confirm operation in Home Assistant

This modular approach ensures robust sensor integration with proper error handling, testing, and Home Assistant compatibility.
