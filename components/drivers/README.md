# Adding a New Sensor (DHT22) to Meteopod

This guide will walk you through all the steps required to integrate a new sensor, for the sake of the example let's say the DHT22 temperature & humidity sensor into the Meteopod firmware and expose it to Home Assistant via MQTT Discovery.

---

## 1. Wire up your DHT22 and driver

1. **Connect** the DHT22 data pin to a free GPIO on your ESP32.  
2. **Install** your DHT22 driver (e.g. [`dht.h` / `dht.c`], or the ESP-IDF `dht` component).

   ```c
   #include "dht.h"
   ```

3. **Extend** your `sensor_readings_t` to hold DHT22 data:

   ```c
   typedef struct {
       bme280_data_t    bme280_readings;
       sht31_data_t     sht31_readings;
       float            dht22_temperature;
       float            dht22_humidity;
   } sensor_readings_t;
   ```

4. **Sample** the DHT22 in your sensor task (before you publish `MSG_SENSOR`):

   ```c
   float temp, hum;
   if (dht_read_float_data(DHT_TYPE_DHT22, GPIO_NUM_XX, &hum, &temp) == ESP_OK) {
       m->dht22_temperature = temp;
       m->dht22_humidity    = hum;
   } else {
       m->dht22_temperature = NAN;
       m->dht22_humidity    = NAN;
   }
   ```

---

## 2. Build a DHT22 MQTT topic

In `mqtt_build_all_topics()` add:

```diff
    /* Per-sensor subtopics */
    …
+   // DHT22
+   err = utils_build_topic(ctx->sensor_topic,
+                           "dht22",
+                           ctx->sensor_dht22_topic,
+                           sizeof(ctx->sensor_dht22_topic));
+   if (err != ESP_OK) {
+       ESP_LOGE(TAG, "Failed to build DHT22 topic: %s", esp_err_to_name(err));
+       return err;
+   }
```

And add a new field to your `app_ctx_t`:

```c
char sensor_dht22_topic[128];
```

---

## 3. Expose via Home Assistant

### 3.1 Add to `ha_sensors[]`

In your list of sensors:

```diff
 static const ha_sensor_config_t ha_sensors[] = {
     /* existing entries… */
+    { "dht22_temperature",
+      "DHT22 Temperature",
+      "°C",
+      "{{ value_json.dht22_temperature }}",
+      "temperature",
+      "diagnostic",
+      NULL },
+
+    { "dht22_humidity",
+      "DHT22 Humidity",
+      "%",
+      "{{ value_json.dht22_humidity }}",
+      "humidity",
+      "diagnostic",
+      NULL },
 };
```

### 3.2 Map suffix → topic

In `get_topic_for_suffix()` add:

```c
    // DHT22 on its own subtopic
    if (strncmp(suffix, "dht22", 5) == 0) {
        return ctx->sensor_dht22_topic;
    }
```

---

## 4. Publish your DHT22 readings

In the `MSG_SENSOR` case of your `mqtt_task`:

```diff
 case MSG_SENSOR: {
     sensor_readings_t *m = &req.data.sensor;
     …
+    // DHT22
+    wipe_payload(payload, sizeof(payload));
+    snprintf(payload, sizeof(payload),
+        "{"temperature":%.2f,"
+         ""humidity":%.2f"
+        "}",
+        m->dht22_temperature,
+        m->dht22_humidity);
+
+    msg_id = esp_mqtt_client_publish(
+        ctx->mqtt_client,
+        ctx->sensor_dht22_topic,
+        payload,
+        0, 1,  /* QoS 1 */
+        0      /* not retained */
+    );
+    ESP_LOGI(TAG, "Published DHT22: %s (mid=%d)", payload, msg_id);
     …
 } break;
```

---

## 5. Rebuild & flash

1. **Compile & flash** your ESP32 with the updated code.  
2. When it connects, it will automatically:
   - **Clear** any old retained discovery messages  
   - **Publish** new discovery configs for `dht22_temperature` & `dht22_humidity`  
   - **Publish** sensor readings on `meteopod/<MAC>/sensor/dht22`

---

## 6. Verify in Home Assistant

1. Go to **Settings → Devices & Services → MQTT → ⋮ → Reload Entities**  
2. Look for the new entities:
   - `sensor.dht22_temperature`
   - `sensor.dht22_humidity`  

They should appear under **Diagnostics** (or your chosen category) and update in real time.

---

That’s it! Following this pattern you can add any new sensor:  
1. Read & store data  
2. Build an MQTT topic  
3. Add to `ha_sensors[]`  
4. Map suffix → topic  
5. Publish JSON in `MSG_SENSOR`  
6. Rebuild, flash, and verify.
