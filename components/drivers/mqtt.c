#include "mqtt.h"
#include <cJSON.h>

#ifndef ARRAY_SIZE
#define ARRAY_SIZE(x) (sizeof(x) / sizeof((x)[0]))
#endif

#define STR_HELPER(x) #x
#define STR(x) STR_HELPER(x)

#define TAG "MQTT"

/**
 * @brief  Clear a payload buffer.
 * 
 * @param buf       Pointer to the buffer to wipe.
 * @param buf_size  Size of the buffer in bytes.
 */
static inline void wipe_payload(char *buf, size_t buf_size)
{
    if (buf && buf_size > 0) {
        memset(buf, 0, buf_size);
    }
}

/** @brief List of all Home Assistant-exposed sensors. */
static const ha_sensor_config_t ha_sensors[] = {
    { "metrics",                "Uptime",                   "ms",  "{{ value_json.uptime_ms }}",    "duration",         "diagnostic", NULL },
    { "rssi",                   "Wi-Fi RSSI",               "dBm", "{{ value_json.rssi }}",         "signal_strength",  "diagnostic", NULL },
    { "ip_address",             "IP Address",               NULL,    "{{ value_json.ip_address }}",   NULL,               "diagnostic", NULL },
    { "bme280_temperature",     "Case Temperature",         "°C",  "{{ value_json.temperature }}",  "temperature",      "diagnostic", NULL },
    { "bme280_humidity",        "Case Humidity",            "%",   "{{ value_json.humidity }}",     "humidity",         "diagnostic", NULL },
    { "bme280_pressure",        "Atmospheric Pressure",     "hPa", "{{ value_json.pressure }}",     "pressure",         NULL,         NULL },
    { "sht31_temperature",      "Temperature",              "°C",  "{{ value_json.temperature }}",  "temperature",      NULL,         NULL },
    { "sht31_humidity",         "Humidity",                 "%",   "{{ value_json.humidity }}",     "humidity",         NULL,         NULL }
};


/**
 * Example MQTT discovery config payload built by this function:
 *
 * Topic: homeassistant/sensor/meteopod_XXYYZZ/bme280_temperature/config
 * Payload:
 * {
 *   "name": "BME280 Temperature",
 *   "state_topic": "meteopod/XXYYZZ/sensor/bme280",
 *   "unit_of_measurement": "°C",
 *   "value_template": "{{ value_json.temperature }}",
 *   "unique_id": "bme280_temperature_XXYYZZ",
 *   "device_class": "temperature",
 *   "device": {
 *     "identifiers": "esp32-meteopod-XXYYZZ",
 *     "name": "Meteopod Dev - Study",
 *     "model": "Meteopod v0.0.3",
 *     "manufacturer": "vasileio"
 *   }
 * }
 */

static void publish_HA_discovery_config(esp_mqtt_client_handle_t client,
                                        const char *mac_str,
                                        const char *suffix,
                                        const char *name,
                                        const char *unit,
                                        const char *value_template,
                                        const char *device_class,
                                        const char *entity_category,
                                        const char *sensor_topic,
                                        const char *device_id,
                                        const char *device_name)
{
    char topic[192];
    char payload[512];
    char unique_id[96];

    // Unique ID now includes MAC to prevent collision across devices
    snprintf(unique_id, sizeof(unique_id), "%s_%s", suffix, mac_str);

    // e.g. homeassistant/sensor/meteopod_XXYYZZ/bme280_temperature/config
    snprintf(topic, sizeof(topic),
             "homeassistant/sensor/meteopod_%s/%s/config", mac_str, suffix);

    cJSON *root = cJSON_CreateObject();

    cJSON_AddStringToObject(root, "name", name);
    cJSON_AddStringToObject(root, "state_topic", sensor_topic);
    cJSON_AddStringToObject(root, "unit_of_measurement", unit);
    cJSON_AddStringToObject(root, "value_template", value_template);
    cJSON_AddStringToObject(root, "unique_id", unique_id);
    /* force the Entity Registry object_id to be exactly our suffix */
    cJSON_AddStringToObject(root, "object_id", suffix);
    if (device_class) {
        cJSON_AddStringToObject(root, "device_class", device_class);
    }

    if (entity_category) {
        cJSON_AddStringToObject(root, "entity_category", entity_category);
    }

    cJSON *device = cJSON_CreateObject();
    cJSON_AddStringToObject(device, "identifiers", device_id);
    cJSON_AddStringToObject(device, "name", CONFIG_MQTT_DEVICE_NAME);

    char model_str[64];
    snprintf(model_str, sizeof(model_str), "Meteopod %s", STR(CONFIG_APP_PROJECT_VER));
    cJSON_AddStringToObject(device, "model", model_str);

    cJSON_AddStringToObject(device, "manufacturer", "vasileio");
    cJSON_AddItemToObject(root, "device", device);

    snprintf(payload, sizeof(payload), "%s", cJSON_PrintUnformatted(root));
    cJSON_Delete(root);

    esp_mqtt_client_publish(client, topic, payload, 0, 1, true);  // retain = true
}

/**
 * @brief Clear all retained MQTT discovery topics for Home Assistant.
 *
 * Publishes empty retained messages to each known discovery config topic.
 */
static void clear_HA_discovery_configs(esp_mqtt_client_handle_t client, const char *mac)
{
    char topic[192];

    for (size_t i = 0; i < ARRAY_SIZE(ha_sensors); ++i) {
        snprintf(topic, sizeof(topic),
                 "homeassistant/sensor/meteopod_%s/%s/config",
                 mac, ha_sensors[i].suffix);

        esp_mqtt_client_publish(client, topic, "", 0, 1, true);
        ESP_LOGI(TAG, "Cleared discovery topic: %s", topic);
    }
}

/**
 * Map a sensor‐suffix to the correct MQTT topic string in ctx.
 * Returns NULL if we don’t know that suffix, so the caller can skip it.
 */
static const char *get_topic_for_suffix(const char *suffix, app_ctx_t *ctx) {
    // BME280 sensors all start with "bme280"
    if (strncmp(suffix, "bme280", 6) == 0) {
        return ctx->sensor_bme280_topic;
    }
    // SHT31 sensors all start with "sht31"
    if (strncmp(suffix, "sht31", 5) == 0) {
        return ctx->sensor_sht31_topic;
    }
    // Everything else on the single "metrics" topic
    if (strcmp(suffix, "metrics")   == 0 ||
        strcmp(suffix, "rssi")      == 0 ||
        strcmp(suffix, "ip_address")== 0) {
        return ctx->metrics_topic;
    }
    // (If you add a new topic, just insert another clause here)
    return NULL;
}



/**
 * @brief Publish MQTT discovery configuration for all known sensors to Home Assistant.
 *
 * This function constructs discovery topics and JSON payloads for all defined sensors,
 * assigning the correct sensor topic based on the suffix, and publishes them with retain=true.
 *
 * @param[in] client     Handle to the initialised MQTT client
 * @param[in] ctx        Application context with MQTT topic strings and MAC address
 */
static void publish_all_discovery_configs(esp_mqtt_client_handle_t client, app_ctx_t *ctx)
{
    const char *mac = ctx->device_mac_str;
    char device_id[64];
    snprintf(device_id, sizeof(device_id),
             "esp32-meteopod-%s", mac);

    /* 1) clear any old retained configs */
    clear_HA_discovery_configs(client, mac);

    /* 2) copy the static sensor‐definitions */
    ha_sensor_config_t sensors[ARRAY_SIZE(ha_sensors)];
    memcpy(sensors, ha_sensors, sizeof(ha_sensors));

    /* 3) single, clean loop */
    for (size_t i = 0; i < ARRAY_SIZE(sensors); ++i) {
        const char *topic = get_topic_for_suffix(sensors[i].suffix, ctx);
        if (!topic) {
            ESP_LOGW(TAG, "Skipping unknown sensor suffix: %s",
                     sensors[i].suffix);
            continue;
        }
        sensors[i].sensor_topic = topic;

        publish_HA_discovery_config(
            client,
            mac,
            sensors[i].suffix,
            sensors[i].name,
            sensors[i].unit,
            sensors[i].value_template,
            sensors[i].device_class,
            sensors[i].entity_category,
            sensors[i].sensor_topic,
            device_id,
            "ESP32 Meteopod"
        );
    }
}

/**
 * @brief Build all MQTT topics based on the device’s topic prefix.
 */
esp_err_t mqtt_build_all_topics(app_ctx_t *ctx)
{
    esp_err_t err;
    if (!ctx) {
        return ESP_ERR_INVALID_ARG;
    }

    /* Build the topic prefix: "meteopod/<MAC>" */
    snprintf(ctx->topic_prefix,
            sizeof(ctx->topic_prefix),
            "meteopod/%s",
            ctx->device_mac_str);

    /* Base sensor topic */
    err = utils_build_topic(ctx->topic_prefix,
                            "sensor",
                            ctx->sensor_topic,
                            sizeof(ctx->sensor_topic));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to build sensor topic: %s", esp_err_to_name(err));
        return err;
    }

    /* Per-sensor subtopics */
    err = utils_build_topic(ctx->sensor_topic,
                            "bme280",
                            ctx->sensor_bme280_topic,
                            sizeof(ctx->sensor_bme280_topic));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to build BME280 topic: %s", esp_err_to_name(err));
        return err;
    }
    err = utils_build_topic(ctx->sensor_topic,
                            "sht31",
                            ctx->sensor_sht31_topic,
                            sizeof(ctx->sensor_sht31_topic));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to build SHT31 topic: %s", esp_err_to_name(err));
        return err;
    }
    err = utils_build_topic(ctx->sensor_topic,
                            "rainfall",
                            ctx->sensor_rainfall_topic,
                            sizeof(ctx->sensor_rainfall_topic));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to build rainfall topic: %s", esp_err_to_name(err));
        return err;
    }

    /* Metrics topic */
    err = utils_build_topic(ctx->topic_prefix,
                            "metrics",
                            ctx->metrics_topic,
                            sizeof(ctx->metrics_topic));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to build metrics topic: %s", esp_err_to_name(err));
        return err;
    }

    /* OTA topics */
    err = utils_build_topic(ctx->topic_prefix,
                            "ota",
                            ctx->ota_topic,
                            sizeof(ctx->ota_topic));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to build OTA base topic: %s", esp_err_to_name(err));
        return err;
    }
    err = utils_build_topic(ctx->ota_topic,
                            "cmd",
                            ctx->ota_cmd_topic,
                            sizeof(ctx->ota_cmd_topic));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to build OTA command topic: %s", esp_err_to_name(err));
        return err;
    }
    err = utils_build_topic(ctx->ota_topic,
                            "status",
                            ctx->ota_status_topic,
                            sizeof(ctx->ota_status_topic));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to build OTA status topic: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

/**
 * @brief MQTT event handler
 */
static void mqtt_event_handler(void *arg,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    if (!arg || !event_data) {
        ESP_LOGE(TAG, "mqtt_event_handler: null arg or event_data");
        return;
    }

    app_ctx_t *ctx = (app_ctx_t*)arg;
    esp_mqtt_event_handle_t evt = (esp_mqtt_event_handle_t)event_data;
    int mid;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");

            /* Subscriptions */
            xEventGroupSetBits(ctx->mqttEventGroup, MQTT_CONNECTED_BIT);
            if (evt->client) {
                ESP_LOGI(TAG, "susbscribing to topic");
                mid = esp_mqtt_client_subscribe(evt->client,
                                                ctx->ota_cmd_topic,
                                                1);
                ESP_LOGI(TAG, "Subscribed to OTA cmd: %s (mid=%d)",
                         ctx->ota_cmd_topic, mid);
            }

            /* Publish Home Assistant discovery configs */
            publish_all_discovery_configs(evt->client, ctx);

            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            xEventGroupClearBits(ctx->mqttEventGroup, MQTT_CONNECTED_BIT);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "Message published (msg_id=%d)", evt->msg_id);
            break;

        case MQTT_EVENT_DATA: {
            char topic_buf[128] = {0};
            char data_buf[64]  = {0};

            if (evt->topic && evt->topic_len > 0) {
                snprintf(topic_buf, sizeof(topic_buf), "%.*s",
                         (int)evt->topic_len, evt->topic);
            }
            ESP_LOGI(TAG, "snprintf(topic_buf, sizeof(topic_buf)");
            if (evt->data && evt->data_len > 0) {
                snprintf(data_buf, sizeof(data_buf), "%.*s",
                         (int)evt->data_len, evt->data);
            }
            ESP_LOGI(TAG, "MQTT DATA: topic='%s', payload='%s'",
                     topic_buf, data_buf);

            if (strcmp(topic_buf, ctx->ota_cmd_topic) == 0 &&
                strcmp(data_buf, "update") == 0) {
                ESP_LOGW(TAG, "OTA update via MQTT");
                int msg_id = esp_mqtt_client_publish(
                    evt->client,
                    ctx->ota_status_topic,
                    "OTA started",
                    0,
                    1,      /* QoS 1 */
                    0       /* not retained */
                );
                ESP_LOGI(TAG, "Published \"OTA started\" (msg_id=%d)", msg_id);
                xTaskCreate(&ota_update_task,
                            "ota_update", 8192, ctx, 5, NULL);
            }
        } break;

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error occurred");
            break;

        case MQTT_EVENT_SUBSCRIBED:
            ESP_LOGI(TAG, "Subscription acknowledged (msg_id=%d)", evt->msg_id);
            break;

        case MQTT_EVENT_BEFORE_CONNECT:
            ESP_LOGI(TAG, "MQTT client initialised; about to connect to %s", CONFIG_MQTT_BROKER_URI);
            break;

        default:
            ESP_LOGI(TAG, "Unhandled event id=%" PRIi32, event_id);
            break;
    }
}

/**
 * @brief MQTT task to manage connection and publish messages.
 */
void mqtt_task(void *pvParameters)
{
    app_ctx_t *ctx = (app_ctx_t *)pvParameters;
    mqtt_queue_item_t req;
    EventBits_t bits;

    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = CONFIG_MQTT_BROKER_URI
    };

    ctx->mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(ctx->mqtt_client,
                                   ESP_EVENT_ANY_ID,
                                   mqtt_event_handler,
                                   ctx);
    esp_mqtt_client_start(ctx->mqtt_client);

    bits = xEventGroupWaitBits(
        ctx->mqttEventGroup,
        MQTT_CONNECTED_BIT,
        pdFALSE,
        pdTRUE,
        pdMS_TO_TICKS(5000)
    );

    for (;;) {
        if (xQueueReceive(ctx->mqttPublishQueue, &req, portMAX_DELAY) == pdTRUE) {
            bits = xEventGroupWaitBits(
                ctx->mqttEventGroup,
                MQTT_CONNECTED_BIT,
                pdFALSE,
                pdTRUE,
                pdMS_TO_TICKS(5000));
        }
        if (!(bits & MQTT_CONNECTED_BIT)) {
            ESP_LOGW(TAG, "Dropped publish: MQTT not connected");
            continue;
        }

        switch (req.type) {
        case MSG_METRICS: {
            system_metrics_t *m = &req.data.metrics;
            // Make buffer big enough for the extra fields
            char payload[256] = {0};
                
            // Build JSON with uptime, heap stats, RSSI and IP address
            snprintf(payload, sizeof(payload),
                "{"
                  "\"uptime_ms\":%lu,"
                  "\"min_free_heap\":%lu,"
                  "\"free_heap\":%lu,"
                  "\"stack_watermark\":%lu,"
                  "\"rssi\":%ld,"
                  "\"ip_address\":\"%s\""
                "}",
                m->uptime_ms,
                m->min_free_heap,
                m->free_heap,
                m->stack_watermark,
                m->wifi_rssi,
                m->ip_address
            );
        
            int msg_id = esp_mqtt_client_publish(
                ctx->mqtt_client,
                ctx->metrics_topic,
                payload,
                0,
                1,      /* QoS 1 */
                0       /* not retained */);
            ESP_LOGI(TAG, "Published metrics: %s (msg_id=%d)", payload, msg_id);
            } break;

            case MSG_SENSOR: {
                sensor_readings_t *m = &req.data.sensor;

                /* BME280 */
                char payload[128] = {0};
                snprintf(payload, sizeof(payload),
                    "{\"temperature\":%.2f,"
                    "\"humidity\":%.2f,"
                    "\"pressure\":%.2f"
                    "}",
                    m->bme280_readings.temperature,
                    m->bme280_readings.humidity,
                    m->bme280_readings.pressure
                );

                int msg_id = esp_mqtt_client_publish(
                    ctx->mqtt_client,
                    ctx->sensor_bme280_topic,
                    payload,
                    0,
                    1,      /* QoS 1 */
                    0       /* not retained */
                );
                ESP_LOGI(TAG, "Published BME280 sensor readings: %s (msg_id=%d)", payload, msg_id);

                /* SHT31 */
                wipe_payload(payload, sizeof(payload));
                snprintf(payload, sizeof(payload),
                    "{\"temperature\":%.2f,"
                    "\"humidity\":%.2f"
                    "}",
                    m->sht31_readings.temperature,
                    m->sht31_readings.humidity);

                msg_id = esp_mqtt_client_publish(
                    ctx->mqtt_client,
                    ctx->sensor_sht31_topic,
                    payload,
                    0,
                    1,      /* QoS 1 */
                    0       /* not retained */
                );
                ESP_LOGI(TAG, "Published SHT31 sensor readings: %s (msg_id=%d)", payload, msg_id);
            } break;
            // add more message types here if you need them

            default:
                ESP_LOGW(TAG, "Unknown queue req type %d", req.type);
                break;
        }
    }


    /* Should never be reached */
    esp_mqtt_client_stop(ctx->mqtt_client);
    esp_mqtt_client_destroy(ctx->mqtt_client);
    vEventGroupDelete(ctx->mqttEventGroup);
    vQueueDelete(ctx->mqttPublishQueue);
    vTaskDelete(NULL);
}
