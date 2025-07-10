#include "mqtt.h"

#define TAG "mqtt_task"

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
            xEventGroupSetBits(ctx->mqttEventGroup, MQTT_CONNECTED_BIT);
            if (evt->client) {
                ESP_LOGI(TAG, "susbscribing to topic");
                mid = esp_mqtt_client_subscribe(evt->client,
                                                ctx->ota_cmd_topic,
                                                1);
                ESP_LOGI(TAG, "Subscribed to OTA cmd: %s (mid=%d)",
                         ctx->ota_cmd_topic, mid);
            }
            ESP_LOGI(TAG, "Cleared retained metrics");
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
                safe_mqtt_publish(evt->client,
                                  ctx->ota_status_topic,
                                  "OTA started",
                                  0,
                                  1,
                                  0);
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
        .broker.address.uri = "mqtt://192.168.1.118:1883"
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
                char payload[128] = {0};
                char buf[128];
                int len = snprintf(payload, sizeof(payload),
                    "{\"uptime_ms\":%lu,"
                    "\"min_free_heap\":%lu,"
                    "\"free_heap\":%lu,"
                    "\"stack_watermark\":%lu"
                    "}",
                    m->uptime_ms,
                    m->min_free_heap,
                    m->free_heap,
                    m->stack_watermark
                );

                int msg_id = esp_mqtt_client_publish(
                    ctx->mqtt_client,
                    ctx->metrics_topic,
                    payload,
                    0,
                    1,      /* QoS 1 */
                    0       /* not retained */
                );
                ESP_LOGI(TAG, "Published metrics: %s (msg_id=%d)", payload, msg_id);
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
