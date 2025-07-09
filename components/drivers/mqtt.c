#include "mqtt.h"
#include "app_context.h"

#define TAG "MQTT"

/**
 * @brief MQTT event handler
 *
 * Invoked by the ESP-MQTT component on various MQTT events.
 * Updates the MQTT_CONNECTED_BIT in the event group to reflect
 * the current connection status, and logs key events.
 *
 * @param arg          Application context pointer
 * @param base         Event base identifier (unused)
 * @param event_id     MQTT event identifier
 * @param event_data   Pointer to event-specific data (esp_mqtt_event_handle_t)
 */
static void mqtt_event_handler(void *arg,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    app_ctx_t *ctx = (app_ctx_t*)arg;
    esp_mqtt_event_handle_t evt = (esp_mqtt_event_handle_t)event_data;

    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            xEventGroupSetBits(ctx->mqttEventGroup, MQTT_CONNECTED_BIT);

            /* Topic subscriptions */
            
            int mid = esp_mqtt_client_subscribe(evt->client, ctx->ota_cmd_topic, 1);
            ESP_LOGI(TAG, "Subscribing to OTA topic: %s (msg_id=%d)", ctx->ota_cmd_topic, mid);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            xEventGroupClearBits(ctx->mqttEventGroup, MQTT_CONNECTED_BIT);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "Message published (msg_id=%d)", evt->msg_id);
            break;

        case MQTT_EVENT_DATA: {
            if (evt->topic_len && evt->data_len) {
                char topic_buf[128] = {0};
                char data_buf[64]   = {0};

                memcpy(topic_buf, evt->topic, MIN(evt->topic_len, sizeof(topic_buf) - 1));
                memcpy(data_buf, evt->data, MIN(evt->data_len, sizeof(data_buf) - 1));

                ESP_LOGI(TAG, "MQTT DATA received: topic='%s', payload='%s'", topic_buf, data_buf);

                if (strcmp(topic_buf, ctx->ota_cmd_topic) == 0 &&
                    strcmp(data_buf, "update") == 0)
                {
                    ESP_LOGW(TAG, "OTA update triggered via MQTT!");

                    /* Publish status to OTA status topic */
                    int msg_id = esp_mqtt_client_publish(
                        ctx->mqtt_client,
                        ctx->ota_status_topic,
                        "OTA started",
                        0,      /* use strlen internally */
                        1,      /* QoS 1 */
                        0       /* not retained */
                    );

                    ESP_LOGI(TAG, "Published OTA status (msg_id=%d)", msg_id);
                    xTaskCreate(&ota_update_task, "ota_update", 8192, ctx, 5, NULL);
                }
            }
            break;
        }

        case MQTT_EVENT_ERROR:
            ESP_LOGE(TAG, "MQTT error occurred");
            break;

        default:
            ESP_LOGD(TAG, "Unhandled MQTT event id=%" PRIi32, event_id);
            break;
    }
}

/**
 * @brief MQTT task to manage connection and publish messages.
 *
 * Initializes the MQTT client, publishes a startup message once connected,
 * then enters the main loop to handle queued publish requests.
 *
 * @param pvParameters Pointer to app_ctx_t containing application context.
 */
void mqtt_task(void *pvParameters)
{
    app_ctx_t *ctx = (app_ctx_t *)pvParameters;
    mqtt_publish_req_t req;
    EventBits_t bits;

    ctx->mqttEventGroup   = xEventGroupCreate();
    configASSERT(ctx->mqttEventGroup);

    ctx->mqttPublishQueue = xQueueCreate(10, sizeof(mqtt_publish_req_t));
    configASSERT(ctx->mqttPublishQueue);

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

    if (bits & MQTT_CONNECTED_BIT) {
        char startup_topic[TOPIC_PREFIX_LEN + 8];
        if (utils_build_topic(ctx->topic_prefix, "status", startup_topic, sizeof(startup_topic)) != ESP_OK) {
            ESP_LOGE(TAG, "Failed to build heartbeat topic");
            vTaskDelete(NULL);
            return;
        }

        int startup_id = esp_mqtt_client_publish(
            ctx->mqtt_client,
            startup_topic,
            "device boot",
            0,
            1,
            1
        );
        ESP_LOGI(TAG, "Startup message on '%s' published (msg_id=%d)", startup_topic, startup_id);
    } else {
        ESP_LOGW(TAG, "Startup message dropped: not connected");
    }

    for (;;) {
        if (xQueueReceive(ctx->mqttPublishQueue, &req, portMAX_DELAY) == pdTRUE) {
            bits = xEventGroupWaitBits(
                ctx->mqttEventGroup,
                MQTT_CONNECTED_BIT,
                pdFALSE,
                pdTRUE,
                pdMS_TO_TICKS(5000)
            );

            if (bits & MQTT_CONNECTED_BIT) {
                int msg_id = esp_mqtt_client_publish(
                    ctx->mqtt_client,
                    req.topic,
                    req.payload,
                    req.len,
                    req.qos,
                    req.retain
                );
                ESP_LOGI(TAG, "Published to %s (msg_id=%d)", req.topic, msg_id);
            } else {
                ESP_LOGW(TAG, "Dropped publish: MQTT not connected");
            }
        }
    }

    esp_mqtt_client_stop(ctx->mqtt_client);
    esp_mqtt_client_destroy(ctx->mqtt_client);
    vEventGroupDelete(ctx->mqttEventGroup);
    vQueueDelete(ctx->mqttPublishQueue);
    vTaskDelete(NULL);
}
