#include "mqtt.h"

static const char *TAG = "MQTT_TASK";

/* Forward declaration of application context pointer */
static app_ctx_t *s_ctx = NULL;

/**
 * @brief MQTT event handler
 *
 * Called by the ESP-MQTT component on various MQTT events.
 * Sets or clears the connected bit in the event group.
 *
 * @param arg Unused
 * @param base Event base identifier
 * @param event_id Event identifier
 * @param event_data Pointer to event-specific data
 */
static void mqtt_event_handler(void *arg, esp_event_base_t base,
                               int32_t event_id, void *event_data)
{
    esp_mqtt_event_handle_t evt = event_data;
    switch (event_id) {
    case MQTT_EVENT_CONNECTED:
        ESP_LOGI(TAG, "MQTT connected");
        xEventGroupSetBits(s_ctx->mqttEventGroup, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_DISCONNECTED:
        ESP_LOGW(TAG, "MQTT disconnected");
        xEventGroupClearBits(s_ctx->mqttEventGroup, MQTT_CONNECTED_BIT);
        break;
    case MQTT_EVENT_PUBLISHED:
        ESP_LOGI(TAG, "Message published, msg_id=%d", evt->msg_id);
        break;
    case MQTT_EVENT_ERROR:
        ESP_LOGE(TAG, "MQTT error");
        break;
    default:
        break;
    }
}

void mqtt_task(void *pvParameters)
{
    /* Store context pointer for handler access */
    s_ctx = (app_ctx_t *)pvParameters;

    /* Create event group and publish queue */
    s_ctx->mqttEventGroup   = xEventGroupCreate();
    s_ctx->mqttPublishQueue = xQueueCreate(10, sizeof(mqtt_publish_req_t));
    configASSERT(s_ctx->mqttEventGroup && s_ctx->mqttPublishQueue);

    /* Configure MQTT client from menuconfig */
    esp_mqtt_client_config_t cfg = {
        .broker.address.uri = "mqtt://192.168.1.118:1883",
    };
    s_ctx->mqtt_client = esp_mqtt_client_init(&cfg);
    esp_mqtt_client_register_event(s_ctx->mqtt_client,
                                   ESP_EVENT_ANY_ID,
                                   mqtt_event_handler,
                                   NULL);
    esp_mqtt_client_start(s_ctx->mqtt_client);

    mqtt_publish_req_t req;
    for (;;) {
        /* Wait indefinitely for a publish request */
        if (xQueueReceive(s_ctx->mqttPublishQueue, &req, portMAX_DELAY) == pdTRUE) {
            /* Wait up to 5 seconds for MQTT connection */
            EventBits_t bits = xEventGroupWaitBits(
                s_ctx->mqttEventGroup,
                MQTT_CONNECTED_BIT,
                pdFALSE,    /* Don't clear bit on exit */
                pdTRUE,     /* Wait for bit to be set */
                pdMS_TO_TICKS(5000)  /* Timeout */
            );

            if (bits & MQTT_CONNECTED_BIT) {
                int msg_id = esp_mqtt_client_publish(
                    s_ctx->mqtt_client,
                    req.topic,
                    req.payload,
                    req.len,
                    req.qos,
                    req.retain
                );
                ESP_LOGI(TAG, "Published to %s (id=%d)", req.topic, msg_id);
            } else {
                ESP_LOGW(TAG, "Publish dropped, not connected");
            }
        }
    }

    /* Should never reach here */
    vTaskDelete(NULL);
}
