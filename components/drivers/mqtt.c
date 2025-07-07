#include "mqtt.h"

#define TAG "mqtt_task"

/**
 * @brief MQTT event handler
 *
 * Invoked by the ESP-MQTT component on various MQTT events.
 * Updates the MQTT_CONNECTED_BIT in the event group to reflect
 * the current connection status, and logs key events.
 *
 * @param arg          Application context pointer (unused here)
 * @param base         Event base identifier (unused)
 * @param event_id     MQTT event identifier
 * @param event_data   Pointer to event-specific data (esp_mqtt_event_handle_t)
 */
static void mqtt_event_handler(void *arg,
                               esp_event_base_t base,
                               int32_t event_id,
                               void *event_data)
{
    /* grab your app context from arg */
    app_ctx_t *ctx = (app_ctx_t*)arg;

    /* Cast to MQTT event handle */
    esp_mqtt_event_handle_t evt = (esp_mqtt_event_handle_t)event_data;

    /*------------------------------------------------------------------------*/
    /* Handle each event type                                                 */
    /*------------------------------------------------------------------------*/
    switch (event_id) {
        case MQTT_EVENT_CONNECTED:
            ESP_LOGI(TAG, "MQTT connected");
            xEventGroupSetBits(ctx->mqttEventGroup, MQTT_CONNECTED_BIT);
            break;

        case MQTT_EVENT_DISCONNECTED:
            ESP_LOGW(TAG, "MQTT disconnected");
            xEventGroupClearBits(ctx->mqttEventGroup, MQTT_CONNECTED_BIT);
            break;

        case MQTT_EVENT_PUBLISHED:
            ESP_LOGI(TAG, "Message published (msg_id=%d)", evt->msg_id);
            break;

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
    app_ctx_t           *ctx = (app_ctx_t *)pvParameters;
    mqtt_publish_req_t   req;
    EventBits_t          bits;

    /*------------------------------------------------------------------------*/
    /* Create synchronization primitives                                      */
    /*------------------------------------------------------------------------*/
    ctx->mqttEventGroup   = xEventGroupCreate();
    configASSERT(ctx->mqttEventGroup);

    ctx->mqttPublishQueue = xQueueCreate(10, sizeof(mqtt_publish_req_t));
    configASSERT(ctx->mqttPublishQueue);

    /*------------------------------------------------------------------------*/
    /* Configure and start MQTT client                                        */
    /*------------------------------------------------------------------------*/
    const esp_mqtt_client_config_t mqtt_cfg = {
        .broker.address.uri = "mqtt://192.168.1.118:1883"
    };

    ctx->mqtt_client = esp_mqtt_client_init(&mqtt_cfg);
    esp_mqtt_client_register_event(ctx->mqtt_client,
                                   ESP_EVENT_ANY_ID,
                                   mqtt_event_handler,
                                   ctx);
    esp_mqtt_client_start(ctx->mqtt_client);

    /*------------------------------------------------------------------------*/
    /* Publish a “startup” message once connected                             */
    /*------------------------------------------------------------------------*/
    bits = xEventGroupWaitBits(
        ctx->mqttEventGroup,
        MQTT_CONNECTED_BIT,
        pdFALSE,              /* don’t clear bit on exit */
        pdTRUE,               /* wait for bit to be set */
        pdMS_TO_TICKS(5000)   /* 5 s timeout */
    );

    if (bits & MQTT_CONNECTED_BIT) {

        /* Build “<prefix>/status”, e.g. “meteopod/ABCDEF012345/status” */
        char startup_topic[TOPIC_PREFIX_LEN + 8];  // +8 for "/status" and NUL
        snprintf(startup_topic,
                sizeof(startup_topic),
                "%s/status",
                ctx->topic_prefix);

        int startup_id = esp_mqtt_client_publish(
            ctx->mqtt_client,
            startup_topic,          /* meteopod/ABCDEF012345/status */
            "device boot",          /* payload */
            0,                      /* use strlen internally */
            1,                      /* QoS 1 */
            1                       /* retained */
        );
        ESP_LOGI(TAG, "Startup message on 'meteopod/status' published (msg_id=%d)", startup_id);
    } else {
        ESP_LOGW(TAG, "Startup message dropped: not connected");
    }

    /*------------------------------------------------------------------------*/
    /* Main loop: wait for publish requests, then send when connected         */
    /*------------------------------------------------------------------------*/
    for (;;) {
        /* Block indefinitely until a publish request arrives */
        if (xQueueReceive(ctx->mqttPublishQueue, &req, portMAX_DELAY) == pdTRUE) {
            /* Wait up to 5 s for the MQTT_CONNECTED_BIT */
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

    /*------------------------------------------------------------------------*/
    /* Cleanup (never reached)                                                */
    /*------------------------------------------------------------------------*/
    esp_mqtt_client_stop(ctx->mqtt_client);
    esp_mqtt_client_destroy(ctx->mqtt_client);
    vEventGroupDelete(ctx->mqttEventGroup);
    vQueueDelete(ctx->mqttPublishQueue);
    vTaskDelete(NULL);
}

