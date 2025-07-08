/**
 * @file ota_update.c
 * @brief OTA firmware update task implementation using HTTPS.
 *
 * This module implements a FreeRTOS task that performs a secure OTA firmware update
 * using the ESP-IDF `esp_https_ota()` API. It uses a mutex to guard against
 * concurrent OTA updates.
 */

#include "ota_update.h"

static const char *TAG = "ota_update";
static SemaphoreHandle_t ota_mutex = NULL;

/**
 * @brief OTA update task.
 *
 * This task downloads a new firmware binary from the URL defined by `OTA_URL`,
 * verifies it, and performs an OTA update. If the update is successful, the device
 * reboots automatically. If another OTA is already in progress, this task exits early.
 *
 * @param[in] pvParameter Unused parameter (set to NULL when creating the task).
 */
void ota_update_task(void *pvParameter)
{
    app_ctx_t *ctx = (app_ctx_t*)pvParameter;
    
    /* Create the mutex once if not already created */
    if (ota_mutex == NULL) {
        ota_mutex = xSemaphoreCreateMutex();
        configASSERT(ota_mutex != NULL);
    }

    /* Try to acquire the mutex. If already taken, another OTA is running. */
    if (xSemaphoreTake(ota_mutex, 0) != pdTRUE) {
        ESP_LOGW(TAG, "OTA already in progress. Skipping.");
        vTaskDelete(NULL);
        return;
    }

    ESP_LOGI(TAG, "Starting OTA update from URL: %s", OTA_URL);

    static esp_http_client_config_t http_config = {
        .url                        = OTA_URL,
        .timeout_ms                = 5000,
        .transport_type            = HTTP_TRANSPORT_OVER_TCP,
        .skip_cert_common_name_check = true
    };

    static esp_https_ota_config_t ota_config = {
        .http_config = &http_config,
    };

    esp_err_t ret = esp_https_ota(&ota_config);

    if (ret == ESP_OK) {
        ESP_LOGI(TAG, "OTA update successful!");

        /* Publish status to OTA status topic */
        int msg_id = esp_mqtt_client_publish(
            ctx->mqtt_client,
            ctx->ota_status_topic,
            "OTA successful",
            0,      /* use strlen internally */
            1,      /* QoS 1 */
            0       /* not retained */
        );
        ESP_LOGI(TAG, "Published OTA status (msg_id=%d)", msg_id);

        ESP_LOGI(TAG, "Rebooting...");
        /* No need to release the mutex; device will reboot */
        esp_restart();
    } else {
        ESP_LOGE(TAG, "OTA update failed: %s", esp_err_to_name(ret));
        /* Publish status to OTA status topic */
        int msg_id = esp_mqtt_client_publish(
            ctx->mqtt_client,
            ctx->ota_status_topic,
            "OTA failed",
            0,      /* use strlen internally */
            1,      /* QoS 1 */
            0       /* not retained */
        );
        ESP_LOGI(TAG, "Published OTA status (msg_id=%d)", msg_id);

        xSemaphoreGive(ota_mutex);
    }

    vTaskDelete(NULL);
}
