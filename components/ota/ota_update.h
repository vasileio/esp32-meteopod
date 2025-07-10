/**
 * @file ota_update.h
 * @brief Interface for triggering OTA firmware updates on the ESP32.
 *
 * This header provides the declaration for the OTA update task. The OTA
 * update is performed via HTTPS using the ESP-IDF `esp_https_ota()` API.
 */

#ifndef OTA_UPDATE_H
#define OTA_UPDATE_H

#ifdef __cplusplus
extern "C" {
#endif

#include "esp_log.h"
#include "esp_https_ota.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "mqtt_client.h"


/* The URL to fetch the firmware binary from */
#ifndef OTA_URL
#define OTA_URL CONFIG_OTA_FIRMWARE_URL
#endif


/**
 * @brief OTA (Over‐The‐Air) update status structure.
 *
 * Represents the current state and progress of an OTA operation.
 */
typedef struct {
    char     status[16];  /**< Operation state: "idle", "started", "in_progress", "completed", or "failed". */
    char     version[32]; /**< Firmware version being applied or currently running. */
    uint8_t  progress;    /**< Percent completion (0–100), meaningful when status is "in_progress". */
    char     error[128];  /**< Error message if status == "failed"; empty otherwise. */
} ota_status_t;

/**
 * @brief OTA update task.
 *
 * This FreeRTOS task downloads and installs new firmware from the URL defined
 * by `CONFIG_OTA_FIRMWARE_URL`. If an update is already in progress, the task
 * exits early. On success, the device will automatically reboot.
 *
 * @param[in] pvParameter Reserved for future use. Set to NULL.
 */
void ota_update_task(void *pvParameter);

#ifdef __cplusplus
}
#endif

#endif /* OTA_UPDATE_H */
