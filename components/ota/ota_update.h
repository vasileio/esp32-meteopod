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
#include "app_context.h"


/* The URL to fetch the firmware binary from */
#ifndef OTA_URL
#define OTA_URL CONFIG_OTA_FIRMWARE_URL
#endif

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
