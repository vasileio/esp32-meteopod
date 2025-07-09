#ifndef COMPONENTS_DRIVERS_MQTT_INCLUDE_MQTT_H_
#define COMPONENTS_DRIVERS_MQTT_INCLUDE_MQTT_H_

#include <inttypes.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "utils.h"
#include "ota_update.h"
#include "sensors.h"
#include "system_monitor.h"
#include <string.h>

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define MQTT_CONNECTED_BIT BIT0

/**
 * @brief Types of messages that can be sent over the MQTT publish queue.
 *
 * This enum is used in mqtt_queue_item_t to indicate which
 * payload union member is valid.
 */
typedef enum {
    MSG_SENSOR,  /**< Sensor reading message (sensor_readings_t). */
    MSG_METRICS,  /**< Metrics/ health check message (system_metrics_t). */
    MSG_OTA      /**< OTA status message (ota_status_t). */
} mqtt_msg_type_t;

/**
 * @brief Item placed on the MQTT publish queue.
 *
 * Encapsulates one of several payload types along with a discriminator
 * indicating which union member is active.
 */
typedef struct {
    mqtt_msg_type_t type;  /**< Discriminator for which payload is valid. */

    /**
     * @brief Union of possible message payloads.
     *
     * Only the member matching @c type is valid.
     */
    union {
        sensor_readings_t sensor; /**< Payload for MSG_SENSOR. */
        system_metrics_t  health; /**< Payload for MSG_HEALTH. */
        ota_status_t   ota;    /**< Payload for MSG_OTA. */
    } data;
} mqtt_queue_item_t;

/**
 * @brief Publish request structure
 *
 * Contains all parameters needed to enqueue an MQTT publish call.
 */
typedef struct 
{
    const char *topic;
    const char *payload;
    int         len;
    int         qos;
    int         retain;
} mqtt_publish_req_t;

/**
 * @brief MQTT processing task
 *
 * Initializes MQTT client, registers event handler, and continuously
 * processes publish requests from the queue. Waits for connection before
 * publishing.
 *
 * @param pvParameters Pointer to application context (app_ctx_t *)
 */
void mqtt_task(void *pvParameters);

#endif /* COMPONENTS_DRIVERS_MQTT_INCLUDE_MQTT_H_ */
