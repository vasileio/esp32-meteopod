#ifndef COMPONENTS_DRIVERS_MQTT_INCLUDE_MQTT_H_
#define COMPONENTS_DRIVERS_MQTT_INCLUDE_MQTT_H_

#include <inttypes.h>
#include "esp_log.h"
#include "mqtt_client.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/event_groups.h"
#include "freertos/queue.h"
#include "app_context.h"
#include "utils.h"
#include "ota_update.h"
#include <string.h>

#ifndef MIN
#define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif

#define MQTT_CONNECTED_BIT BIT0

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
