#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

#include "esp_timer.h"

#include "driver/i2c_master.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

#include "dfrobot_rainfall_sensor.h"
#include "sht31.h"
#include "mqtt.h"


/* Raw MAC is 6 bytes */
#define MAC_RAW_LEN       6
/* 6 bytes × 2 hex digits + NUL */
#define MAC_STR_LEN       (2 * MAC_RAW_LEN + 1)
/* "meteopod/" is 9 chars, plus MAC_STR_LEN */
#define TOPIC_PREFIX_LEN  (9 + MAC_STR_LEN)

typedef struct {
    /* Buses & peripherals */
    i2c_master_bus_handle_t     i2c_bus;
    DFRobot_rainfall_sensor_t   rain_sensor;

    /* Queues & mutexes */
    QueueHandle_t               sensorDataQueue;
    QueueHandle_t               commandQueue;
    SemaphoreHandle_t           sensorDataMutex;

    /* Tasks */
    TaskHandle_t                sensorTaskHandle;
    TaskHandle_t                loggingTaskHandle;
    TaskHandle_t                commandTaskHandle;
    TaskHandle_t                watchdogTaskHandle;
    TaskHandle_t                blinkTaskHandle;
    TaskHandle_t                monitorTaskHandle;

    /* Timer */
    esp_timer_handle_t          periodic_timer;

    /* MQTT */
    esp_mqtt_client_handle_t    mqtt_client;
    TaskHandle_t                mqttTaskHandle;
    QueueHandle_t               mqttPublishQueue;
    EventGroupHandle_t          mqttEventGroup;
    uint8_t                     device_mac[MAC_RAW_LEN];         
    char                        device_mac_str[MAC_STR_LEN];     
    char                        topic_prefix[TOPIC_PREFIX_LEN];
    char                        ota_cmd_topic[64];
    char                        ota_status_topic[64];
} app_ctx_t;

#endif  // APP_CONTEXT_H
