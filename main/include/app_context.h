#ifndef APP_CONTEXT_H
#define APP_CONTEXT_H

/* ESP headers */
#include "esp_timer.h"
#include "driver/i2c_master.h"

/* FreeRTOS headers */
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

/* Sensor headers */
#include "dfrobot_rainfall_sensor.h"
#include "sht31.h"
#include "bme280.h"
#include "bh1750.h"   
#include "dfrobot_as3935.h"
#include "wind_sensor.h"
#include "mpu6050.h"

/**
 * @brief Sensors readings structure
 *
 * Contains all readings acquired by the various sensors.
 */
typedef struct 
{
    bme280_data_t       bme280_readings;
    sht31_data_t        sht31_readings;
    wind_data_t         wind_readings;
    mpu6050_data_t      mpu6050_readings;
    float               light_lux;
    lightning_data_t    lightning_readings;
    bool                lightning_detected;  // Flag to indicate valid lightning data
} sensor_readings_t;

/* MQTT */
// #include "mqtt.h"
typedef struct esp_mqtt_client* esp_mqtt_client_handle_t;  /**< Forward‐declare the MQTT client handle */



/* Raw MAC is 6 bytes */
#define MAC_RAW_LEN       6
/* 6 bytes × 2 hex digits + NUL */
#define MAC_STR_LEN       (2 * MAC_RAW_LEN + 1)
/* "meteopod/" is 9 chars, plus MAC_STR_LEN */
#define TOPIC_PREFIX_LEN  (9 + MAC_STR_LEN)

#define TOPIC_LEN 64

typedef struct {
    /* Buses & peripherals */
    i2c_master_bus_handle_t     i2c_bus;
    DFRobot_rainfall_sensor_t   rain_sensor;
    sht31_handle_t              sh31_sensor;
    bh1750_handle_t             bh1750_sensor;
    mpu6050_handle_t            mpu6050_sensor;
    dfrobot_as3935_t            as3935_sensor;

    /* Queues */
    QueueHandle_t               sensorDataQueue;
    QueueHandle_t               commandQueue;
    QueueHandle_t               mqttPublishQueue;

    /* Mutexes */
    SemaphoreHandle_t           sensorDataMutex;

    /* Tasks */
    TaskHandle_t                sensorTaskHandle;
    TaskHandle_t                watchdogTaskHandle;
    TaskHandle_t                blinkTaskHandle;
    TaskHandle_t                monitorTaskHandle;
    TaskHandle_t                mqttTaskHandle;

    /* Readings */
    sensor_readings_t           sensor_readings;

    /* MQTT client */
    esp_mqtt_client_handle_t    mqtt_client;
    EventGroupHandle_t          mqttEventGroup;

    /* Identifiers & topics */
    uint8_t                     device_mac[MAC_RAW_LEN];
    char                        device_mac_str[MAC_STR_LEN];
    char                        topic_prefix[TOPIC_PREFIX_LEN];
    char                        sensor_topic[TOPIC_LEN];
    char                        metrics_topic[TOPIC_LEN];
    char                        ota_topic[TOPIC_LEN];
    char                        ota_cmd_topic[TOPIC_LEN];
    char                        ota_status_topic[TOPIC_LEN];

    /* Per‐sensor subtopics under <prefix>/sensor */
    char                        sensor_bme280_topic[TOPIC_LEN];
    char                        sensor_sht31_topic[TOPIC_LEN];
    char                        sensor_rainfall_topic[TOPIC_LEN];
    char                        sensor_wind_topic[TOPIC_LEN];
    char                        sensor_light_topic[TOPIC_LEN];
    char                        sensor_mpu6050_topic[TOPIC_LEN];
    char                        sensor_lightning_topic[TOPIC_LEN];
} app_ctx_t;


#endif  // APP_CONTEXT_H
