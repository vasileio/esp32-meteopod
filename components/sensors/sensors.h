/* sensors.h */
/**
 * @file sensors.h
 * @brief Sensor initialization and data acquisition component.
 *
 * This component initialises sensors and provides a FreeRTOS task
 * to periodically read sensor values and output them via ESP_LOG.
 * Future sensors can be added by extending sensors_init and sensors_task.
 */
#pragma once
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sht31.h"
#include "bme280.h"
#include "i2c.h"
#include "wind_sensor.h"

#define BME280_READY_TIMEOUT_MS   1000
#define BME280_POLL_INTERVAL_MS     10

/**
 * @brief Delay between sensor readings in milliseconds.
 */
#define SENSOR_READ_INTERVAL_MS 5000

/**
 * @brief Sensors readings structure
 *
 * Contains all readings acquired by the various sensors.
 */
typedef struct 
{
    bme280_data_t   bme280_readings;
    sht31_data_t    sht31_readings;
    wind_data_t     wind_readings;
    float           light_lux;
} sensor_readings_t;

/**
 * @brief initialise all configured sensors.
 *
 * This calls underlying driver initializations (e.g., SHT31).
 * Ensure i2c_init() has been called prior to this.
 */
void sensors_init(void *pvParameters);

/**
 * @brief FreeRTOS task for sensor data acquisition.
 *
 * This task reads sensor data at fixed intervals and logs the results.
 * It is the caller's responsibility to create this task from main.
 *
 * @param pvParameters Task parameters (unused, set to NULL).
 */
void sensors_task(void *pvParameters);
