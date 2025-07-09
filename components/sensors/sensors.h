/* sensors.h */
/**
 * @file sensors.h
 * @brief Sensor initialization and data acquisition component.
 *
 * This component initializes sensors and provides a FreeRTOS task
 * to periodically read sensor values and output them via ESP_LOG.
 * Future sensors can be added by extending sensors_init and sensors_task.
 */
#ifndef COMPONENTS_SENSORS_SENSORS_H_
#define COMPONENTS_SENSORS_SENSORS_H_

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_err.h"
#include "sht31.h"
#include "bme280.h"
#include "app_context.h"
#include "i2c.h"


#ifdef __cplusplus
extern "C" {
#endif


/**
 * @brief Delay between sensor readings in milliseconds.
 */
#define SENSOR_READ_INTERVAL_MS 5000

/**
 * @brief Initialize all configured sensors.
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

#ifdef __cplusplus
}
#endif

#endif /* COMPONENTS_SENSORS_SENSORS_H_ */
