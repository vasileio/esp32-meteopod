#pragma once

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"

#include "i2c.h"
#include "sht31.h"


typedef struct {
    int temp_C;
    int rel_hum;
    int pm25;
    int pm10;
} sensor_data_t;

/* Initialises all sensors.
 * Returns first non-OK error, or ESP_OK. */
esp_err_t init_sensors(void);

void sensor_temp_hum_task(void *pvParameters);
void sensor_pm_task(void *pvParameters);
void update_pm_values(sensor_data_t *data);
