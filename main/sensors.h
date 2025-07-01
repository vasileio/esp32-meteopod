#pragma once

#include "freertos/FreeRTOS.h"

typedef struct {
    int temp_C;
    int rel_hum;
    int pm25;
    int pm10;
} sensor_data_t;

void sensor_temp_hum_task(void *pvParameters);
void sensor_pm_task(void *pvParameters);
void update_pm_values(sensor_data_t *data);
