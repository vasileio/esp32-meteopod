#pragma once

#include "freertos/FreeRTOS.h"

typedef struct {
    int accel_x;
    int accel_y;
    int accel_z;
    int pm25;
    int pm10;
} sensor_data_t;

void sensor_accel_task(void *pvParameters);
void sensor_pm_task(void *pvParameters);
void update_pm_values(sensor_data_t *data);
