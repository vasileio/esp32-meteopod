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

typedef struct {
    /* Buses & peripherals */
    i2c_master_bus_handle_t   i2c_bus;
    DFRobot_rainfall_sensor_t rain_sensor;

    /* Queues & mutexes */
    QueueHandle_t             sensorDataQueue;
    QueueHandle_t             commandQueue;
    SemaphoreHandle_t         sensorDataMutex;

    /* Tasks */
    TaskHandle_t              sensorTaskHandle;
    TaskHandle_t              loggingTaskHandle;
    TaskHandle_t              commandTaskHandle;
    TaskHandle_t              watchdogTaskHandle;
    TaskHandle_t              blinkTaskHandle;
    TaskHandle_t              monitorTaskHandle;

    /* Timer */
    esp_timer_handle_t        periodic_timer;
} app_ctx_t;

#endif  // APP_CONTEXT_H
