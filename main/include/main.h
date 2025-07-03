// main/include/main.h
#pragma once
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "wifi.h"
#include "system_monitor.h"
#include "sensors.h"

void init_uart(void);
void periodic_sensor_cb(void *arg);

// expose the task handle so tests can stub/inspect it
extern TaskHandle_t sensorTaskHandle;