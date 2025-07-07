#pragma once

#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include "app_context.h"
#include "utils.h"

// A simple POD to hold all of your stats
typedef struct {
    size_t free_heap;
    size_t min_free_heap;
    int64_t uptime_ms;
    UBaseType_t stack_watermark;
} system_metrics_t;

// Gathers one snapshot
system_metrics_t get_system_metrics(void);

// Logs one snapshot
void log_system_metrics(const system_metrics_t *m);

// Your task just becomes a loop around those two:
void system_monitor_task(void *pvParameters);
