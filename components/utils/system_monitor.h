#pragma once

#include <stdint.h>

#include "esp_log.h"
#include "esp_timer.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "utils.h"

/**
 * @brief Health‐check data structure.
 *
 * Contains basic device health metrics for monitoring.
 */
typedef struct {
    uint32_t uptime_ms;           /**< Milliseconds since device boot. */
    uint32_t min_free_heap;       /**< The minimum free heap observed. */
    uint32_t free_heap;         /**< Bytes of free heap memory. */
    uint32_t stack_watermark;  /**< Minimum free stack space (high watermark). */
} system_metrics_t;

// Gathers one snapshot
system_metrics_t get_system_metrics(void);

// Logs one snapshot
void log_system_metrics(const system_metrics_t *m);

// Your task just becomes a loop around those two:
void system_monitor_task(void *pvParameters);
