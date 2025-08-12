/**
 * @file watchdog.h
 * @brief Enhanced watchdog system for monitoring critical tasks and system health.
 */

#ifndef WATCHDOG_H
#define WATCHDOG_H

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Watchdog configuration structure
 */
typedef struct {
    uint32_t check_period_ms;          /**< Period between watchdog checks in milliseconds */
    uint32_t task_timeout_ms;          /**< Timeout for task monitoring in milliseconds */
    bool monitor_tasks;                /**< Enable task monitoring */
    bool monitor_heap;                 /**< Enable heap monitoring */
    uint32_t min_free_heap_threshold;  /**< Minimum free heap threshold in bytes */
} watchdog_config_t;

/**
 * @brief Default watchdog configuration
 */
#define WATCHDOG_CONFIG_DEFAULT() {        \
    .check_period_ms = 1000,               \
    .task_timeout_ms = 30000,              \
    .monitor_tasks = true,                 \
    .monitor_heap = true,                  \
    .min_free_heap_threshold = 8192        \
}

/**
 * @brief Initialize the watchdog system
 * 
 * @param config Watchdog configuration, NULL for default
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t watchdog_init(const watchdog_config_t *config);

/**
 * @brief Start the watchdog task
 * 
 * @param pvParameters Application context pointer
 */
void watchdog_task(void *pvParameters);

/**
 * @brief Register a task to be monitored by the watchdog
 * 
 * @param task_handle Handle of the task to monitor
 * @param task_name Name of the task for logging
 * @return ESP_OK on success, error code otherwise
 */
esp_err_t watchdog_register_task(TaskHandle_t task_handle, const char *task_name);

/**
 * @brief Feed the watchdog (reset the timer)
 * Called by monitored tasks to indicate they are alive
 * 
 * @param task_handle Handle of the calling task
 */
void watchdog_feed(TaskHandle_t task_handle);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_H */