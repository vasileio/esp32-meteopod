/**
 * @file watchdog.c
 * @brief Enhanced watchdog system implementation for monitoring critical tasks and system health.
 */

#include "watchdog.h"
#include "app_context.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_task_wdt.h"
#include "esp_timer.h"
#include <string.h>

static const char *TAG = "WATCHDOG";

#define MAX_MONITORED_TASKS 8

/**
 * @brief Structure to track monitored tasks
 */
typedef struct {
    TaskHandle_t handle;
    char name[configMAX_TASK_NAME_LEN];
    int64_t last_feed_time;
    bool active;
} monitored_task_t;

/**
 * @brief Watchdog internal state
 */
typedef struct {
    watchdog_config_t config;
    monitored_task_t monitored_tasks[MAX_MONITORED_TASKS];
    uint8_t task_count;
    bool initialized;
} watchdog_state_t;

static watchdog_state_t g_watchdog_state = {0};

esp_err_t watchdog_init(const watchdog_config_t *config)
{
    if (g_watchdog_state.initialized) {
        ESP_LOGW(TAG, "Watchdog already initialized");
        return ESP_OK;
    }

    if (config != NULL) {
        g_watchdog_state.config = *config;
    } else {
        watchdog_config_t default_config = WATCHDOG_CONFIG_DEFAULT();
        g_watchdog_state.config = default_config;
    }

    g_watchdog_state.task_count = 0;
    g_watchdog_state.initialized = true;

    ESP_LOGI(TAG, "Watchdog initialized - check period: %lu ms, task timeout: %lu ms",
             g_watchdog_state.config.check_period_ms,
             g_watchdog_state.config.task_timeout_ms);

    return ESP_OK;
}

esp_err_t watchdog_register_task(TaskHandle_t task_handle, const char *task_name)
{
    if (!g_watchdog_state.initialized) {
        ESP_LOGE(TAG, "Watchdog not initialized");
        return ESP_ERR_INVALID_STATE;
    }

    if (task_handle == NULL || task_name == NULL) {
        ESP_LOGE(TAG, "Invalid parameters");
        return ESP_ERR_INVALID_ARG;
    }

    if (g_watchdog_state.task_count >= MAX_MONITORED_TASKS) {
        ESP_LOGE(TAG, "Maximum monitored tasks limit reached");
        return ESP_ERR_NO_MEM;
    }

    monitored_task_t *task = &g_watchdog_state.monitored_tasks[g_watchdog_state.task_count];
    task->handle = task_handle;
    strncpy(task->name, task_name, sizeof(task->name) - 1);
    task->name[sizeof(task->name) - 1] = '\0';
    task->last_feed_time = esp_timer_get_time();
    task->active = true;

    g_watchdog_state.task_count++;
    
    ESP_LOGI(TAG, "Registered task for monitoring: %s", task_name);
    return ESP_OK;
}

void watchdog_feed(TaskHandle_t task_handle)
{
    if (!g_watchdog_state.initialized || task_handle == NULL) {
        return;
    }

    for (uint8_t i = 0; i < g_watchdog_state.task_count; i++) {
        if (g_watchdog_state.monitored_tasks[i].handle == task_handle) {
            g_watchdog_state.monitored_tasks[i].last_feed_time = esp_timer_get_time();
            break;
        }
    }
}

static void check_monitored_tasks(void)
{
    if (!g_watchdog_state.config.monitor_tasks) {
        return;
    }

    int64_t current_time = esp_timer_get_time();
    int64_t timeout_us = g_watchdog_state.config.task_timeout_ms * 1000;

    for (uint8_t i = 0; i < g_watchdog_state.task_count; i++) {
        monitored_task_t *task = &g_watchdog_state.monitored_tasks[i];
        
        if (!task->active) {
            continue;
        }

        int64_t time_since_feed = current_time - task->last_feed_time;
        
        if (time_since_feed > timeout_us) {
            ESP_LOGE(TAG, "Task '%s' has not responded for %lld ms (timeout: %lu ms)",
                     task->name,
                     time_since_feed / 1000,
                     g_watchdog_state.config.task_timeout_ms);
            
            // Check if task still exists
            eTaskState task_state = eTaskGetState(task->handle);
            if (task_state == eDeleted) {
                ESP_LOGW(TAG, "Task '%s' has been deleted", task->name);
                task->active = false;
            } else {
                ESP_LOGW(TAG, "Task '%s' state: %d", task->name, task_state);
            }
        }
    }
}

static void check_heap_health(void)
{
    if (!g_watchdog_state.config.monitor_heap) {
        return;
    }

    uint32_t free_heap = esp_get_free_heap_size();
    uint32_t min_free_heap = esp_get_minimum_free_heap_size();
    
    if (free_heap < g_watchdog_state.config.min_free_heap_threshold) {
        ESP_LOGW(TAG, "Low heap warning: %lu bytes free (threshold: %lu bytes)",
                 free_heap, g_watchdog_state.config.min_free_heap_threshold);
    }

    ESP_LOGD(TAG, "Heap status: %lu bytes free, %lu bytes minimum ever free",
             free_heap, min_free_heap);
}

void watchdog_task(void *pvParameters)
{
    (void)pvParameters;  // Parameter not currently used
    
    if (!g_watchdog_state.initialized) {
        ESP_LOGE(TAG, "Watchdog not initialized, using default config");
        watchdog_init(NULL);
    }

    ESP_LOGI(TAG, "Watchdog task started");

    // Add this task to the ESP task watchdog
    esp_task_wdt_add(NULL);

    uint32_t check_count = 0;
    
    while (1) {
        // Reset ESP task watchdog
        esp_task_wdt_reset();

        // Check monitored tasks
        check_monitored_tasks();

        // Check heap health
        check_heap_health();

        // Log periodic status
        if (++check_count % 10 == 0) {
            ESP_LOGI(TAG, "Watchdog heartbeat - monitoring %u tasks", 
                     g_watchdog_state.task_count);
        }

        vTaskDelay(pdMS_TO_TICKS(g_watchdog_state.config.check_period_ms));
    }
}