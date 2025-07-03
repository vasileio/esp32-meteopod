#include "system_monitor.h"

static const char *TAG = "SYSTEM_MONITOR";

system_metrics_t get_system_metrics(void)
{
    system_metrics_t metrics;

    metrics.free_heap      = esp_get_free_heap_size();
    metrics.min_free_heap  = esp_get_minimum_free_heap_size();
    metrics.uptime_ms      = esp_timer_get_time() / 1000;
    metrics.stack_watermark = uxTaskGetStackHighWaterMark(NULL);
    return metrics;
}

void log_system_metrics(const system_metrics_t *metrics)
{
    ESP_LOGI(TAG, "Free Heap: %d bytes, Min Free Heap: %d bytes",
             metrics->free_heap, metrics->min_free_heap);
    ESP_LOGI(TAG, "Uptime: %lld ms", metrics->uptime_ms);
    ESP_LOGI(TAG, "Current Task Stack High Water Mark: %u bytes",
             metrics->stack_watermark);
}

void system_monitor_task(void *pvParameters)
{
    (void)pvParameters;

    system_metrics_t metrics;

    while (1) {

        metrics = get_system_metrics();
        log_system_metrics(&metrics);
        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
