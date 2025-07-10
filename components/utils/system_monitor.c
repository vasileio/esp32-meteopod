#include "system_monitor.h"
#include "app_context.h"
#include "mqtt.h"


static const char *TAG = "SYSTEM_MONITOR";

system_metrics_t get_system_metrics(void)
{
    system_metrics_t metrics;

    metrics.free_heap       = esp_get_free_heap_size();
    metrics.min_free_heap   = esp_get_minimum_free_heap_size();
    metrics.uptime_ms       = esp_timer_get_time() / 1000;
    metrics.stack_watermark = uxTaskGetStackHighWaterMark(NULL);

    return metrics;
}

void log_system_metrics(const system_metrics_t *metrics)
{
    ESP_LOGI(TAG, "Free Heap: %lu bytes, Min Free Heap: %lu bytes",
             metrics->free_heap, metrics->min_free_heap);
    ESP_LOGI(TAG, "Uptime: %lu ms", metrics->uptime_ms);
    ESP_LOGI(TAG, "Task Stack High Water Mark: %lu bytes",
             metrics->stack_watermark);
}

void system_monitor_task(void *pvParameters)
{
    app_ctx_t *ctx =    pvParameters;
    mqtt_queue_item_t   item;
    system_metrics_t    metrics;

    item.type = MSG_METRICS;


    while (1) 
    {
        // 1) Collect metrics
        metrics = get_system_metrics();

        // 2) Log locally
        log_system_metrics(&metrics);

        // 3) Copy into our queue item
        item.data.metrics = metrics;

        // 4) Enqueue for the MQTT task to format & publish
        if (xQueueSend(ctx->mqttPublishQueue, &item, portMAX_DELAY) != pdTRUE) {
            ESP_LOGW(TAG, "MQTT metrics queue full, dropping heartbeat");
        }

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
