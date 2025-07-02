#include "system_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "esp_timer.h"
#include "esp_system.h"
#include "esp_task_wdt.h"

static const char *TAG = "SYSTEM_MONITOR";

void system_monitor_task(void *pvParameters)
{
    while (1) {
        size_t free_heap = esp_get_free_heap_size();
        size_t min_free_heap = esp_get_minimum_free_heap_size();
        int64_t uptime_ms = esp_timer_get_time() / 1000;
        UBaseType_t stack_watermark = uxTaskGetStackHighWaterMark(NULL);

        ESP_LOGI(TAG, "Free Heap: %d bytes, Min Free Heap: %d bytes", free_heap, min_free_heap);
        ESP_LOGI(TAG, "Uptime: %lld ms", uptime_ms);
        ESP_LOGI(TAG, "Current Task Stack High Water Mark: %u bytes", stack_watermark);

        vTaskDelay(pdMS_TO_TICKS(10000));
    }
}
