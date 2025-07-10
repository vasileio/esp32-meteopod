#include "system_monitor.h"
#include "app_context.h"
#include "mqtt.h"
#include <esp_wifi.h>
#include <esp_netif.h>
#include <lwip/ip4_addr.h>

static const char *TAG = "SYSTEM_MONITOR";

system_metrics_t get_system_metrics(void)
{
    system_metrics_t metrics;

    metrics.free_heap       = esp_get_free_heap_size();
    metrics.min_free_heap   = esp_get_minimum_free_heap_size();
    metrics.uptime_ms       = esp_timer_get_time() / 1000;
    metrics.stack_watermark = uxTaskGetStackHighWaterMark(NULL);

    /* Wi-Fi RSSI */
    wifi_ap_record_t ap_info;
    if (esp_wifi_sta_get_ap_info(&ap_info) == ESP_OK) {
        metrics.wifi_rssi = ap_info.rssi;
    } else {
        metrics.wifi_rssi = 0;
    }

    /* IP Address */
    esp_netif_t *sta_netif = esp_netif_get_handle_from_ifkey("WIFI_STA_DEF");
    if (sta_netif) {
        esp_netif_ip_info_t ip_info;
        if (esp_netif_get_ip_info(sta_netif, &ip_info) == ESP_OK) {
            snprintf(metrics.ip_address, sizeof(metrics.ip_address),
                     IPSTR, IP2STR(&ip_info.ip));
        } else {
            // safe fallback:
            snprintf(metrics.ip_address, sizeof(metrics.ip_address),
                     "0.0.0.0");
        }
    } else {
        snprintf(metrics.ip_address, sizeof(metrics.ip_address),
                 "0.0.0.0");
    }

    return metrics;
}

void log_system_metrics(const system_metrics_t *metrics)
{
    ESP_LOGI(TAG, "Wi-Fi RSSI: %ld dBm",
             metrics->wifi_rssi);
    ESP_LOGI(TAG, "IP address: %s",
             metrics->ip_address);
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
