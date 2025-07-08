#include "include/main.h"
#include "app_context.h"

#define TAG                  "MAIN"
#define BLINK_GPIO           2
#define WATCHDOG_TIMEOUT_SEC 5

static void watchdog_task(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    while(1)
    {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

static void blink_task(void *pvParameters)
{
    (void)pvParameters;
    esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);
    for (;;) {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));
        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    static app_ctx_t ctx;  // all shared state

    ESP_LOGI(TAG, "System booting…");

    init_uart(&ctx);
    wifi_init_sta();

    /* I²C bus init */
    esp_err_t err = i2c_init(&ctx.i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return;
    }
    /* 1) Read raw MAC */
    esp_read_mac(ctx.device_mac, ESP_MAC_WIFI_STA);

    /* 2) Build MAC string (uppercase, no separators) */
    snprintf(ctx.device_mac_str,
             MAC_STR_LEN,
             "%02X%02X%02X%02X%02X%02X",
             ctx.device_mac[0],
             ctx.device_mac[1],
             ctx.device_mac[2],
             ctx.device_mac[3],
             ctx.device_mac[4],
             ctx.device_mac[5]);

    /* 3) Build topic prefix "meteopod/<MAC>" */
    snprintf(ctx.topic_prefix,
             TOPIC_PREFIX_LEN,
             "meteopod/%s",
             ctx.device_mac_str);

    snprintf(ctx.ota_cmd_topic,
            sizeof(ctx.ota_cmd_topic),
            "%s/ota/update",
            ctx.topic_prefix);

    // Build OTA status topic: meteopod/<mac>/ota/status
    snprintf(ctx.ota_status_topic,
            sizeof(ctx.ota_status_topic),
            "%s/ota/status",
            ctx.topic_prefix);

    /* 4) Print them out */
    ESP_LOGI(TAG, "Device MAC: %s",        ctx.device_mac_str);
    ESP_LOGI(TAG, "MQTT topic prefix: %s", ctx.topic_prefix);

    const esp_app_desc_t *desc = esp_app_get_description();
    ESP_LOGI(TAG, "Firmware version: %s", desc->version);

    /* Queues & mutex */
    ctx.commandQueue    = xQueueCreate(10, sizeof(uart_event_t));
    ctx.sensorDataMutex = xSemaphoreCreateMutex();

    /* Spawn tasks */
    xTaskCreate(watchdog_task,        "watchdog",  2048, &ctx, 6, &ctx.watchdogTaskHandle);
    xTaskCreate(blink_task,           "blink",     3072, &ctx, 1, &ctx.blinkTaskHandle);
    xTaskCreate(system_monitor_task,  "monitor",   4096, &ctx, 1, &ctx.monitorTaskHandle);
    xTaskCreate(mqtt_task,            "mqtt_task", 4096, &ctx, 5, &ctx.mqttTaskHandle);
}
