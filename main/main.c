#include "include/main.h"
#include "app_context.h"

#define TAG                  "MAIN"
#define BLINK_GPIO           2
#define WATCHDOG_TIMEOUT_SEC 5


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
    static app_ctx_t ctx;

    ESP_LOGI(TAG, "System booting…");

    init_uart(&ctx);
    wifi_init_sta();

    /* I²C bus init */
    esp_err_t err = i2c_init(&ctx.i2c_bus);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C init failed: %s", esp_err_to_name(err));
        return;
    }

    const esp_app_desc_t *desc = esp_app_get_description();
    ESP_LOGI(TAG, "Firmware version: %s", desc->version);

    /* Queues & mutex */
    ctx.commandQueue    = xQueueCreate(10, sizeof(uart_event_t));

    ctx.mqttPublishQueue = xQueueCreate( 10, sizeof(mqtt_queue_item_t));
    configASSERT(ctx.mqttPublishQueue);
    
    ctx.mqttEventGroup = xEventGroupCreate();
    configASSERT(ctx.mqttEventGroup);

    ctx.sensorDataMutex = xSemaphoreCreateMutex();

    /* Initialize watchdog system */
    watchdog_config_t watchdog_config = WATCHDOG_CONFIG_DEFAULT();
    watchdog_config.check_period_ms = 2000;
    watchdog_config.task_timeout_ms = 120000;  // 2 minutes timeout - more reasonable for slow tasks
    esp_err_t watchdog_err = watchdog_init(&watchdog_config);
    if (watchdog_err != ESP_OK) {
        ESP_LOGE(TAG, "Watchdog init failed: %s", esp_err_to_name(watchdog_err));
    }

    /* Build MQTT topics */
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
    /* 3) Build the rest of the mqtt topics */
     mqtt_build_all_topics(&ctx);

    /* Create tasks */
    xTaskCreate(watchdog_task,        "watchdog",     STACK_WATCHDOG_WORDS, &ctx, PRIO_WATCHDOG, &ctx.watchdogTaskHandle);
    xTaskCreate(blink_task,           "blink",        STACK_BLINK_WORDS,    &ctx, PRIO_BLINK,    &ctx.blinkTaskHandle);
    xTaskCreate(system_monitor_task,  "monitor",      STACK_MONITOR_WORDS,  &ctx, PRIO_MONITOR,  &ctx.monitorTaskHandle);
    xTaskCreate(mqtt_task,            "mqtt_task",    STACK_MQTT_WORDS,     &ctx, PRIO_MQTT,     &ctx.mqttTaskHandle);
    xTaskCreate(sensors_task,         "sensors_task", STACK_SENSORS_WORDS,  &ctx, PRIO_SENSORS,  &ctx.sensorTaskHandle);

    /* Register critical tasks for monitoring */
    // Note: Task registration temporarily disabled due to dependency issues
    // watchdog still monitors system health and heap usage

}
