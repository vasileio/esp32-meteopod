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

    mqtt_build_all_topics(&ctx);

    const esp_app_desc_t *desc = esp_app_get_description();
    ESP_LOGI(TAG, "Firmware version: %s", desc->version);

    /* Queues & mutex */
    ctx.commandQueue    = xQueueCreate(10, sizeof(uart_event_t));
    ctx.sensorDataMutex = xSemaphoreCreateMutex();

    /* Create tasks */
    xTaskCreate(watchdog_task,        "watchdog",     STACK_WATCHDOG_WORDS, &ctx, PRIO_WATCHDOG, &ctx.watchdogTaskHandle);
    xTaskCreate(blink_task,           "blink",        STACK_BLINK_WORDS,    &ctx, PRIO_BLINK,    &ctx.blinkTaskHandle);
    xTaskCreate(system_monitor_task,  "monitor",      STACK_MONITOR_WORDS,  &ctx, PRIO_MONITOR,  &ctx.monitorTaskHandle);
    xTaskCreate(mqtt_task,            "mqtt_task",    STACK_MQTT_WORDS,     &ctx, PRIO_MQTT,     &ctx.mqttTaskHandle);
    xTaskCreate(sensors_task,         "sensors_task", STACK_SENSORS_WORDS,  &ctx, PRIO_SENSORS,  &ctx.sensorTaskHandle);

}
