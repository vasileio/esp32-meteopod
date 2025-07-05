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

    /* SHT31 */
    err = sht31_init(ctx.i2c_bus,
                     SHT31_I2C_ADDR_DEFAULT,
                     I2C_SPEED_STANDARD_MODE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "SHT31 init failed: %s", esp_err_to_name(err));
    }

    /* Rain sensor */
    err = DFRobot_rainfall_sensor_init(&ctx.rain_sensor,
                                       ctx.i2c_bus,
                                       DFROBOT_RAINFALL_SENSOR_I2C_ADDR_DEFAULT,
                                       I2C_SPEED_STANDARD_MODE);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Rain init failed: %s", esp_err_to_name(err));
    } else {
        if (DFRobot_rainfall_sensor_begin(&ctx.rain_sensor) != ESP_OK) {
            ESP_LOGE(TAG, "Rain probe failed");
        }
    }

    /* Queues & mutex */
    ctx.commandQueue    = xQueueCreate(10, sizeof(uart_event_t));
    ctx.sensorDataMutex = xSemaphoreCreateMutex();

    /* Spawn tasks */
    xTaskCreate(watchdog_task,        "watchdog",  2048, &ctx, 6, &ctx.watchdogTaskHandle);
    xTaskCreate(blink_task,           "blink",     3072, &ctx, 1, &ctx.blinkTaskHandle);
    xTaskCreate(system_monitor_task,  "monitor",   4096, &ctx, 1, &ctx.monitorTaskHandle);
}
