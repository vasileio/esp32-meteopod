/**
 * @file sensors.c
 * @brief Sensor initialization and data acquisition implementation.
 */

#include "sensors.h"

static const char *TAG = "SENSORS";

void sensors_init(void *pvParameters)
{
    esp_err_t ret;

    app_ctx_t *ctx = (app_ctx_t *)pvParameters;  

    /* Initialize SHT31 temperature/humidity sensor */
    ret = sht31_init(ctx->i2c_bus, SHT31_I2C_ADDR_DEFAULT, SHT31_I2C_SPEED);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT31 init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SHT31 initialized");
    }

    /* TODO: Add initialization for additional sensors here */
}

void sensors_task(void *pvParameters)
{
    /* Ensure sensors are initialized */
    sensors_init(pvParameters);

    while (1) {
        /* Read SHT31 sensor */
        float temperature = 0.0f;
        float humidity    = 0.0f;
        esp_err_t ret = sht31_read(&temperature, &humidity);

        if (ret == ESP_OK) {
            ESP_LOGI(TAG,
                     "Temperature: %.2f °C, Humidity: %.2f %%",
                     temperature, humidity);
        } else {
            ESP_LOGE(TAG, "SHT31 read failed: %s", esp_err_to_name(ret));
        }

        /* TODO: Read additional sensors here */

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}
