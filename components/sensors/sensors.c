/**
 * @file sensors.c
 * @brief Sensor initialization and data acquisition implementation.
 */

#include "sensors.h"

static const char *TAG = "SENSORS";

static bme280_handle_t bme_handle;  // static so it lives forever

/**
 * @brief Initialize BME280 sensor component
 */
static esp_err_t bme280_component_init(void *pvParameters) {

    // app_ctx_t *ctx = (app_ctx_t *)pvParameters;  

    // Configure BME280 for weather monitoring

    bme280_config_t config = {
        .osrs_t       = BME280_OSRS_X2,      // temp ×2
        .osrs_p       = BME280_OSRS_X16,     // press ×16
        .osrs_h       = BME280_OSRS_X1,      // hum ×1
        .filter       = BME280_FILTER_4,     // IIR=4
        .standby_time = BME280_STANDBY_1000, // 1000 ms
        .mode         = BME280_MODE_FORCED,  // continuous
    };
    
    esp_err_t ret = bme280_init(&bme_handle, I2C_PORT, BME280_I2C_ADDR, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BME280 initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BME280 sensor initialized successfully");
    return ESP_OK;
}

void sensors_init(void *pvParameters)
{
    esp_err_t ret;

    // app_ctx_t *ctx = (app_ctx_t *)pvParameters;  

    /* Initialize BME280 temperature/humidity/pressure sensor */
    ret = bme280_component_init(pvParameters);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BME280 init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "BME280 initialized");
    }


    /* TODO: Add initialization for additional sensors here */
}

void sensors_task(void *pvParameters)
{
    /* Ensure sensors are initialized */
    sensors_init(pvParameters);

    while (1) {

        bme280_data_t data;
        esp_err_t err;

        // just call bme280_read_data() to get the latest values.
        bme280_trigger_measurement(&bme_handle);

        bool ready = false;
        do {
            vTaskDelay(pdMS_TO_TICKS(10));
            bme280_is_meas_ready(&bme_handle, &ready);
        } while (!ready);

        err = bme280_read_data(&bme_handle, &data);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Failed to read BME280: %s", esp_err_to_name(err));
            return;
        }

        ESP_LOGI(TAG, "Temperature: %.2f °C", data.temperature);
        ESP_LOGI(TAG, "Pressure:    %.2f hPa", data.pressure);
        ESP_LOGI(TAG, "Humidity:    %.2f %%RH", data.humidity);
        /* TODO: Read additional sensors here */

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}
