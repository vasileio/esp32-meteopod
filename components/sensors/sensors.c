/**
 * @file sensors.c
 * @brief Sensor initialization and data acquisition implementation.
 */

#include "sensors.h"
#include "app_context.h"
#include "mqtt.h"
#include "bh1750.h"

static const char *TAG = "SENSORS";

static bme280_handle_t bme_handle;  // static so it lives forever

/**
 * @brief initialise SHT31 sensor
 */
static esp_err_t sht31_component_init(void *pvParameters) {

    app_ctx_t *ctx = (app_ctx_t *)pvParameters;  
    
    esp_err_t ret = sht31_init(&ctx->sh31_sensor, I2C_PORT, SHT31_ADDR_DEFAULT);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT31 initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "SHT31 sensor initialised successfully");
    return ESP_OK;
}

/**
 * @brief initialise BME280 sensor
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
        .mode         = BME280_MODE_FORCED,  // on-demand
    };
    
    esp_err_t ret = bme280_init(&bme_handle, I2C_PORT, BME280_I2C_ADDR, &config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BME280 initialization failed: %s", esp_err_to_name(ret));
        return ret;
    }
    
    ESP_LOGI(TAG, "BME280 sensor initialised successfully");
    return ESP_OK;
}

static esp_err_t read_bme280_measurement(bme280_handle_t *handle, bme280_data_t *out_data)
{
    TickType_t start_tick = xTaskGetTickCount();
    const TickType_t timeout_ticks = pdMS_TO_TICKS(BME280_READY_TIMEOUT_MS);
    bool ready = false;
    esp_err_t err;

    bme280_trigger_measurement(handle);

    // Wait for measurement to be ready (with timeout)
    do {
        err = bme280_is_meas_ready(handle, &ready);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "BME280 readiness check failed: %s", esp_err_to_name(err));
            return err;
        }

        if (ready) {
            break;
        }

        if ((xTaskGetTickCount() - start_tick) > timeout_ticks) {
            ESP_LOGW(TAG, "BME280 not ready after %d ms", BME280_READY_TIMEOUT_MS);
            return ESP_ERR_TIMEOUT;
        }

        vTaskDelay(pdMS_TO_TICKS(BME280_POLL_INTERVAL_MS));
    } while (1);

    // Now actually read the data
    err = bme280_read_data(handle, out_data);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to read BME280 data: %s", esp_err_to_name(err));
    }
    return err;
}


void sensors_init(void *pvParameters)
{
    esp_err_t ret;

    app_ctx_t *ctx = (app_ctx_t *)pvParameters;  

    /* initialise BME280 temperature/humidity/pressure sensor */
    ret = bme280_component_init(pvParameters);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "BME280 init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "BME280 initialised");
    }

    ret = sht31_component_init(pvParameters);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SHT31 init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "SHT31 initialised");
    }

    ret = wind_sensor_init();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Wind sensor init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Wind sensor initialised");
    }

    ret = bh1750_init(&ctx->bh1750_sensor, I2C_PORT, BH1750_I2C_ADDR);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Light sensor init failed: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "Light sensor initialised");
        bh1750_set_mode(&ctx->bh1750_sensor, CONTINUOUS_HIGH_RES_MODE);
    }
    



    /* TODO: Add initialization for additional sensors here */
}

void sensors_task(void *pvParameters)
{
    app_ctx_t *ctx =        (app_ctx_t *)pvParameters;
    bme280_data_t           bme280_data;
    sht31_data_t            sht31_data;
    wind_data_t             wind_data;
    float                   light_lux;
    mqtt_queue_item_t       item;
    esp_err_t               err;

    item.type = MSG_SENSOR;

    /* Ensure sensors are initialised */
    sensors_init(pvParameters);

    while (1) {
        /* BME280 */

        // Try to read — on error, just warn and continue
        err = read_bme280_measurement(&bme_handle, &bme280_data);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "BME280 read error (continuing): %s", esp_err_to_name(err));
        } else {
            /* Acquire mutex before updating shared context */
            if (xSemaphoreTake(ctx->sensorDataMutex, portMAX_DELAY) == pdTRUE)
            {
                ctx->sensor_readings.bme280_readings.temperature = bme280_data.temperature;
                ctx->sensor_readings.bme280_readings.humidity   = bme280_data.humidity;
                ctx->sensor_readings.bme280_readings.pressure   = bme280_data.pressure;
                xSemaphoreGive(ctx->sensorDataMutex);
                
                /* Copy into our queue item */
                item.data.sensor.bme280_readings = bme280_data;

                /* Log from the local buffer (no need to hold the lock while logging) */
                ESP_LOGI(TAG, "[BME280] Temperature: %.2f °C", bme280_data.temperature);
                ESP_LOGI(TAG, "[BME280] Humidity:    %.2f %%RH", bme280_data.humidity);
                ESP_LOGI(TAG, "[BME280] Pressure:    %.2f hPa", bme280_data.pressure);
            } else {
                ESP_LOGW(TAG, "Failed to take sensorDataMutex");
            }

        }

        err = sht31_read_temp_hum(&ctx->sh31_sensor, &sht31_data.temperature, &sht31_data.humidity);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "SHT31 read error (continuing): %s", esp_err_to_name(err));
        } else {

            /* Acquire mutex before updating shared context */
            if (xSemaphoreTake(ctx->sensorDataMutex, portMAX_DELAY) == pdTRUE) 
            {
                ctx->sensor_readings.sht31_readings.temperature = sht31_data.temperature;
                ctx->sensor_readings.sht31_readings.humidity   =    sht31_data.humidity;
                xSemaphoreGive(ctx->sensorDataMutex);
                
                /* Copy into our queue item */
                item.data.sensor.sht31_readings = sht31_data;

                ESP_LOGI(TAG, "[SHT31] Temperature: %.2f °C", sht31_data.temperature);
                ESP_LOGI(TAG, "[SHT31] Humidity:    %.2f %%RH", sht31_data.humidity);
            }
    }

        err = wind_sensor_read(&wind_data);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "SHT31 read error (continuing): %s", esp_err_to_name(err));
        } else {

            /* Acquire mutex before updating shared context */
            if (xSemaphoreTake(ctx->sensorDataMutex, portMAX_DELAY) == pdTRUE)
            {
                ctx->sensor_readings.wind_readings.direction = wind_data.direction;
                ctx->sensor_readings.wind_readings.speed = wind_data.speed;
                xSemaphoreGive(ctx->sensorDataMutex);

                /* Copy into our queue item */
                item.data.sensor.wind_readings = wind_data;

                ESP_LOGI(TAG, "[WIND] Direction: %s", wind_data.direction);
                ESP_LOGI(TAG, "[WIND] Speed: %.1f m/s", wind_data.speed);
            }
    }

        err = bh1750_read_light(&ctx->bh1750_sensor, &light_lux);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "BH1750 read error (continuing): %s", esp_err_to_name(err));
        } else {

            /* Acquire mutex before updating shared context */
            if (xSemaphoreTake(ctx->sensorDataMutex, portMAX_DELAY) == pdTRUE)
            {
                ctx->sensor_readings.light_lux = light_lux;
                xSemaphoreGive(ctx->sensorDataMutex);

                /* Copy into our queue item */
                item.data.sensor.light_lux = light_lux;

                ESP_LOGI(TAG, "[LIGHT] Ambient light: %.0f", light_lux);
            }
    }

    /* Enqueue for the MQTT task to format & publish */
    if (xQueueSend(ctx->mqttPublishQueue, &item, portMAX_DELAY) != pdTRUE) {
        ESP_LOGW(TAG, "MQTT metrics queue full, dropping heartbeat");
    }
        /* TODO: Read additional sensors here */

        vTaskDelay(pdMS_TO_TICKS(SENSOR_READ_INTERVAL_MS));
    }
}
