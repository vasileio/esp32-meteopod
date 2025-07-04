#include "sensors.h"

extern SemaphoreHandle_t sensorDataMutex;
extern sensor_data_t shared_sensor_data;

static const char *TAG = "sensors";

esp_err_t init_sensors(void)
{
    esp_err_t err;
    i2c_master_bus_handle_t bus_handle;

    // 1) Get existing I2C bus handle, or initialize if not yet created
    err = i2c_master_get_bus_handle(I2C_PORT, &bus_handle);
    if (err == ESP_ERR_INVALID_STATE) {
        // bus wasn’t initialized yet, so do it now
        if ((err = i2c_init(&bus_handle)) != ESP_OK) {
            ESP_LOGE(TAG, "I2C master init failed: %s", esp_err_to_name(err));
            return err;
        }
    } else if (err != ESP_OK) {
        // some other failure retrieving the handle
        ESP_LOGE(TAG, "Failed to get I2C bus handle: %s", esp_err_to_name(err));
        return err;
    }

    // 2) Register our SHT31 on that bus
    if ((err = sht31_init(bus_handle, 0x44, SHT31_I2C_SPEED)) != ESP_OK) {
        ESP_LOGE(TAG, "SHT31 init failed: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

void sensor_temp_hum_task(void *pvParameters)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY)) {
            shared_sensor_data.temp_C = rand() % 30;
            shared_sensor_data.rel_hum = rand() % 100;
            xSemaphoreGive(sensorDataMutex);
        } else {
            ESP_LOGW("TEMP", "Mutex unavailable");
        }
    }
}

void sensor_pm_task(void *pvParameters)
{
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        if (xSemaphoreTake(sensorDataMutex, portMAX_DELAY)) {
            shared_sensor_data.pm25 = rand() % 50;
            shared_sensor_data.pm10 = rand() % 100;
            xSemaphoreGive(sensorDataMutex);
        } else {
            ESP_LOGW("PM", "Mutex unavailable");
        }
    }
}

void update_pm_values(sensor_data_t *data)
{
    data->pm25 = rand() % 50;
    data->pm10 = rand() % 100;
}
