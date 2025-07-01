#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "sensors.h"

#define IMU_UART_NUM      UART_NUM_2
#define IMU_UART_TXD      GPIO_NUM_25
#define IMU_UART_RXD      GPIO_NUM_26
#define IMU_UART_BAUDRATE 115200
#define IMU_UART_BUF_SIZE 512

extern SemaphoreHandle_t sensorDataMutex;
extern sensor_data_t shared_sensor_data;

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
