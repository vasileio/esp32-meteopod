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

static const char *TAG = "SENSORS";

void imu_uart_init(void)
{
    const uart_config_t uart_config = {
        .baud_rate = IMU_UART_BAUDRATE,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_APB,
    };

    ESP_ERROR_CHECK(uart_driver_install(IMU_UART_NUM, IMU_UART_BUF_SIZE * 2, 0, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(IMU_UART_NUM, &uart_config));
    ESP_ERROR_CHECK(uart_set_pin(IMU_UART_NUM, IMU_UART_TXD, IMU_UART_RXD, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART2 initialized for IMU");
}

void sensor_accel_task(void *pvParameters)
{
    imu_uart_init();

    uint8_t data[IMU_UART_BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(IMU_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        if (len > 0) {
            ESP_LOGI("IMU", "Received %d bytes from IMU", len);
            ESP_LOG_BUFFER_HEXDUMP("IMU", data, len, ESP_LOG_INFO);
        }

        vTaskDelay(pdMS_TO_TICKS(200)); // ~5Hz polling
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
