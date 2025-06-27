
#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"
#include "driver/uart.h"
#include "driver/gpio.h"
#include "esp_log.h"
#include "esp_system.h"
#include "esp_timer.h"
#include "esp_task_wdt.h"

#include "wifi.h"
#include "system_monitor.h"

#define TAG "MAIN"

#define BLINK_GPIO 2

TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t loggingTaskHandle = NULL;
TaskHandle_t commandTaskHandle = NULL;
TaskHandle_t watchdogTaskHandle = NULL;

QueueHandle_t sensorDataQueue;
QueueHandle_t commandQueue;

typedef struct {
    int accel_x;
    int accel_y;
    int accel_z;
    int pm25;
    int pm10;
} sensor_data_t;

sensor_data_t shared_sensor_data;
SemaphoreHandle_t sensorDataMutex;

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024
#define UART_RX_PIN 16
#define UART_TX_PIN 17

#define WATCHDOG_TIMEOUT_SEC 5

#define IMU_UART_NUM      UART_NUM_2
#define IMU_UART_TXD      GPIO_NUM_25
#define IMU_UART_RXD      GPIO_NUM_26
#define IMU_UART_BAUDRATE 115200
#define IMU_UART_BUF_SIZE 512

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

void init_uart()
{
    uart_config_t uart_config = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_PORT, &uart_config);
    uart_set_pin(UART_PORT, UART_TX_PIN, UART_RX_PIN, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_PORT, BUF_SIZE * 2, BUF_SIZE * 2, 20, &commandQueue, 0);
}

void periodic_sensor_cb(void* arg)
{
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    vTaskNotifyGiveFromISR(sensorTaskHandle, &xHigherPriorityTaskWoken);
    portYIELD_FROM_ISR(xHigherPriorityTaskWoken);
}

void sensor_accel_task(void *pvParameters)
{
    imu_uart_init();

    uint8_t data[IMU_UART_BUF_SIZE];
    while (1) {
        int len = uart_read_bytes(IMU_UART_NUM, data, sizeof(data), pdMS_TO_TICKS(100));
        if (len > 0) {
            // This is a stub — real parsing should use preamble + checksum
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

void logging_task(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50))) {
            ESP_LOGI(TAG, "Accel: (%d, %d, %d), PM2.5: %d, PM10: %d",
                     shared_sensor_data.accel_x,
                     shared_sensor_data.accel_y,
                     shared_sensor_data.accel_z,
                     shared_sensor_data.pm25,
                     shared_sensor_data.pm10);
            xSemaphoreGive(sensorDataMutex);
        } else {
            ESP_LOGW(TAG, "Failed to take data mutex");
        }

        vTaskDelay(pdMS_TO_TICKS(1000));  // Log every 1s, adjust as needed
    }
}

void command_task(void *pvParameters)
{
    uart_event_t event;
    uint8_t data[BUF_SIZE];
    while (1) {
        if (xQueueReceive(commandQueue, &event, portMAX_DELAY)) {
            if (event.type == UART_DATA) {
                int len = uart_read_bytes(UART_PORT, data, event.size, portMAX_DELAY);
                data[len] = '\0';
                ESP_LOGI(TAG, "Received Command: %s", (char*)data);
            }
        }
    }
}

void watchdog_task(void *pvParameters)
{
    esp_task_wdt_add(NULL);
    while (1) {
        esp_task_wdt_reset();
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}

void blink_task(void *pvParameters)
{
    esp_rom_gpio_pad_select_gpio(BLINK_GPIO);
    gpio_set_direction(BLINK_GPIO, GPIO_MODE_OUTPUT);

    while (1) {
        gpio_set_level(BLINK_GPIO, 1);
        vTaskDelay(pdMS_TO_TICKS(500));

        gpio_set_level(BLINK_GPIO, 0);
        vTaskDelay(pdMS_TO_TICKS(500));
    }
}

void app_main(void)
{
    ESP_LOGI(TAG, "System booting...");
    init_uart();
    wifi_init_sta();

    sensorDataQueue = xQueueCreate(10, sizeof(sensor_data_t));
    commandQueue = xQueueCreate(10, sizeof(uart_event_t));

    esp_timer_create_args_t timer_args = {
        .callback = &periodic_sensor_cb,
        .name = "sensor_timer"
    };

    esp_timer_handle_t periodic_timer;
    esp_timer_create(&timer_args, &periodic_timer);
    esp_timer_start_periodic(periodic_timer, 1000000); // 1s

    sensorDataMutex = xSemaphoreCreateMutex();
    if (sensorDataMutex == NULL) {
        ESP_LOGE("MAIN", "Failed to create mutex");
    }

    xTaskCreate(sensor_accel_task, "sensor_accel_task", 4096, NULL, 5, &sensorTaskHandle);
    xTaskCreate(sensor_pm_task, "sensor_pm_task", 4096, NULL, 5, &sensorTaskHandle);
    xTaskCreate(logging_task, "LoggingTask", 4096, NULL, 4, &loggingTaskHandle);
    xTaskCreate(command_task, "CommandTask", 4096, NULL, 3, &commandTaskHandle);
    xTaskCreate(watchdog_task, "WatchdogTask", 2048, NULL, 6, &watchdogTaskHandle);
    xTaskCreate(blink_task, "blink_task", 3072, NULL, 1, NULL);
    xTaskCreate(system_monitor_task, "SystemMonitorTask", 4096, NULL, 1, NULL);
}
