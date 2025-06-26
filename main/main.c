
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

#define TAG "MAIN"

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

#define UART_PORT UART_NUM_1
#define BUF_SIZE 1024
#define UART_RX_PIN 16
#define UART_TX_PIN 17

#define WATCHDOG_TIMEOUT_SEC 5

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

void sensor_task(void *pvParameters)
{
    sensor_data_t data;
    while (1) {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        data.accel_x = rand() % 100;
        data.accel_y = rand() % 100;
        data.accel_z = rand() % 100;
        data.pm25 = rand() % 50;
        data.pm10 = rand() % 100;
        if (xQueueSend(sensorDataQueue, &data, pdMS_TO_TICKS(50)) != pdPASS) {
            ESP_LOGW(TAG, "Sensor queue full");
        }
    }
}

void logging_task(void *pvParameters)
{
    sensor_data_t received_data;
    while (1) {
        if (xQueueReceive(sensorDataQueue, &received_data, portMAX_DELAY)) {
            ESP_LOGI(TAG, "Accel: (%d, %d, %d), PM2.5: %d, PM10: %d",
                     received_data.accel_x,
                     received_data.accel_y,
                     received_data.accel_z,
                     received_data.pm25,
                     received_data.pm10);
        }
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

    xTaskCreate(sensor_task, "SensorTask", 4096, NULL, 5, &sensorTaskHandle);
    xTaskCreate(logging_task, "LoggingTask", 4096, NULL, 4, &loggingTaskHandle);
    xTaskCreate(command_task, "CommandTask", 4096, NULL, 3, &commandTaskHandle);
    xTaskCreate(watchdog_task, "WatchdogTask", 2048, NULL, 2, &watchdogTaskHandle);
}
