#include "include/main.h"

#define TAG "MAIN"

#define BLINK_GPIO 2

TaskHandle_t sensorTaskHandle = NULL;
TaskHandle_t loggingTaskHandle = NULL;
TaskHandle_t commandTaskHandle = NULL;
TaskHandle_t watchdogTaskHandle = NULL;

QueueHandle_t sensorDataQueue;
QueueHandle_t commandQueue;

sensor_data_t shared_sensor_data;
SemaphoreHandle_t sensorDataMutex;

static i2c_master_bus_handle_t bus_handle;
static DFRobot_rainfall_sensor_t rain_sensor;

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

void logging_task(void *pvParameters)
{
    while (1) {
        if (xSemaphoreTake(sensorDataMutex, pdMS_TO_TICKS(50))) {
            ESP_LOGI(TAG, "Temp: %d C, Rel. Humidity: %d, PM2.5: %d, PM10: %d",
                     shared_sensor_data.temp_C,
                     shared_sensor_data.rel_hum,
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
    ESP_LOGI(TAG, "System booting…");

    init_uart();
    wifi_init_sta();

    esp_err_t err = i2c_init(&bus_handle);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "I2C master init failed: %s", esp_err_to_name(err));
        return;
    }

    /*––– Initialize SHT31 on I2C bus –––*/
    err = sht31_init(bus_handle,
                     SHT31_I2C_ADDR_DEFAULT,
                     I2C_SPEED_STANDARD_MODE);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "SHT31 init failed: %s", esp_err_to_name(err));
    }

    /*––– Initialize DFRobot rain-bucket sensor on same I2C bus –––*/
    err = DFRobot_rainfall_sensor_init(&rain_sensor,
                                       bus_handle,
                                       DFRBOT_RAINFALL_SENSOR_I2C_ADDR_DEFAULT,
                                       I2C_SPEED_STANDARD_MODE);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Rain sensor init failed: %s", esp_err_to_name(err));
    } else {
        /* optional: verify PID/VID */
        err = DFRobot_rainfall_sensor_begin(&rain_sensor);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "Rain sensor probe failed: %s", esp_err_to_name(err));
        }
    }

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

    xTaskCreate(sensor_temp_hum_task, "sensor_temp_hum_task", 4096, NULL, 5, &sensorTaskHandle);
    xTaskCreate(sensor_pm_task, "sensor_pm_task", 4096, NULL, 5, &sensorTaskHandle);
    xTaskCreate(logging_task, "LoggingTask", 4096, NULL, 4, &loggingTaskHandle);
    xTaskCreate(command_task, "CommandTask", 4096, NULL, 3, &commandTaskHandle);
    xTaskCreate(watchdog_task, "WatchdogTask", 2048, NULL, 6, &watchdogTaskHandle);
    xTaskCreate(blink_task, "blink_task", 3072, NULL, 1, NULL);
    xTaskCreate(system_monitor_task, "SystemMonitorTask", 4096, NULL, 1, NULL);
}
