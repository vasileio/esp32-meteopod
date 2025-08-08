/**
 * @file dfrobot_as3935.c
 * @brief AS3935 lightning sensor driver for ESP-IDF using new I2C driver (i2c_master_transmit).
 */

#include "dfrobot_as3935.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

#define AS3935_REG_AFE_GAIN         0x00
#define AS3935_REG_WATCHDOG         0x01
#define AS3935_REG_LIGHTNING_1      0x02
#define AS3935_REG_INT_SRC          0x03
#define AS3935_REG_DISTANCE         0x04
#define AS3935_REG_ENERGY_MSB       0x05
#define AS3935_REG_ENERGY_MID       0x06
#define AS3935_REG_ENERGY_LSB       0x07
#define AS3935_REG_TUN_CAP          0x08
#define AS3935_REG_RESET            0x3C
#define AS3935_REG_CALIB_RCO        0x3D /* Calibrates automatically the internal RC Oscillators */

#define AS3935_RESET_CMD            0x96

static const char *TAG = "AS3935";

static void IRAM_ATTR as3935_irq_handler(void* arg)
{
    dfrobot_as3935_t* sensor = (dfrobot_as3935_t*)arg;
    BaseType_t xHigherPriorityTaskWoken = pdFALSE;
    
    uint32_t irq_event = 1;
    xQueueSendFromISR(sensor->irq_queue, &irq_event, &xHigherPriorityTaskWoken);
    
    if (xHigherPriorityTaskWoken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
}

/**
 * @brief Read multiple bytes starting at a sensor register.
 */
static esp_err_t read_regs(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(dev, &reg, 1, dst, len, portMAX_DELAY);
}

/**
 * @brief Write a single byte to a sensor register.
 */
static esp_err_t write_reg(i2c_master_dev_handle_t dev, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    return i2c_master_transmit(dev, buf, sizeof(buf), portMAX_DELAY);
}

/**
 * @brief Write a single byte with the provided mask to a sensor register.
 */
static esp_err_t write_reg_with_mask(i2c_master_dev_handle_t dev,
                                     uint8_t reg,
                                     uint8_t mask,
                                     uint8_t val)
{
    uint8_t read_data, new_data;

    ESP_RETURN_ON_ERROR(read_regs(dev, reg, &read_data, 1), TAG, "Read register failed");

    new_data = ((read_data & ~mask) | (val & mask));

    return write_reg(dev, reg, new_data);
}

/**
 * @brief Initialise the AS3935 lightning sensor.
 */
esp_err_t dfrobot_as3935_init(dfrobot_as3935_t *sensor, i2c_port_t port, uint8_t addr)
{
    // Validate handle
    ESP_RETURN_ON_FALSE(sensor, ESP_ERR_INVALID_ARG, TAG, "null handle");

    // Get I2C bus handle
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(port, &sensor->bus));

    // Add device to bus
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(sensor->bus, &dc, &sensor->dev), TAG, "i2c_master_bus_add_device() failed");

    sensor->irq_pin = GPIO_NUM_NC;
    sensor->irq_queue = NULL;

    ESP_RETURN_ON_ERROR(dfrobot_as3935_power_up(sensor), TAG, "dfrobot_as3935_power_up() failed");

    return ESP_OK;
}

/**
 * @brief Initialise the AS3935 lightning sensor with IRQ support.
 */
esp_err_t dfrobot_as3935_init_with_irq(dfrobot_as3935_t *sensor, i2c_port_t port, uint8_t addr, gpio_num_t irq_pin)
{
    // Validate handle
    ESP_RETURN_ON_FALSE(sensor, ESP_ERR_INVALID_ARG, TAG, "null handle");

    // Get I2C bus handle
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(port, &sensor->bus));

    // Add device to bus
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000
    };
    ESP_RETURN_ON_ERROR(i2c_master_bus_add_device(sensor->bus, &dc, &sensor->dev), TAG, "i2c_master_bus_add_device() failed");

    sensor->irq_pin = irq_pin;
    
    // Create IRQ queue for interrupt events
    sensor->irq_queue = xQueueCreate(10, sizeof(uint32_t));
    if (sensor->irq_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create IRQ queue");
        return ESP_ERR_NO_MEM;
    }

    // Configure GPIO for interrupt
    gpio_config_t io_conf = {
        .pin_bit_mask = (1ULL << irq_pin),
        .mode = GPIO_MODE_INPUT,
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_NEGEDGE
    };
    ESP_RETURN_ON_ERROR(gpio_config(&io_conf), TAG, "GPIO config failed");

    // Install GPIO ISR service if not already installed
    esp_err_t ret = gpio_install_isr_service(0);
    if (ret != ESP_OK && ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "GPIO ISR service install failed: %s", esp_err_to_name(ret));
        return ret;
    }

    // Hook interrupt handler for specific GPIO pin
    ESP_RETURN_ON_ERROR(gpio_isr_handler_add(irq_pin, as3935_irq_handler, (void*)sensor), TAG, "GPIO ISR handler add failed");

    ESP_RETURN_ON_ERROR(dfrobot_as3935_power_up(sensor), TAG, "dfrobot_as3935_power_up() failed");

    ESP_LOGI(TAG, "AS3935 initialized with IRQ on GPIO %d", irq_pin);
    return ESP_OK;
}

/**
 * @brief Power-up sequence based on datasheet, pg 23/27
 */
esp_err_t dfrobot_as3935_power_up(dfrobot_as3935_t *sensor)
{
    /* Register 0x00, PWD bit: 0 (clears PWD) */
    ESP_RETURN_ON_ERROR(write_reg_with_mask(sensor->dev, AS3935_REG_AFE_GAIN, 0x01, 0x00), TAG, "Clear PWD bit failed"); 

    /* Calibrate RCO */
    ESP_RETURN_ON_ERROR(dfrobot_as3935_calibrate_rco(sensor), TAG, "Calibrate RCO failed"); 

    /* Set DISP_SRCO to 1 */
    ESP_RETURN_ON_ERROR(write_reg_with_mask(sensor->dev, AS3935_REG_TUN_CAP, 0x20, 0x20), TAG, "Set DISP_SRCO to 1 failed"); 
    
    vTaskDelay(pdMS_TO_TICKS(2));

    /* Set DISP_SRCO to 0 */
    ESP_RETURN_ON_ERROR(write_reg_with_mask(sensor->dev, AS3935_REG_TUN_CAP, 0x20, 0x00), TAG, "Set DISP_SRCO to 0 failed"); 

    return ESP_OK;
}

/**
 * @brief Power-down command issue
 */
esp_err_t dfrobot_as3935_power_down(dfrobot_as3935_t *sensor)
{
    /* Register 0x00, PWD bit: 0 (sets PWD) */
    ESP_RETURN_ON_ERROR(write_reg_with_mask(sensor->dev, AS3935_REG_AFE_GAIN, 0x01, 0x01), TAG, "Set PWD bit failed"); 
    ESP_LOGI(TAG, "AS3935 powered down");

    return ESP_OK;
}

/**
 * @brief Reset the AS3935 sensor to default settings.
 */
esp_err_t dfrobot_as3935_reset(dfrobot_as3935_t *sensor)
{
    return write_reg(sensor->dev, AS3935_REG_RESET, AS3935_RESET_CMD);
}

/**
 * @brief Calibrate the internal RC oscillators automatically.
 */
esp_err_t dfrobot_as3935_calibrate_rco(dfrobot_as3935_t *sensor)
{
    ESP_RETURN_ON_ERROR(write_reg(sensor->dev, AS3935_REG_CALIB_RCO, 0x96), TAG, "Calibrating RCO failed");
    vTaskDelay(pdMS_TO_TICKS(2));

    return ESP_OK;
}

/**
 * @brief Set the antenna tuning capacitors value.
 */
esp_err_t dfrobot_as3935_set_tuning_caps(dfrobot_as3935_t *sensor, uint8_t cap)
{
    uint8_t val;
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_TUN_CAP, &val, 1), TAG, "Read TUN_CAP failed");
    val = (val & 0xF0) | (cap & 0x0F);
    return write_reg(sensor->dev, AS3935_REG_TUN_CAP, val);
}

/**
 * @brief Get the interrupt source register value.
 */
esp_err_t dfrobot_as3935_get_interrupt_src(dfrobot_as3935_t *sensor, uint8_t *irq_src)
{
    return read_regs(sensor->dev, AS3935_REG_INT_SRC, irq_src, 1);
}

/**
 * @brief Get the estimated distance to the lightning strike.
 */
esp_err_t dfrobot_as3935_get_lightning_distance(dfrobot_as3935_t *sensor, uint8_t *distance)
{
    uint8_t val;
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_DISTANCE, &val, 1), TAG, "Read distance failed");
    *distance = val & 0x3F;
    return ESP_OK;
}

/**
 * @brief Get the lightning strike energy value.
 */
esp_err_t dfrobot_as3935_get_strike_energy(dfrobot_as3935_t *sensor, uint32_t *energy)
{
    uint8_t msb, mid, lsb;
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_ENERGY_MSB, &msb, 1), TAG, "Read energy MSB failed");
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_ENERGY_MID, &mid, 1), TAG, "Read energy MID failed");
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_ENERGY_LSB, &lsb, 1), TAG, "Read energy LSB failed");

    *energy = ((uint32_t)msb << 16) | ((uint32_t)mid << 8) | lsb;
    return ESP_OK;
}

/**
 * @brief Configure the sensor for indoor operation.
 */
esp_err_t dfrobot_as3935_set_indoor(dfrobot_as3935_t *sensor)
{
    uint8_t val;
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_AFE_GAIN, &val, 1), TAG, "Read AFE_GAIN failed");
    val = (val & 0x3F) | (0x12 << 6);
    return write_reg(sensor->dev, AS3935_REG_AFE_GAIN, val);
}

/**
 * @brief Configure the sensor for outdoor operation.
 */
esp_err_t dfrobot_as3935_set_outdoor(dfrobot_as3935_t *sensor)
{
    uint8_t val;
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_AFE_GAIN, &val, 1), TAG, "Read AFE_GAIN failed");
    val = (val & 0x3F) | (0x0E << 6);
    return write_reg(sensor->dev, AS3935_REG_AFE_GAIN, val);
}

/**
 * @brief Set the noise floor level threshold.
 */
esp_err_t dfrobot_as3935_set_noise_floor(dfrobot_as3935_t *sensor, uint8_t level)
{
    if (level > 7) return ESP_ERR_INVALID_ARG;
    uint8_t val;
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_WATCHDOG, &val, 1), TAG, "Read WATCHDOG failed");
    val = (val & 0x8F) | (level << 4);
    return write_reg(sensor->dev, AS3935_REG_WATCHDOG, val);
}

/**
 * @brief Set the watchdog threshold level.
 */
esp_err_t dfrobot_as3935_set_watchdog_threshold(dfrobot_as3935_t *sensor, uint8_t level)
{
    if (level > 15) return ESP_ERR_INVALID_ARG;
    uint8_t val;
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_WATCHDOG, &val, 1), TAG, "Read WATCHDOG failed");
    val = (val & 0xF0) | (level & 0x0F);
    return write_reg(sensor->dev, AS3935_REG_WATCHDOG, val);
}

/**
 * @brief Set the minimum number of lightning events.
 */
esp_err_t dfrobot_as3935_set_min_lightning(dfrobot_as3935_t *sensor, uint8_t min_events)
{
    if (min_events > 3) return ESP_ERR_INVALID_ARG;
    uint8_t val;
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_LIGHTNING_1, &val, 1), TAG, "Read LIGHTNING_1 failed");
    val = (val & 0xCF) | ((min_events & 0x03) << 4);
    return write_reg(sensor->dev, AS3935_REG_LIGHTNING_1, val);
}

/**
 * @brief Enable or disable disturber detection.
 */
esp_err_t dfrobot_as3935_set_disturber(dfrobot_as3935_t *sensor, bool enable)
{
    uint8_t val;
    ESP_RETURN_ON_ERROR(read_regs(sensor->dev, AS3935_REG_INT_SRC, &val, 1), TAG, "Read INT_SRC failed");
    val &= ~(enable << 5);
    return write_reg(sensor->dev, AS3935_REG_INT_SRC, val);
}

/**
 * @brief Process pending IRQ events from the AS3935 sensor.
 */
esp_err_t dfrobot_as3935_process_irq(dfrobot_as3935_t *sensor, lightning_data_t *lightning_data, uint32_t timeout_ms)
{
    ESP_RETURN_ON_FALSE(sensor && lightning_data, ESP_ERR_INVALID_ARG, TAG, "Invalid arguments");
    ESP_RETURN_ON_FALSE(sensor->irq_queue, ESP_ERR_INVALID_ARG, TAG, "No IRQ queue - sensor not initialized with IRQ support");

    uint32_t irq_event;
    TickType_t timeout_ticks = (timeout_ms == portMAX_DELAY) ? portMAX_DELAY : pdMS_TO_TICKS(timeout_ms);
    
    // Wait for IRQ event
    if (xQueueReceive(sensor->irq_queue, &irq_event, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    // Read interrupt source register to determine what triggered the IRQ
    uint8_t int_source;
    ESP_RETURN_ON_ERROR(dfrobot_as3935_get_interrupt_src(sensor, &int_source), TAG, "Failed to read interrupt source");

    // Extract interrupt reason from bits 3:0
    dfrobot_as3935_int_source_t interrupt_reason = (dfrobot_as3935_int_source_t)(int_source & 0x0F);
    
    ESP_LOGI(TAG, "IRQ triggered - source register: 0x%02X, reason: %d", int_source, interrupt_reason);

    switch (interrupt_reason) {
        case DFROBOT_AS3935_INT_LIGHTNING:
            ESP_LOGI(TAG, "Lightning detected!");
            
            // Read lightning distance
            ESP_RETURN_ON_ERROR(dfrobot_as3935_get_lightning_distance(sensor, &lightning_data->distance_km), TAG, "Failed to read distance");
            
            // Read lightning energy
            ESP_RETURN_ON_ERROR(dfrobot_as3935_get_strike_energy(sensor, &lightning_data->strike_energy), TAG, "Failed to read energy");
            
            ESP_LOGI(TAG, "Lightning strike: %d km away, energy: %lu", lightning_data->distance_km, lightning_data->strike_energy);
            break;
            
        case DFROBOT_AS3935_INT_DISTURBER:
            ESP_LOGW(TAG, "Disturber detected (false positive)");
            break;
            
        case DFROBOT_AS3935_INT_NOISE:
            ESP_LOGW(TAG, "Noise level too high");
            break;
            
        default:
            ESP_LOGW(TAG, "Unknown interrupt source: %d", interrupt_reason);
            return ESP_FAIL;
    }

    return ESP_OK;
}
