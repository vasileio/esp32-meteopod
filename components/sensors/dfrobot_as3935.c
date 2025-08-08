/**
 * @file dfrobot_as3935.c
 * @brief AS3935 lightning sensor driver for ESP-IDF using new I2C driver (i2c_master_transmit).
 */

#include "dfrobot_as3935.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

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

    ESP_RETURN_ON_ERROR(dfrobot_as3935_power_up(sensor), TAG, "dfrobot_as3935_power_up() failed");

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
