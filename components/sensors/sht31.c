#include "sht31.h"

static const char *TAG = "SHT31";
static i2c_master_dev_handle_t dev_handle;

/* High-repeatability measurement command (no clock stretching) */
static const uint16_t CMD_MEAS_HIGHREP = 0x2400;

/*
 * @brief  Initialize SHT31 on given I2C bus
 * @param  bus_handle: handle to an already configured I2C bus
 * @param  dev_addr:   7-bit I2C address of the SHT31 sensor
 * @param  clk_speed_hz: I2C clock speed
 * @return ESP_OK on success, otherwise an ESP_ERR code
 */
esp_err_t sht31_init(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed_hz)
{
    i2c_device_config_t dev_cfg = {
        .device_address  = dev_addr,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz    = clk_speed_hz,
    };

    esp_err_t err = i2c_master_bus_add_device(bus_handle, &dev_cfg, &dev_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to add device to bus: %s", esp_err_to_name(err));
        return err;
    }

    return ESP_OK;
}

/*
 * @brief  Send a 16-bit command to the sensor
 * @param  cmd: command word
 * @return ESP_OK on success, otherwise an ESP_ERR code
 */
static esp_err_t i2c_write_cmd(uint16_t cmd)
{
    uint8_t data[2] = {
        (uint8_t)(cmd >> 8),
        (uint8_t)(cmd & 0xFF)
    };

    esp_err_t ret = i2c_master_transmit(dev_handle, data, sizeof(data), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C transmit failed: %s", esp_err_to_name(ret));
        return ret;
    }

    return ESP_OK;
}

/*
 * @brief  Read temperature and humidity from SHT31
 * @param  temperature: pointer to store Celsius temperature
 * @param  humidity:    pointer to store relative humidity (%)
 * @return ESP_OK on success, ESP_ERR_INVALID_CRC on CRC failure, otherwise an ESP_ERR code
 */
esp_err_t sht31_read(float *temperature, float *humidity)
{
    esp_err_t ret;

    /* Trigger a single high-repeatability measurement */
    ret = i2c_write_cmd(CMD_MEAS_HIGHREP);
    if (ret != ESP_OK) {
        return ret;
    }

    /* Wait for the measurement to complete (~15 ms) */
    vTaskDelay(pdMS_TO_TICKS(15));

    uint8_t buf[6];
    ret = i2c_master_receive(dev_handle, buf, sizeof(buf), pdMS_TO_TICKS(1000));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "I2C receive failed: %s", esp_err_to_name(ret));
        return ret;
    }

    /* CRC verification using ROM routine */
    if (esp_rom_crc8_be(SHT31_CRC8_INIT, buf, 2) != buf[2]) {
        ESP_LOGE(TAG, "Temperature CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }
    if (esp_rom_crc8_be(SHT31_CRC8_INIT, buf + 3, 2) != buf[5]) {
        ESP_LOGE(TAG, "Humidity CRC mismatch");
        return ESP_ERR_INVALID_CRC;
    }

    uint16_t raw_t = (buf[0] << 8) | buf[1];
    uint16_t raw_h = (buf[3] << 8) | buf[4];

    /* Convert to human-readable values */
    *temperature = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    *humidity    = 100.0f * ((float)raw_h / 65535.0f);

    return ESP_OK;
}
