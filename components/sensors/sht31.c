/**
 * @file sht31_espidf_driver.c
 * @brief Implementation of SHT31 driver using i2c_master API with improved retry and reset
 */

#include "sht31.h"
#include "driver/i2c_master.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "sht31";

// CRC8 polynomial 0x31
static uint8_t sht31_crc8(const uint8_t *data, int len) {
    const uint8_t POLY = 0x31;
    uint8_t crc = 0xFF;
    for (int i = 0; i < len; i++) {
        crc ^= data[i];
        for (int bit = 0; bit < 8; bit++) {
            crc = (crc & 0x80) ? (crc << 1) ^ POLY : (crc << 1);
        }
    }
    return crc;
}

esp_err_t sht31_init(sht31_handle_t *h, i2c_port_t port, uint8_t addr) {
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");

    // Get bus handle
    esp_err_t err = i2c_master_get_bus_handle(port, &h->bus);
    ESP_RETURN_ON_ERROR(err, TAG, "get bus");

    // Add device
    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000
    };
    err = i2c_master_bus_add_device(h->bus, &dev_conf, &h->dev);
    ESP_RETURN_ON_ERROR(err, TAG, "add device");

    // Soft reset
    sht31_reset(h);
    vTaskDelay(pdMS_TO_TICKS(15));

    // Verify status
    uint16_t status;
    err = sht31_read_status(h, &status);
    ESP_RETURN_ON_ERROR(err, TAG, "read status");
    ESP_RETURN_ON_FALSE(status != 0xFFFF, ESP_ERR_INVALID_STATE, TAG, "bad status");

    h->initialised = true;
    return ESP_OK;
}

esp_err_t sht31_deinit(sht31_handle_t *h) {
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not init");
    esp_err_t err = i2c_master_bus_rm_device(h->dev);
    ESP_RETURN_ON_ERROR(err, TAG, "rm device");
    h->initialised = false;
    return ESP_OK;
}

esp_err_t sht31_read_status(sht31_handle_t *h, uint16_t *status) {
    ESP_RETURN_ON_FALSE(h && status, ESP_ERR_INVALID_ARG, TAG, "bad args");
    uint8_t cmd[2] = { SHT31_CMD_READSTATUS >> 8, SHT31_CMD_READSTATUS & 0xFF };
    uint8_t buf[3];
    esp_err_t err = i2c_master_transmit_receive(h->dev, cmd, 2, buf, 3, portMAX_DELAY);
    ESP_RETURN_ON_ERROR(err, TAG, "tx_rx status");
    *status = (buf[0] << 8) | buf[1];
    return ESP_OK;
}

esp_err_t sht31_reset(sht31_handle_t *h) {
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not init");
    uint8_t cmd[2] = { SHT31_CMD_SOFTRESET >> 8, SHT31_CMD_SOFTRESET & 0xFF };
    esp_err_t err = i2c_master_transmit(h->dev, cmd, 2, portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(15));
    return err;
}

esp_err_t sht31_heater(sht31_handle_t *h, bool enable) {
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not init");
    uint16_t c = enable ? SHT31_CMD_HEATEREN : SHT31_CMD_HEATERDIS;
    uint8_t cmd[2] = { c >> 8, c & 0xFF };
    return i2c_master_transmit(h->dev, cmd, 2, portMAX_DELAY);
}

esp_err_t sht31_read_temp_hum(sht31_handle_t *h, float *temperature, float *humidity) {
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not init");

    esp_err_t err;
    for (int attempt = 0; attempt < SHT31_MAX_TRIES; ++attempt) {
        if (attempt > 0) {
            ESP_LOGW(TAG, "Retrying measurement, attempt %d", attempt+1);
            // Reset sensor between retries
            sht31_reset(h);
        }

        // Send measurement command with clock stretching
        uint8_t cmd[2] = { SHT31_CMD_MEAS_HIGHREP_STRETCH >> 8, SHT31_CMD_MEAS_HIGHREP_STRETCH & 0xFF };
        err = i2c_master_transmit(h->dev, cmd, 2, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Measure cmd NACK, retry %d", attempt+1);
            continue;
        }

        // Wait for sensor to complete (max 15ms per datasheet)
        vTaskDelay(pdMS_TO_TICKS(20));

        uint8_t buf[6];
        err = i2c_master_receive(h->dev, buf, 6, portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Read data NACK, retry %d", attempt+1);
            continue;
        }

        // CRC checks
        if (buf[2] != sht31_crc8(buf,2) || buf[5] != sht31_crc8(buf+3,2)) {
            ESP_LOGW(TAG, "CRC fail, retry %d", attempt+1);
            continue;
        }

        // Parse temperature
        uint16_t raw_t = (buf[0]<<8) | buf[1];
        int32_t st = ((4375 * (int32_t)raw_t) >> 14) - 4500;
        if (temperature) *temperature = st / 100.0f;

        // Parse humidity
        uint16_t raw_h = (buf[3]<<8) | buf[4];
        uint32_t sh = (625 * (uint32_t)raw_h) >> 12;
        if (humidity) *humidity = sh / 100.0f;

        return ESP_OK;
    }

    ESP_LOGE(TAG, "All I2C attempts failed, resetting bus and sensor");
    // Final recovery: reset sensor
    sht31_reset(h);
    return ESP_ERR_TIMEOUT;
}

esp_err_t sht31_read_temperature(sht31_handle_t *h, float *temperature) {
    return sht31_read_temp_hum(h, temperature, NULL);
}

esp_err_t sht31_read_humidity(sht31_handle_t *h, float *humidity) {
    return sht31_read_temp_hum(h, NULL, humidity);
}
