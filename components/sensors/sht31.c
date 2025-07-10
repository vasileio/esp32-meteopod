/*
 * @file sht31.c
 * @brief Implementation of SHT31 driver using i2c_master API with improved retry and reset
 *
 * Adapted for Meteopod project. Mirrors BME280 driver style.
 * Logic unchanged; comments reformatted and doxygen-added.
 * License: BSD
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

/* CRC8 polynomial 0x31 */
static uint8_t sht31_crc8(const uint8_t *data, int len)
{
    const uint8_t POLY = 0x31;
    uint8_t crc = 0xFF;
    int i, bit;

    for (i = 0; i < len; i++) {
        crc ^= data[i];
        for (bit = 0; bit < 8; bit++) {
            if (crc & 0x80) {
                crc = (crc << 1) ^ POLY;
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

/**
 * @brief Initialize the SHT31 sensor
 *
 * @param h Handle to sht31_handle_t
 * @param port I2C port number (e.g. I2C_NUM_0)
 * @param addr I2C address (use SHT31_ADDR_DEFAULT)
 *
 * @return ESP_OK on success, or error code
 */
esp_err_t sht31_init(sht31_handle_t *h, i2c_port_t port, uint8_t addr)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");

    /* Obtain I2C bus handle */
    esp_err_t err = i2c_master_get_bus_handle(port, &h->bus);
    ESP_RETURN_ON_ERROR(err, TAG, "get bus");

    /* Add I2C device */
    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000
    };
    err = i2c_master_bus_add_device(h->bus, &dev_conf, &h->dev);
    ESP_RETURN_ON_ERROR(err, TAG, "add device");

    /* Soft reset sensor */
    sht31_reset(h);
    vTaskDelay(pdMS_TO_TICKS(15));

    /* Validate status register */
    uint16_t status;
    err = sht31_read_status(h, &status);
    ESP_RETURN_ON_ERROR(err, TAG, "read status");
    ESP_RETURN_ON_FALSE(status != 0xFFFF, ESP_ERR_INVALID_STATE, TAG, "bad status");

    h->initialised = true;
    return ESP_OK;
}

/**
 * @brief Deinitialize the SHT31 sensor
 *
 * @param h Handle to sht31_handle_t
 *
 * @return ESP_OK on success, or error code
 */
esp_err_t sht31_deinit(sht31_handle_t *h)
{
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not init");

    esp_err_t err = i2c_master_bus_rm_device(h->dev);
    ESP_RETURN_ON_ERROR(err, TAG, "rm device");

    h->initialised = false;
    return ESP_OK;
}

/**
 * @brief Read the status register
 *
 * @param h Handle to sht31_handle_t
 * @param status Pointer to store status
 *
 * @return ESP_OK on success, or error code
 */
esp_err_t sht31_read_status(sht31_handle_t *h, uint16_t *status)
{
    ESP_RETURN_ON_FALSE(h && status, ESP_ERR_INVALID_ARG, TAG, "bad args");

    uint8_t cmd[2] = { (uint8_t)(SHT31_CMD_READSTATUS >> 8), (uint8_t)SHT31_CMD_READSTATUS };
    uint8_t buf[3];

    esp_err_t err = i2c_master_transmit_receive(h->dev, cmd, sizeof(cmd), buf, sizeof(buf), portMAX_DELAY);
    ESP_RETURN_ON_ERROR(err, TAG, "tx_rx status");

    *status = (uint16_t)((buf[0] << 8) | buf[1]);
    return ESP_OK;
}

/**
 * @brief Soft-reset the sensor
 *
 * @param h Handle to sht31_handle_t
 *
 * @return ESP_OK on success, or error code
 */
esp_err_t sht31_reset(sht31_handle_t *h)
{
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not init");

    uint8_t cmd[2] = { (uint8_t)(SHT31_CMD_SOFTRESET >> 8), (uint8_t)SHT31_CMD_SOFTRESET };
    esp_err_t err = i2c_master_transmit(h->dev, cmd, sizeof(cmd), portMAX_DELAY);
    vTaskDelay(pdMS_TO_TICKS(15));

    return err;
}

/**
 * @brief Enable or disable the internal heater
 *
 * @param h Handle to sht31_handle_t
 * @param enable true to enable, false to disable
 *
 * @return ESP_OK on success, or error code
 */
esp_err_t sht31_heater(sht31_handle_t *h, bool enable)
{
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not init");

    uint16_t c = enable ? SHT31_CMD_HEATEREN : SHT31_CMD_HEATERDIS;
    uint8_t cmd[2] = { (uint8_t)(c >> 8), (uint8_t)c };

    return i2c_master_transmit(h->dev, cmd, sizeof(cmd), portMAX_DELAY);
}

/**
 * @brief Read temperature and humidity values
 *
 * Retries on NACK or CRC failure, then resets sensor on persistent failure.
 *
 * @param h Handle to sht31_handle_t
 * @param temperature Pointer to store temperature (°C), or NULL
 * @param humidity Pointer to store humidity (%%RH), or NULL
 *
 * @return ESP_OK on success, ESP_ERR_TIMEOUT if retries exhausted
 */
esp_err_t sht31_read_temp_hum(sht31_handle_t *h, float *temperature, float *humidity)
{
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not init");

    for (int attempt = 0; attempt < SHT31_MAX_TRIES; ++attempt) {
        if (attempt > 0) {
            ESP_LOGW(TAG, "Retrying measurement, attempt %d", attempt + 1);
            sht31_reset(h);
        }

        /* Send measurement command with clock stretching */
        uint8_t cmd[2] = { (uint8_t)(SHT31_CMD_MEAS_HIGHREP_STRETCH >> 8),
                          (uint8_t)SHT31_CMD_MEAS_HIGHREP_STRETCH };
        esp_err_t err = i2c_master_transmit(h->dev, cmd, sizeof(cmd), portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Measure cmd NACK, retry %d", attempt + 1);
            continue;
        }

        vTaskDelay(pdMS_TO_TICKS(20));

        /* Read 6 bytes: temp MSB, temp LSB, temp CRC, hum MSB, hum LSB, hum CRC */
        uint8_t buf[6];
        err = i2c_master_receive(h->dev, buf, sizeof(buf), portMAX_DELAY);
        if (err != ESP_OK) {
            ESP_LOGW(TAG, "Read data NACK, retry %d", attempt + 1);
            continue;
        }

        /* Validate CRCs */
        if (buf[2] != sht31_crc8(buf, 2) || buf[5] != sht31_crc8(buf + 3, 2)) {
            ESP_LOGW(TAG, "CRC fail, retry %d", attempt + 1);
            continue;
        }

        /* Parse temperature */
        uint16_t raw_t = (uint16_t)((buf[0] << 8) | buf[1]);
        int32_t st = ((4375 * (int32_t)raw_t) >> 14) - 4500;
        if (temperature) {
            *temperature = (float)st / 100.0f;
        }

        /* Parse humidity */
        uint16_t raw_h = (uint16_t)((buf[3] << 8) | buf[4]);
        uint32_t sh = (uint32_t)((625 * raw_h) >> 12);
        if (humidity) {
            *humidity = (float)sh / 100.0f;
        }

        return ESP_OK;
    }

    /* Final recovery on persistent failure */
    ESP_LOGE(TAG, "All I2C attempts failed, resetting sensor");
    sht31_reset(h);
    return ESP_ERR_TIMEOUT;
}

/** @brief Read only temperature
 *
 * @param h Handle to sht31_handle_t
 * @param temperature Pointer to store temperature
 * @return ESP_OK on success
 */
esp_err_t sht31_read_temperature(sht31_handle_t *h, float *temperature)
{
    return sht31_read_temp_hum(h, temperature, NULL);
}

/** @brief Read only humidity
 *
 * @param h Handle to sht31_handle_t
 * @param humidity Pointer to store humidity
 * @return ESP_OK on success
 */
esp_err_t sht31_read_humidity(sht31_handle_t *h, float *humidity)
{
    return sht31_read_temp_hum(h, NULL, humidity);
}
