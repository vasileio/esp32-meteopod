/*
 * @file sht31.h
 * @brief ESP-IDF SHT31 sensor driver header
 *
 * Adapted for Meteopod project. Mirrors BME280 driver style.
 * License: BSD
 */

#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"
#include <stdbool.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

/** Handle for SHT31 device */
typedef struct {
    bool initialised;                     /**< Initialization flag */
    i2c_master_bus_handle_t bus;          /**< I2C bus handle */
    i2c_master_dev_handle_t dev;          /**< I2C device handle */
} sht31_handle_t;

/**
 * @brief Measurement results.
 */
typedef struct {
    float   temperature; /**< °C */
    float   humidity;    /**< %RH */
} sht31_data_t;

/** Default I2C address */
#define SHT31_ADDR_DEFAULT                 0x44

/** Commands */
#define SHT31_CMD_MEAS_HIGHREP_STRETCH     0x2C06  /**< High repeatability w/ clock stretch */
#define SHT31_CMD_MEAS_HIGHREP             0x2400  /**< High repeatability, no stretch */
#define SHT31_CMD_READSTATUS               0xF32D  /**< Read status register */
#define SHT31_CMD_SOFTRESET                0x30A2  /**< Soft reset */
#define SHT31_CMD_HEATEREN                 0x306D  /**< Enable heater */
#define SHT31_CMD_HEATERDIS                0x3066  /**< Disable heater */

/* Maximum retry attempts for measurement */
#define SHT31_MAX_TRIES                    3

/**
 * @brief Initialize SHT31 sensor
 *
 * @param h       Handle to sht31_handle_t
 * @param port    I2C port (e.g. I2C_NUM_0)
 * @param addr    Sensor I2C address
 *
 * @return ESP_OK on success
 */
esp_err_t sht31_init(sht31_handle_t *h, i2c_port_t port, uint8_t addr);

/**
 * @brief Deinitialize SHT31 sensor
 *
 * @param h       Handle to sht31_handle_t
 *
 * @return ESP_OK on success
 */
esp_err_t sht31_deinit(sht31_handle_t *h);

/**
 * @brief Read status register
 *
 * @param h       Handle to sht31_handle_t
 * @param status  Pointer to store 16-bit status
 *
 * @return ESP_OK on success
 */
esp_err_t sht31_read_status(sht31_handle_t *h, uint16_t *status);

/**
 * @brief Soft reset sensor
 *
 * @param h       Handle to sht31_handle_t
 *
 * @return ESP_OK on success
 */
esp_err_t sht31_reset(sht31_handle_t *h);

/**
 * @brief Enable or disable heater
 *
 * @param h       Handle to sht31_handle_t
 * @param enable  true to enable, false to disable
 *
 * @return ESP_OK on success
 */
esp_err_t sht31_heater(sht31_handle_t *h, bool enable);

/**
 * @brief Read temperature and humidity
 *
 * @param h            Handle to sht31_handle_t
 * @param temperature  Pointer to store temperature (°C), or NULL
 * @param humidity     Pointer to store humidity (%%RH), or NULL
 *
 * @return ESP_OK on success
 */
esp_err_t sht31_read_temp_hum(sht31_handle_t *h, float *temperature, float *humidity);

/**
 * @brief Read only temperature
 *
 * @param h            Handle to sht31_handle_t
 * @param temperature  Pointer to store temperature (°C)
 *
 * @return ESP_OK on success
 */
esp_err_t sht31_read_temperature(sht31_handle_t *h, float *temperature);

/**
 * @brief Read only humidity
 *
 * @param h        Handle to sht31_handle_t
 * @param humidity Pointer to store humidity (%%RH)
 *
 * @return ESP_OK on success
 */
esp_err_t sht31_read_humidity(sht31_handle_t *h, float *humidity);

#ifdef __cplusplus
}
#endif
