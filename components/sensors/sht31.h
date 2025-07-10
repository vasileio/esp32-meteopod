/**
 * @file sht31_espidf_driver.h
 * @brief ESP-IDF SHT31 sensor driver using i2c_master API
 *
 * Adapted for Meteopod project, mirrors BME280 driver style.
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

/**
 * Handle for SHT31 device
 */
typedef struct {
    bool initialised;
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
} sht31_handle_t;

/** Default I2C address */
#define SHT31_ADDR_DEFAULT        0x44

/** Commands */
#define SHT31_CMD_MEAS_HIGHREP_STRETCH      0x2C06
#define SHT31_CMD_MEAS_HIGHREP              0x2400
#define SHT31_CMD_READSTATUS                0xF32D
#define SHT31_CMD_SOFTRESET                 0x30A2
#define SHT31_CMD_HEATEREN                  0x306D
#define SHT31_CMD_HEATERDIS                 0x3066
#define SHT31_STATUS_HEATER_BIT             0x0008

#define SHT31_MAX_TRIES           3

/**
 * @brief Initialize SHT31 on given I2C port and address.
 * @param h    Handle to initialize
 * @param port I2C peripheral (e.g. I2C_NUM_0)
 * @param addr I2C address (use SHT31_ADDR_DEFAULT)
 * @return ESP_OK on success
 */
esp_err_t sht31_init(sht31_handle_t *h, i2c_port_t port, uint8_t addr);

/**
 * @brief Deinitialize the device
 */
esp_err_t sht31_deinit(sht31_handle_t *h);

/** Read status register */
esp_err_t sht31_read_status(sht31_handle_t *h, uint16_t *status);

/** Soft reset sensor */
esp_err_t sht31_reset(sht31_handle_t *h);

/** Enable/disable heater */
esp_err_t sht31_heater(sht31_handle_t *h, bool enable);

/** Read both temperature (°C) and humidity (%%RH) */
esp_err_t sht31_read_temp_hum(sht31_handle_t *h, float *temperature, float *humidity);

/** Read only temperature */
esp_err_t sht31_read_temperature(sht31_handle_t *h, float *temperature);

/** Read only humidity */
esp_err_t sht31_read_humidity(sht31_handle_t *h, float *humidity);

#ifdef __cplusplus
}
#endif
