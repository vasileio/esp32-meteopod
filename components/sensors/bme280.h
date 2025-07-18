/**
 * @file bme280.h
 * @brief Bosch BME280 sensor driver using ESP-IDF bus–device I2C API.
 *
 * This header consolidates all includes, register defines, and
 * interface declarations for temperature, pressure, and humidity measurements.
 */
#ifndef BME280_H
#define BME280_H

#include <stdint.h>
#include <stdbool.h>
#include <string.h>
#include <math.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "esp_log.h"
#include "esp_check.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @def BME280_I2C_ADDR
 * @brief Default 7-bit I2C address for BME280 (0x76 or 0x77).
 */
#define BME280_I2C_ADDR       0x76

/** @name Register addresses */
/** @{ */
#define BME280_REG_CHIP_ID      0xD0
#define BME280_CHIP_ID          0x60
#define BME280_REG_RESET        0xE0
#define BME280_RESET_VALUE      0xB6
#define BME280_REG_CTRL_HUM     0xF2
#define BME280_REG_STATUS       0xF3
#define BME280_REG_CTRL_MEAS    0xF4
#define BME280_REG_CONFIG       0xF5
#define BME280_REG_DATA         0xF7
#define BME280_REG_CALIB00      0x88
#define BME280_REG_CALIB26      0xE1
/** @} */

/**
 * @brief Oversampling settings.
 */
typedef enum {
    BME280_OSRS_SKIP = 0,
    BME280_OSRS_X1   = 1,
    BME280_OSRS_X2   = 2,
    BME280_OSRS_X4   = 3,
    BME280_OSRS_X8   = 4,
    BME280_OSRS_X16  = 5
} bme280_oversample_t;

/**
 * @brief IIR filter coefficients.
 */
typedef enum {
    BME280_FILTER_OFF = 0,
    BME280_FILTER_2   = 1,
    BME280_FILTER_4   = 2,
    BME280_FILTER_8   = 3,
    BME280_FILTER_16  = 4
} bme280_filter_t;

/**
 * @brief Standby durations (ms) in normal mode.
 */
typedef enum {
    BME280_STANDBY_0_5   = 0,
    BME280_STANDBY_62_5  = 1,
    BME280_STANDBY_125   = 2,
    BME280_STANDBY_250   = 3,
    BME280_STANDBY_500   = 4,
    BME280_STANDBY_1000  = 5,
    BME280_STANDBY_10    = 6,
    BME280_STANDBY_20    = 7
} bme280_standby_t;

/**
 * @brief Operating modes.
 */
typedef enum {
    BME280_MODE_SLEEP  = 0,
    BME280_MODE_FORCED = 1,
    BME280_MODE_NORMAL = 3
} bme280_mode_t;

/**
 * @brief User configuration.
 */
typedef struct {
    bme280_oversample_t osrs_t;
    bme280_oversample_t osrs_p;
    bme280_oversample_t osrs_h;
    bme280_filter_t     filter;
    bme280_standby_t    standby_time;
    bme280_mode_t       mode;
} bme280_config_t;

/**
 * @brief Factory calibration data.
 */
typedef struct {
    uint16_t dig_T1; int16_t dig_T2; int16_t dig_T3;
    uint16_t dig_P1; int16_t dig_P2; int16_t dig_P3;
    int16_t  dig_P4; int16_t dig_P5; int16_t dig_P6;
    int16_t  dig_P7; int16_t dig_P8; int16_t dig_P9;
    uint8_t  dig_H1; int16_t dig_H2; uint8_t dig_H3;
    int16_t  dig_H4; int16_t dig_H5; int8_t  dig_H6;
} bme280_calib_data_t;

/**
 * @brief Measurement results.
 */
typedef struct {
    float   temperature; /**< °C */
    float   pressure;    /**< hPa */
    float   humidity;    /**< %RH */
    int32_t t_fine;      /**< for internal compensation */
} bme280_data_t;

/**
 * @brief Device handle.
 */
typedef struct {
    i2c_master_bus_handle_t  bus;
    i2c_master_dev_handle_t  dev;
    bme280_config_t          config;
    bme280_calib_data_t      calib;
    int32_t                  t_fine;
    SemaphoreHandle_t        mutex;
    bool                     initialised;
} bme280_handle_t;

/**
 * @brief initialise the BME280 sensor.
 */
esp_err_t bme280_init(bme280_handle_t *handle,
                      i2c_port_t        i2c_port,
                      uint8_t           i2c_addr,
                      const bme280_config_t *config);

/**
 * @brief Deinitialise the sensor and release resources.
 */
esp_err_t bme280_deinit(bme280_handle_t *handle);

/**
 * @brief Change configuration at runtime.
 */
esp_err_t bme280_configure(bme280_handle_t *handle,
                           const bme280_config_t *config);

/**
 * @brief Populate default configuration.
 */
void bme280_get_default_config(bme280_config_t *config);

/**
 * @brief Trigger a forced measurement.
 */
esp_err_t bme280_trigger_measurement(bme280_handle_t *handle);

/**
 * @brief Check if forced measurement completed.
 */
esp_err_t bme280_is_meas_ready(bme280_handle_t *handle, bool *ready);

/**
 * @brief Read compensated data (blocking).
 */
esp_err_t bme280_read_data(bme280_handle_t *handle, bme280_data_t *out);

/**
 * @brief ISR-safe read.
 */
esp_err_t bme280_read_data_isr(bme280_handle_t *handle, bme280_data_t *out);

/**
 * @brief Soft-reset the sensor.
 */
esp_err_t bme280_reset(bme280_handle_t *handle);

/**
 * @brief Read chip ID.
 */
esp_err_t bme280_get_chip_id(bme280_handle_t *handle, uint8_t *chip_id);

/**
 * @brief Compute sea-level pressure from altitude.
 */
float bme280_calculate_sea_level_pressure(float pressure, float altitude);

#ifdef __cplusplus
}
#endif

#endif // BME280_H
