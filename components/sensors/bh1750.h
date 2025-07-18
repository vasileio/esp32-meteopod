/**
 * @file bh1750.h
 * @brief BH1750FVI digital light sensor driver using ESP-IDF bus–device I2C API.
 *
 *   Datasheet: http://www.elechouse.com/elechouse/images/product/Digital%20light%20Sensor/bh1750fvi-e.pdf
 *   Written by Christopher Laws, March 2013. Updated for ESP-IDF I2C bus-device API.
 */

#pragma once

#include "driver/i2c_master.h"
#include "esp_err.h"

/** @brief Default 7-bit I2C address for BH1750 (depends on ADDR pin: 0x23 or 0x5C) */
#define BH1750_I2C_ADDR        0x23

/** @brief Default MTreg sensitivity value */
#define BH1750_DEFAULT_MTREG   69

/** @brief Minimum allowed MTreg value */
#define BH1750_MTREG_MIN       31

/** @brief Maximum allowed MTreg value */
#define BH1750_MTREG_MAX       254

/** @brief Conversion factor used to calculate lux */
#define BH1750_CONV_FACTOR     1.2f

/**
 * @brief Measurement modes supported by BH1750.
 */
typedef enum {
    UNCONFIGURED                 = 0x00, /**< Default power-down state or unconfigured */
    CONTINUOUS_HIGH_RES_MODE    = 0x10, /**< Continuous measurement, 1 lux resolution, ~120ms */
    CONTINUOUS_HIGH_RES_MODE_2  = 0x11, /**< Continuous measurement, 0.5 lux resolution, ~120ms */
    CONTINUOUS_LOW_RES_MODE     = 0x13, /**< Continuous measurement, 4 lux resolution, ~16ms */
    ONE_TIME_HIGH_RES_MODE      = 0x20, /**< One-time measurement, 1 lux resolution, ~120ms */
    ONE_TIME_HIGH_RES_MODE_2    = 0x21, /**< One-time measurement, 0.5 lux resolution, ~120ms */
    ONE_TIME_LOW_RES_MODE       = 0x23  /**< One-time measurement, 4 lux resolution, ~16ms */
} bh1750_mode_t;

/**
 * @brief BH1750 device handle structure.
 */
typedef struct {
    i2c_master_bus_handle_t bus; /**< I2C bus handle */
    i2c_master_dev_handle_t dev; /**< I2C device handle */
    bh1750_mode_t mode;          /**< Current measurement mode */
    uint8_t mtreg;               /**< MTreg sensitivity value */
    bool initialised;            /**< Flag indicating if the sensor has been initialized */
} bh1750_handle_t;

/**
 * @brief Initialize the BH1750 sensor.
 *
 * @param[inout] h      Pointer to sensor handle to initialize.
 * @param[in]    port   I2C port number (e.g., I2C_NUM_0).
 * @param[in]    addr   I2C address of the sensor.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t bh1750_init(bh1750_handle_t *h, i2c_port_t port, uint8_t addr);

/**
 * @brief Set the measurement mode of the sensor.
 *
 * @param[inout] h      Pointer to initialized sensor handle.
 * @param[in]    mode   Desired measurement mode.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t bh1750_set_mode(bh1750_handle_t *h, bh1750_mode_t mode);

/**
 * @brief Reset the BH1750 data register (must be powered on first).
 *
 * @param[inout] h      Pointer to initialized sensor handle.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t bh1750_reset(bh1750_handle_t *h);

/**
 * @brief Set the MTreg value (measurement time register).
 *
 * @param[inout] h       Pointer to initialized sensor handle.
 * @param[in]    mtreg   MTreg value (31–254).
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t bh1750_set_mtreg(bh1750_handle_t *h, uint8_t mtreg);

/**
 * @brief Read light level in lux from the BH1750 sensor.
 *
 * Automatically triggers measurement if in one-time mode.
 *
 * @param[inout] h      Pointer to initialized sensor handle.
 * @param[out]   lux    Pointer to variable to store result in lux.
 * @return ESP_OK on success, error code otherwise.
 */
esp_err_t bh1750_read_light(bh1750_handle_t *h, float *lux);
