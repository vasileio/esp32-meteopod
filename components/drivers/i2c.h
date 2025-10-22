/**
 * @file i2c.h
 * @brief I2C master bus initialization and configuration for ESP32 Meteopod
 *
 * Provides I2C master bus setup with standardized configuration for sensor
 * communication. Uses ESP-IDF's new I2C master driver API.
 */

#pragma once

#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

/** @brief I2C port number used for sensor communication */
#define I2C_PORT       I2C_NUM_0

/** @brief GPIO pin for I2C SDA (data line) */
#define I2C_SDA_PIN    GPIO_NUM_21

/** @brief GPIO pin for I2C SCL (clock line) */
#define I2C_SCL_PIN    GPIO_NUM_22

/** @brief Glitch filter count for noise suppression */
#define I2C_GLITCH_CNT 7

/** @brief Standard I2C speed: 100 kHz, up to 100 kbit/s */
#define I2C_SPEED_STANDARD_MODE    (100000U)

/** @brief Fast I2C speed: 400 kHz, up to 400 kbit/s */
#define I2C_SPEED_FAST_MODE        (400000U)

/** @brief Fast Plus I2C speed: 1 MHz, up to 1 Mbit/s */
#define I2C_SPEED_FAST_PLUS_MODE  (1000000U)

/** @brief High Speed I2C mode: 3.4 MHz, up to 3.4 Mbit/s */
#define I2C_SPEED_HIGH_SPEED_MODE (3400000U)

/**
 * @brief Initialize I2C master bus with standard configuration
 *
 * Sets up the I2C master bus on pins 21 (SDA) and 22 (SCL) with
 * fast mode (400 kHz) speed and glitch filtering enabled.
 *
 * @param[out] out_bus_handle Pointer to store the I2C bus handle
 * @return
 *   - ESP_OK: I2C bus initialized successfully
 *   - ESP_ERR_INVALID_ARG: Invalid parameter
 *   - ESP_ERR_NO_MEM: Memory allocation failed
 *   - Other ESP-IDF I2C error codes
 */
esp_err_t i2c_init(i2c_master_bus_handle_t *out_bus_handle);
