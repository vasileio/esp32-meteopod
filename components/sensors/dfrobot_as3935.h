/**
 * @file dfrobot_as3935.h
 * @brief Driver for AS3935 lightning sensor using ESP-IDF i2c_master driver
 *
 * Ported and adapted to ESP32 Meteopod style from DFRobot Arduino library
 */

#pragma once

#include "driver/i2c_master.h"
#include <stdint.h>
#include <esp_err.h>

#ifdef __cplusplus
extern "C" {
#endif

#define AS3935_I2C_ADDR     0x03

/**
 * @brief AS3935 sensor handle structure
 */
typedef struct {
    i2c_master_bus_handle_t bus;
    i2c_master_dev_handle_t dev;
    uint8_t i2c_addr;
} dfrobot_as3935_t;

/**
 * @brief AS3935 gain boost mode
 */
typedef enum {
    DFROBOT_AS3935_GAIN_INDOOR = 0,
    DFROBOT_AS3935_GAIN_OUTDOOR = 1
} dfrobot_as3935_gain_mode_t;

/**
 * @brief AS3935 interrupt source
 */
typedef enum {
    DFROBOT_AS3935_INT_NOISE = 1,
    DFROBOT_AS3935_INT_DISTURBER = 4,
    DFROBOT_AS3935_INT_LIGHTNING = 8
} dfrobot_as3935_int_source_t;

/**
 * @brief Lightning data structure containing all relevant sensor information
 */
typedef struct {
    uint8_t distance_km;                           /**< Distance to lightning strike in kilometers */
    uint32_t strike_energy;                        /**< Lightning strike energy value (24-bit) */
} lightning_data_t;

/**
 * @brief Initialize the AS3935 lightning sensor with I2C communication
 * 
 * This function initializes the AS3935 lightning sensor by setting up I2C communication
 * and performing a soft reset to restore the sensor to its default configuration.
 * The sensor structure will be configured with the provided I2C port and device address.
 * 
 * @param[out] sensor Pointer to AS3935 sensor structure to be initialized
 * @param[in] port I2C port number (I2C_NUM_0 or I2C_NUM_1)
 * @param[in] addr 7-bit I2C device address (typically 0x03)
 * 
 * @return 
 *         - ESP_OK: Success
 *         - ESP_ERR_INVALID_ARG: Invalid arguments provided
 *         - ESP_FAIL: I2C communication error or sensor initialization failed
 * 
 * @note After successful initialization, the sensor will be in its default state.
 *       Additional configuration may be required for optimal performance.
 * 
 * @warning Ensure I2C master is properly configured before calling this function.
 */
esp_err_t dfrobot_as3935_init(dfrobot_as3935_t *sensor, i2c_port_t port, uint8_t addr);

/**
 * @brief Calibrate the internal RC oscillators automatically
 * @param sensor Pointer to AS3935 sensor structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_calibrate_rco(dfrobot_as3935_t *sensor);

/**
 * @brief Set gain control mode
 * @param sensor Pointer to AS3935 sensor structure
 * @param mode Indoor or Outdoor gain mode
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_set_gain(dfrobot_as3935_t *sensor, dfrobot_as3935_gain_mode_t mode);

/**
 * @brief Set noise floor level (0-7)
 * @param sensor Pointer to AS3935 sensor structure
 * @param level Noise floor level (0-7, higher values = less sensitive)
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if level > 7
 */
esp_err_t dfrobot_as3935_set_noise_floor(dfrobot_as3935_t *sensor, uint8_t level);

/**
 * @brief Set watchdog threshold (0-15)
 * @param sensor Pointer to AS3935 sensor structure
 * @param threshold Watchdog threshold level (0-15)
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if threshold > 15
 */
esp_err_t dfrobot_as3935_set_watchdog_threshold(dfrobot_as3935_t *sensor, uint8_t threshold);

/**
 * @brief Set minimum number of lightning events before IRQ
 * @param sensor Pointer to AS3935 sensor structure
 * @param num Minimum lightning events (0-3)
 * @return esp_err_t ESP_OK on success, ESP_ERR_INVALID_ARG if num > 3
 */
esp_err_t dfrobot_as3935_set_min_lightning(dfrobot_as3935_t *sensor, uint8_t num);

/**
 * @brief Enable or disable disturber detection
 * @param sensor Pointer to AS3935 sensor structure
 * @param enable true to enable disturber detection, false to disable
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_set_disturber(dfrobot_as3935_t *sensor, bool enable);

/**
 * @brief Read distance to storm in km
 *
 * @param sensor Sensor handle
 * @param dist_km Pointer to result distance in km
 * @return true on success
 */
esp_err_t dfrobot_as3935_read_distance(dfrobot_as3935_t *sensor, uint8_t *dist_km);

/**
 * @brief Read lightning energy value (MSB+LSB+LSB)
 * @param sensor Pointer to AS3935 sensor structure
 * @param energy Pointer to store energy value (24-bit)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_read_energy(dfrobot_as3935_t *sensor, uint32_t *energy);

/**
 * @brief Read interrupt source
 * @param sensor Pointer to AS3935 sensor structure
 * @param src Pointer to store interrupt source value
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_get_interrupt(dfrobot_as3935_t *sensor, dfrobot_as3935_int_source_t *src);

/**
 * @brief Power-up the AS3935 sensor
 * @param sensor Pointer to AS3935 sensor structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_power_up(dfrobot_as3935_t *sensor);

/**
 * @brief Power-down the AS3935 sensor
 * @param sensor Pointer to AS3935 sensor structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_power_down(dfrobot_as3935_t *sensor);

/**
 * @brief Reset the AS3935 sensor to default settings
 * @param sensor Pointer to AS3935 sensor structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_reset(dfrobot_as3935_t *sensor);

/**
 * @brief Set the antenna tuning capacitors value
 * @param sensor Pointer to AS3935 sensor structure
 * @param cap Capacitor value (0-15)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_set_tuning_caps(dfrobot_as3935_t *sensor, uint8_t cap);

/**
 * @brief Get the interrupt source register value
 * @param sensor Pointer to AS3935 sensor structure
 * @param irq_src Pointer to store interrupt source value
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_get_interrupt_src(dfrobot_as3935_t *sensor, uint8_t *irq_src);

/**
 * @brief Get the estimated distance to the lightning strike
 * @param sensor Pointer to AS3935 sensor structure
 * @param distance Pointer to store distance value (in km)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_get_lightning_distance(dfrobot_as3935_t *sensor, uint8_t *distance);

/**
 * @brief Get the lightning strike energy value
 * @param sensor Pointer to AS3935 sensor structure
 * @param energy Pointer to store energy value (24-bit)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_get_strike_energy(dfrobot_as3935_t *sensor, uint32_t *energy);

/**
 * @brief Configure the sensor for indoor operation
 * @param sensor Pointer to AS3935 sensor structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_set_indoor(dfrobot_as3935_t *sensor);

/**
 * @brief Configure the sensor for outdoor operation
 * @param sensor Pointer to AS3935 sensor structure
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t dfrobot_as3935_set_outdoor(dfrobot_as3935_t *sensor);

#ifdef __cplusplus
}
#endif
