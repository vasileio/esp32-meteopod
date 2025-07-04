#ifndef SHT31_SENSOR_H
#define SHT31_SENSOR_H

#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_crc.h"
#include "driver/i2c_master.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* Default 7-bit I²C address for SHT31 (ADDR pin low) */
#define SHT31_I2C_ADDR_DEFAULT   0x44

#define SHT31_I2C_SPEED   100000U
#define SHT31_CRC8_INIT  0xFF

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the SHT31 sensor on the given I2C bus.
 *
 * @param bus_handle I2C master bus handle from i2c_init()
 * @param dev_addr 7-bit I2C address (e.g., 0x44)
 * @param clk_speed_hz I2C clock speed for this device (e.g., 100000)
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sht31_init(i2c_master_bus_handle_t bus_handle, uint8_t dev_addr, uint32_t clk_speed_hz);

/**
 * @brief Read temperature and humidity from the SHT31 sensor.
 *
 * @param temperature Pointer to float to receive temperature in Celsius
 * @param humidity Pointer to float to receive relative humidity in %%
 * @return esp_err_t ESP_OK on success, error code otherwise
 */
esp_err_t sht31_read(float *temperature, float *humidity);

#ifdef __cplusplus
}
#endif

#endif // SHT31_SENSOR_H