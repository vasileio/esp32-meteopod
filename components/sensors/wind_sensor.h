#pragma once

#include "esp_err.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Measurement results.
 */
typedef struct {
    const char      *direction;     /**< String */
    float           speed;          /**< m/s */
} wind_data_t;


/**
 * @brief Initialise the wind direction and speed sensor subsystem.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wind_sensor_init(void);

/**
 * @brief Initialize ADC channel and configuration.
 *
 * @param channel ADC channel number.
 * @param attenuation ADC attenuation setting.
 * @param[out] adc_handle Output ADC handle.
 * @return ESP_OK on success, error code on failure.
 */
esp_err_t wind_sensor_read(wind_data_t *wind_readings);

#ifdef __cplusplus
}
#endif
