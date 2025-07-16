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


esp_err_t wind_sensor_direction_init(void);
esp_err_t wind_sensor_direction_read(const char **wind_dir);

#ifdef __cplusplus
}
#endif
