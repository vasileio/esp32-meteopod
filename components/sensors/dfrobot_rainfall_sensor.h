#ifndef DFROBOT_RAINFALL_SENSOR_H
#define DFROBOT_RAINFALL_SENSOR_H

#include <stdint.h>
#include <string.h>    /* for memcpy() */
#include "esp_log.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/i2c_master.h"

/* Default 7-bit I²C address for the DFRobot rain-bucket sensor */
#define DFRBOT_RAINFALL_SENSOR_I2C_ADDR_DEFAULT   0x1D

/* Register addresses */
#define DFROBOT_RAINFALL_SENSOR_REG_PID            0x00
#define DFROBOT_RAINFALL_SENSOR_REG_VID            0x02
#define DFROBOT_RAINFALL_SENSOR_REG_VERSION        0x0A
#define DFROBOT_RAINFALL_SENSOR_REG_TIME_RAINFALL  0x0C
#define DFROBOT_RAINFALL_SENSOR_REG_CUMULATIVE     0x10
#define DFROBOT_RAINFALL_SENSOR_REG_RAW_DATA       0x14
#define DFROBOT_RAINFALL_SENSOR_REG_SYS_TIME       0x18
#define DFROBOT_RAINFALL_SENSOR_REG_RAIN_HOUR      0x26
#define DFROBOT_RAINFALL_SENSOR_REG_BASE_RAINFALL  0x28

/**
 * @brief Handle for the DFRobot rain-bucket sensor
 */
typedef struct {
    i2c_master_dev_handle_t dev;  /* I²C device handle */
    uint32_t pid;                 /* Product ID */
    uint32_t vid;                 /* Vendor ID */
} DFRobot_rainfall_sensor_t;

/**
 * @brief  Initialize sensor over I²C
 * @param  sensor:      pointer to handle
 * @param  bus:         already-configured I2C bus handle
 * @param  addr_7bit:   7-bit sensor address (default 0x1D)
 * @param  clk_speed_hz: I²C clock speed
 * @return ESP_OK on success, else error
 */
esp_err_t DFRobot_rainfall_sensor_init(DFRobot_rainfall_sensor_t *sensor,
                                       i2c_master_bus_handle_t bus,
                                       uint8_t addr_7bit,
                                       uint32_t clk_speed_hz);

/**
 * @brief  Probe and read PID/VID
 * @param  sensor: handle returned by init()
 * @return ESP_OK if expected PID/VID, ESP_ERR_NOT_FOUND otherwise
 */
esp_err_t DFRobot_rainfall_sensor_begin(DFRobot_rainfall_sensor_t *sensor);

/**
 * @brief  Read raw version word (M.m.p.f)
 */
esp_err_t DFRobot_rainfall_sensor_get_version(DFRobot_rainfall_sensor_t *sensor,
                                              uint16_t *version);

/**
 * @brief  Get total rainfall since power-up (mm)
 */
esp_err_t DFRobot_rainfall_sensor_get_cumulative(DFRobot_rainfall_sensor_t *sensor,
                                                 float *out_mm);

/**
 * @brief  Get rainfall over last N hours (1–24) (mm)
 */
esp_err_t DFRobot_rainfall_sensor_get_cumulative_hours(DFRobot_rainfall_sensor_t *sensor,
                                                       uint8_t hours,
                                                       float *out_mm);

/**
 * @brief  Get raw tip-count since power-up
 */
esp_err_t DFRobot_rainfall_sensor_get_raw_count(DFRobot_rainfall_sensor_t *sensor,
                                                uint32_t *out_cnt);

/**
 * @brief  Configure bucket volume (mm per tip)
 */
esp_err_t DFRobot_rainfall_sensor_set_bucket_volume(DFRobot_rainfall_sensor_t *sensor,
                                                    float mm_per_tip);

/**
 * @brief  Read uptime (hours)
 */
esp_err_t DFRobot_rainfall_sensor_get_uptime(DFRobot_rainfall_sensor_t *sensor,
                                             float *out_hr);

#endif  /* DFROBOT_RAINFALL_SENSOR_H */
