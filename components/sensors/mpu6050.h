/*
 * @file mpu6050.h
 * @brief Implementation of MPU-6050 sensor driver functions for ESP32 using ESP-IDF v4.5.1+ I2C master driver.
 *
 * @original_author Jeff Rowberg (I2Cdev library) and ElectronicCats
 */

#ifndef MPU6050_H
#define MPU6050_H

#include <stdint.h>
#include <stdbool.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/**
 * @brief MPU6050 handle structure.
 */
typedef struct {
    i2c_master_bus_handle_t     bus;    /**< I2C bus handle */
    i2c_master_dev_handle_t     dev;    /**< I2C device handle */
    float                       accel_res; /**< Acceleration resolution (g/LSB) */
    float                       gyro_res;  /**< Gyroscope resolution (deg/s/LSB) */
    bool                        initialized; /**< Initialization flag */
} mpu6050_handle_t;

/**
 * @brief Sensor data structure holding all MPU6050 measurements.
 */
typedef struct {
    float accel_x;             /**< X-axis acceleration in g */
    float accel_y;             /**< Y-axis acceleration in g */
    float accel_z;             /**< Z-axis acceleration in g */
    float gyro_x;              /**< X-axis rotation in deg/s */
    float gyro_y;              /**< Y-axis rotation in deg/s */
    float gyro_z;              /**< Z-axis rotation in deg/s */
    float temperature;         /**< Temperature in °C */
} mpu6050_data_t;

/* Register definitions */
#define MPU6050_REG_PWR_MGMT_1    0x6B
#define MPU6050_REG_SMPLRT_DIV    0x19
#define MPU6050_REG_CONFIG        0x1A
#define MPU6050_REG_GYRO_CONFIG   0x1B
#define MPU6050_REG_ACCEL_CONFIG  0x1C
#define MPU6050_REG_INT_ENABLE    0x38
#define MPU6050_REG_INT_STATUS    0x3A
#define MPU6050_REG_ACCEL_XOUT_H  0x3B
#define MPU6050_REG_TEMP_OUT_H    0x41
#define MPU6050_REG_GYRO_XOUT_H   0x43

/* Default I2C address */
#define MPU6050_I2C_ADDR          0x68

/**
 * @brief Initialize the MPU6050 sensor.
 *
 * @param h        Pointer to sensor handle
 * @param port     I2C port to use
 * @param addr     I2C address (use MPU6050_I2C_ADDR)
 * @return esp_err_t
 */
esp_err_t mpu6050_init(mpu6050_handle_t *h, i2c_port_t port, uint8_t addr);

/**
 * @brief Deinitialize the MPU6050 sensor.
 *
 * @param h  Pointer to sensor handle
 * @return esp_err_t
 */
esp_err_t mpu6050_deinit(mpu6050_handle_t *h);

/**
 * @brief Verify device connection.
 *
 * @param h  Pointer to sensor handle
 * @return true if device ID matches, false otherwise
 */
bool mpu6050_test_connection(mpu6050_handle_t *h);

/**
 * @brief Read raw accelerometer data.
 *
 * @param h    Pointer to sensor handle
 * @param ax   Pointer to X-axis acceleration (g)
 * @param ay   Pointer to Y-axis acceleration (g)
 * @param az   Pointer to Z-axis acceleration (g)
 * @return esp_err_t
 */
esp_err_t mpu6050_read_accel(mpu6050_handle_t *h, float *ax, float *ay, float *az);

/**
 * @brief Read raw gyroscope data.
 *
 * @param h    Pointer to sensor handle
 * @param gx   Pointer to X-axis rotation (deg/s)
 * @param gy   Pointer to Y-axis rotation (deg/s)
 * @param gz   Pointer to Z-axis rotation (deg/s)
 * @return esp_err_t
 */
esp_err_t mpu6050_read_gyro(mpu6050_handle_t *h, float *gx, float *gy, float *gz);

/**
 * @brief Read temperature data.
 *
 * @param h    Pointer to sensor handle
 * @param temp Pointer to temperature (°C)
 * @return esp_err_t
 */
esp_err_t mpu6050_read_temp(mpu6050_handle_t *h, float *temp);

/**
 * @brief Read all sensor data in one call.
 *
 * @param h    Pointer to sensor handle
 * @param data Pointer to mpu6050_data_t structure to fill
 * @return esp_err_t
 */
esp_err_t mpu6050_read_all(mpu6050_handle_t *h, mpu6050_data_t *data);

#endif /* MPU6050_H */
