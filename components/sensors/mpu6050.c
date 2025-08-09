/*
 * @file mpu6050.c
 * @brief Implementation of MPU-6050 sensor driver functions for ESP32 using ESP-IDF v4.5.1+ I2C master driver.
 *
 * @original_author Jeff Rowberg (I2Cdev library) and ElectronicCats
 */

#include "mpu6050.h"
#include <esp_log.h>
#include "driver/i2c_master.h"

#define TAG "MPU6050"

/**
 * @brief  Write a single byte to an MPU-6050 register.
 * @param  h    MPU handle (h->dev is the i2c_master_dev_handle_t)
 * @param  reg  Register address
 * @param  val  Value to write
 * @return ESP_OK on success
 */
static esp_err_t write_reg(mpu6050_handle_t *h, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(
        h->dev,        /* device handle */
        buf,           /* data to write */
        sizeof(buf),   /* write length */
        1000           /* timeout in ms */
    );
}

/**
 * @brief  Read multiple bytes starting from an MPU-6050 register.
 * @param  h    MPU handle
 * @param  reg  Starting register address
 * @param  dst  Buffer to receive data
 * @param  len  Number of bytes to read
 * @return ESP_OK on success
 */
static esp_err_t read_regs(mpu6050_handle_t *h, uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(
        h->dev,        /* device handle */
        &reg,          /* pointer to register address */
        1,             /* write length = 1 */
        dst,           /* read buffer */
        len,           /* read length */
        1000           /* timeout in ms */
    );
}

esp_err_t mpu6050_init(mpu6050_handle_t *h, i2c_port_t port, uint8_t addr)
{
    if (!h) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = i2c_master_get_bus_handle(port, &h->bus);
    if (err != ESP_OK) {
        return err;
    }

    i2c_device_config_t dev_conf = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000
    };
    err = i2c_master_bus_add_device(h->bus, &dev_conf, &h->dev);
    if (err != ESP_OK) {
        return err;
    }

    // Wake up device by clearing sleep bit
    err = write_reg(h, MPU6050_REG_PWR_MGMT_1, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to wake MPU6050: %s", esp_err_to_name(err));
        return err;
    }

    // Set sample rate divider (1 kHz / (1 + divider) = sample rate)
    err = write_reg(h, MPU6050_REG_SMPLRT_DIV, 7);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set sample rate: %s", esp_err_to_name(err));
        return err;
    }

    // Configure DLPF to 42 Hz
    err = write_reg(h, MPU6050_REG_CONFIG, 3);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set DLPF: %s", esp_err_to_name(err));
        return err;
    }

    // Set gyro range to ±250 deg/s
    err = write_reg(h, MPU6050_REG_GYRO_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set gyro range: %s", esp_err_to_name(err));
        return err;
    }
    h->gyro_res = 250.0f / 32768.0f;

    // Set accel range to ±2g
    err = write_reg(h, MPU6050_REG_ACCEL_CONFIG, 0x00);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to set accel range: %s", esp_err_to_name(err));
        return err;
    }
    h->accel_res = 2.0f / 32768.0f;

    h->initialized = true;
    ESP_LOGI(TAG, "MPU6050 initialized at address 0x%02X", addr);
    return ESP_OK;
}

esp_err_t mpu6050_deinit(mpu6050_handle_t *h)
{
    if (!h || !h->initialized) {
        return ESP_ERR_INVALID_STATE;
    }
    esp_err_t err = i2c_master_bus_rm_device(h->dev);
    if (err == ESP_OK) {
        h->initialized = false;
    }
    return err;
}

bool mpu6050_test_connection(mpu6050_handle_t *h)
{
    uint8_t who_am_i;
    if (read_regs(h, 0x75, &who_am_i, 1) != ESP_OK) {
        return false;
    }
    return (who_am_i == MPU6050_I2C_ADDR);
}

esp_err_t mpu6050_read_accel(mpu6050_handle_t *h, float *ax, float *ay, float *az)
{
    uint8_t buf[6];
    esp_err_t err = read_regs(h, MPU6050_REG_ACCEL_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }
    int16_t raw_x = (buf[0] << 8) | buf[1];
    int16_t raw_y = (buf[2] << 8) | buf[3];
    int16_t raw_z = (buf[4] << 8) | buf[5];
    *ax = raw_x * h->accel_res;
    *ay = raw_y * h->accel_res;
    *az = raw_z * h->accel_res;
    return ESP_OK;
}

esp_err_t mpu6050_read_gyro(mpu6050_handle_t *h, float *gx, float *gy, float *gz)
{
    uint8_t buf[6];
    esp_err_t err = read_regs(h, MPU6050_REG_GYRO_XOUT_H, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }
    int16_t raw_x = (buf[0] << 8) | buf[1];
    int16_t raw_y = (buf[2] << 8) | buf[3];
    int16_t raw_z = (buf[4] << 8) | buf[5];
    *gx = raw_x * h->gyro_res;
    *gy = raw_y * h->gyro_res;
    *gz = raw_z * h->gyro_res;
    return ESP_OK;
}

esp_err_t mpu6050_read_temp(mpu6050_handle_t *h, float *temp)
{
    uint8_t buf[2];
    esp_err_t err = read_regs(h, MPU6050_REG_TEMP_OUT_H, buf, sizeof(buf));
    if (err != ESP_OK) {
        return err;
    }
    int16_t raw_t = (buf[0] << 8) | buf[1];
    *temp = (raw_t / 340.0f) + 36.53f;
    return ESP_OK;
}

esp_err_t mpu6050_read_all(mpu6050_handle_t *h, mpu6050_data_t *data)
{
    if (!h || !data || !h->initialized) {
        return ESP_ERR_INVALID_ARG;
    }
    esp_err_t err = mpu6050_read_accel(h, &data->accel_x, &data->accel_y, &data->accel_z);
    if (err != ESP_OK) return err;
    err = mpu6050_read_gyro(h, &data->gyro_x, &data->gyro_y, &data->gyro_z);
    if (err != ESP_OK) return err;
    return mpu6050_read_temp(h, &data->temperature);
}
