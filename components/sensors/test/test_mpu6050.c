/**
 * @file test_mpu6050.c
 * @brief Unit tests for MPU6050 6-axis accelerometer/gyroscope sensor driver
 *
 * This file contains comprehensive unit tests for the MPU6050 sensor driver,
 * covering initialization, configuration, data reading, conversion,
 * and error handling scenarios.
 */

#include "unity.h"
#include "mpu6050.h"
#include "esp_err.h"
#include <string.h>
#include <math.h>

/**
 * @brief External stub variables from i2c_test_stubs.c
 */
extern esp_err_t   stub_transmit_ret;  /**< I2C transmit return value */
extern esp_err_t   stub_receive_ret;   /**< I2C receive return value */
extern uint8_t     fake_buf[];         /**< Fake I2C data buffer */

/**
 * @defgroup mpu6050_tests MPU6050 6-Axis Sensor Tests
 * @brief Test cases for MPU6050 accelerometer/gyroscope driver
 * @{
 */

/**
 * @brief Test MPU6050 initialization with valid parameters
 *
 * Verifies that sensor initializes correctly, configures registers,
 * and sets up resolution values for accelerometer and gyroscope.
 */
TEST_CASE("MPU6050 init succeeds with valid params", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    esp_err_t err = mpu6050_init(&handle, I2C_NUM_0, 0x68);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(handle.initialized);
    TEST_ASSERT_NOT_NULL(handle.dev);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 250.0f / 32768.0f, handle.gyro_res);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.0f / 32768.0f, handle.accel_res);
}

/**
 * @brief Test MPU6050 initialization fails with null handle
 *
 * Verifies that initialization properly rejects null sensor handle
 * and returns appropriate error code.
 */
TEST_CASE("MPU6050 init fails with null handle", "[mpu6050]") {
    esp_err_t err = mpu6050_init(NULL, I2C_NUM_0, 0x68);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test MPU6050 initialization fails with I2C error
 *
 * Verifies that I2C communication errors during initialization
 * are properly handled and appropriate error codes returned.
 */
TEST_CASE("MPU6050 init handles I2C communication error", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    
    stub_transmit_ret = ESP_ERR_TIMEOUT;  /* Simulate I2C error */
    
    esp_err_t err = mpu6050_init(&handle, I2C_NUM_0, 0x68);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @brief Test MPU6050 deinitialization
 *
 * Verifies that deinitialization properly removes I2C device
 * and resets initialization flag.
 */
TEST_CASE("MPU6050 deinit succeeds with initialized handle", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    
    /* Setup initialized handle */
    handle.initialized = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    esp_err_t err = mpu6050_deinit(&handle);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FALSE(handle.initialized);
}

/**
 * @brief Test MPU6050 deinit fails with null handle
 *
 * Verifies that deinitialization properly validates handle pointer
 * and returns appropriate error for null input.
 */
TEST_CASE("MPU6050 deinit fails with null handle", "[mpu6050]") {
    esp_err_t err = mpu6050_deinit(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test MPU6050 data reading with valid sensor data
 *
 * Verifies that sensor data is read correctly and converted to proper
 * physical units using the configured resolution values.
 */
TEST_CASE("MPU6050 read data succeeds", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    handle.initialized = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    handle.accel_res = 2.0f / 32768.0f;  /* ±2g range */
    handle.gyro_res = 250.0f / 32768.0f;  /* ±250 deg/s range */
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock sensor data (14 bytes starting from accel registers) */
    /* Accel X: 0x4000 (~1g), Y: 0x0000 (~0g), Z: 0x8000 (~-1g) */
    /* Temp: 0x0000 (~36.53°C), Gyro X,Y,Z: 0x1000 (~45 deg/s each) */
    fake_buf[0] = 0x40;  /* Accel X MSB */
    fake_buf[1] = 0x00;  /* Accel X LSB */
    fake_buf[2] = 0x00;  /* Accel Y MSB */
    fake_buf[3] = 0x00;  /* Accel Y LSB */
    fake_buf[4] = 0x80;  /* Accel Z MSB */
    fake_buf[5] = 0x00;  /* Accel Z LSB */
    fake_buf[6] = 0x00;  /* Temp MSB */
    fake_buf[7] = 0x00;  /* Temp LSB */
    fake_buf[8] = 0x10;  /* Gyro X MSB */
    fake_buf[9] = 0x00;  /* Gyro X LSB */
    fake_buf[10] = 0x10; /* Gyro Y MSB */
    fake_buf[11] = 0x00; /* Gyro Y LSB */
    fake_buf[12] = 0x10; /* Gyro Z MSB */
    fake_buf[13] = 0x00; /* Gyro Z LSB */
    
    mpu6050_data_t data;
    esp_err_t err = mpu6050_read_data(&handle, &data);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Check accelerometer values (approximately 1g, 0g, -1g) */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, data.accel_x);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, data.accel_y);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -1.0f, data.accel_z);
    
    /* Check gyroscope values (approximately 45 deg/s each) */
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 45.0f, data.gyro_x);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 45.0f, data.gyro_y);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 45.0f, data.gyro_z);
    
    /* Check temperature (approximately 36.53°C) */
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 36.53f, data.temperature);
}

/**
 * @brief Test MPU6050 data reading with uninitialized handle
 *
 * Verifies that data reading fails appropriately when handle
 * is not initialized.
 */
TEST_CASE("MPU6050 read data fails with uninitialized handle", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    handle.initialized = false;
    
    mpu6050_data_t data;
    esp_err_t err = mpu6050_read_data(&handle, &data);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test MPU6050 data reading with null handle
 *
 * Verifies that data reading properly validates handle pointer
 * and returns appropriate error for null input.
 */
TEST_CASE("MPU6050 read data fails with null handle", "[mpu6050]") {
    mpu6050_data_t data;
    esp_err_t err = mpu6050_read_data(NULL, &data);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test MPU6050 data reading with null data pointer
 *
 * Verifies that data reading properly validates data pointer
 * and returns appropriate error for null input.
 */
TEST_CASE("MPU6050 read data fails with null data pointer", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    handle.initialized = true;
    
    esp_err_t err = mpu6050_read_data(&handle, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test MPU6050 data reading I2C communication error handling
 *
 * Verifies that I2C errors during data reading are properly handled
 * and appropriate error codes are returned.
 */
TEST_CASE("MPU6050 read data handles I2C error", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    handle.initialized = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_ERR_TIMEOUT;  /* Simulate I2C error */
    
    mpu6050_data_t data;
    esp_err_t err = mpu6050_read_data(&handle, &data);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @brief Test MPU6050 accelerometer-only reading
 *
 * Verifies that accelerometer-only convenience function works correctly
 * by reading just the accelerometer data.
 */
TEST_CASE("MPU6050 read accelerometer only succeeds", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    handle.initialized = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    handle.accel_res = 2.0f / 32768.0f;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock accelerometer data */
    fake_buf[0] = 0x40;  /* Accel X MSB */
    fake_buf[1] = 0x00;  /* Accel X LSB */
    fake_buf[2] = 0x00;  /* Accel Y MSB */
    fake_buf[3] = 0x00;  /* Accel Y LSB */
    fake_buf[4] = 0x80;  /* Accel Z MSB */
    fake_buf[5] = 0x00;  /* Accel Z LSB */
    
    float accel_x, accel_y, accel_z;
    esp_err_t err = mpu6050_read_accel(&handle, &accel_x, &accel_y, &accel_z);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 1.0f, accel_x);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, accel_y);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -1.0f, accel_z);
}

/**
 * @brief Test MPU6050 gyroscope-only reading
 *
 * Verifies that gyroscope-only convenience function works correctly
 * by reading just the gyroscope data.
 */
TEST_CASE("MPU6050 read gyroscope only succeeds", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    handle.initialized = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    handle.gyro_res = 250.0f / 32768.0f;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock gyroscope data */
    fake_buf[0] = 0x10;  /* Gyro X MSB */
    fake_buf[1] = 0x00;  /* Gyro X LSB */
    fake_buf[2] = 0x20;  /* Gyro Y MSB */
    fake_buf[3] = 0x00;  /* Gyro Y LSB */
    fake_buf[4] = 0x30;  /* Gyro Z MSB */
    fake_buf[5] = 0x00;  /* Gyro Z LSB */
    
    float gyro_x, gyro_y, gyro_z;
    esp_err_t err = mpu6050_read_gyro(&handle, &gyro_x, &gyro_y, &gyro_z);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Check that values are different (indicating correct channel separation) */
    TEST_ASSERT_NOT_EQUAL(gyro_x, gyro_y);
    TEST_ASSERT_NOT_EQUAL(gyro_y, gyro_z);
    TEST_ASSERT_TRUE(gyro_x > 0.0f);
    TEST_ASSERT_TRUE(gyro_y > gyro_x);
    TEST_ASSERT_TRUE(gyro_z > gyro_y);
}

/**
 * @brief Test MPU6050 temperature-only reading
 *
 * Verifies that temperature-only convenience function works correctly
 * by reading just the temperature data.
 */
TEST_CASE("MPU6050 read temperature only succeeds", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    handle.initialized = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock temperature data (0x0800 should give ~45°C) */
    fake_buf[0] = 0x08;  /* Temp MSB */
    fake_buf[1] = 0x00;  /* Temp LSB */
    
    float temperature;
    esp_err_t err = mpu6050_read_temperature(&handle, &temperature);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 45.0f, temperature);
}

/**
 * @brief Test MPU6050 data conversion edge cases
 *
 * Verifies data conversion accuracy at measurement range edges
 * using extreme raw values for accelerometer and gyroscope.
 */
TEST_CASE("MPU6050 data conversion edge cases", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    handle.initialized = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    handle.accel_res = 2.0f / 32768.0f;
    handle.gyro_res = 250.0f / 32768.0f;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Test maximum positive values */
    fake_buf[0] = 0x7F;  /* Accel X MSB (max positive) */
    fake_buf[1] = 0xFF;  /* Accel X LSB */
    fake_buf[2] = 0x80;  /* Accel Y MSB (max negative) */
    fake_buf[3] = 0x00;  /* Accel Y LSB */
    fake_buf[4] = 0x00;  /* Accel Z MSB (zero) */
    fake_buf[5] = 0x00;  /* Accel Z LSB */
    fake_buf[6] = 0x00;  /* Temp MSB */
    fake_buf[7] = 0x00;  /* Temp LSB */
    fake_buf[8] = 0x7F;  /* Gyro X MSB (max positive) */
    fake_buf[9] = 0xFF;  /* Gyro X LSB */
    fake_buf[10] = 0x80; /* Gyro Y MSB (max negative) */
    fake_buf[11] = 0x00; /* Gyro Y LSB */
    fake_buf[12] = 0x00; /* Gyro Z MSB (zero) */
    fake_buf[13] = 0x00; /* Gyro Z LSB */
    
    mpu6050_data_t data;
    esp_err_t err = mpu6050_read_data(&handle, &data);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Check that max positive accel is close to +2g */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 2.0f, data.accel_x);
    /* Check that max negative accel is close to -2g */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, -2.0f, data.accel_y);
    /* Check that zero accel is close to 0g */
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, data.accel_z);
    
    /* Check that max positive gyro is close to +250 deg/s */
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 250.0f, data.gyro_x);
    /* Check that max negative gyro is close to -250 deg/s */
    TEST_ASSERT_FLOAT_WITHIN(10.0f, -250.0f, data.gyro_y);
    /* Check that zero gyro is close to 0 deg/s */
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 0.0f, data.gyro_z);
}

/**
 * @brief Test MPU6050 convenience functions parameter validation
 *
 * Verifies that convenience functions properly validate input parameters
 * and return appropriate errors for null pointers.
 */
TEST_CASE("MPU6050 convenience functions validate parameters", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    float x, y, z, temp;
    
    /* Test null handle */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mpu6050_read_accel(NULL, &x, &y, &z));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mpu6050_read_gyro(NULL, &x, &y, &z));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mpu6050_read_temperature(NULL, &temp));
    
    /* Test null output pointers */
    handle.initialized = true;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mpu6050_read_accel(&handle, NULL, &y, &z));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mpu6050_read_accel(&handle, &x, NULL, &z));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mpu6050_read_accel(&handle, &x, &y, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mpu6050_read_gyro(&handle, NULL, &y, &z));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mpu6050_read_gyro(&handle, &x, NULL, &z));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mpu6050_read_gyro(&handle, &x, &y, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, mpu6050_read_temperature(&handle, NULL));
}

/**
 * @brief Test MPU6050 temperature conversion accuracy
 *
 * Verifies temperature conversion formula using known test values
 * and checks that results are within expected range.
 */
TEST_CASE("MPU6050 temperature conversion accuracy", "[mpu6050]") {
    mpu6050_handle_t handle = { 0 };
    handle.initialized = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Test room temperature (raw value 0x0000 should give ~36.53°C) */
    fake_buf[0] = 0x00;  /* Temp MSB */
    fake_buf[1] = 0x00;  /* Temp LSB */
    
    float temperature;
    esp_err_t err = mpu6050_read_temperature(&handle, &temperature);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 36.53f, temperature);
    
    /* Test higher temperature (raw value 0x1000 should give higher temp) */
    fake_buf[0] = 0x10;  /* Temp MSB */
    fake_buf[1] = 0x00;  /* Temp LSB */
    
    float temperature2;
    err = mpu6050_read_temperature(&handle, &temperature2);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(temperature2 > temperature);
}

/**
 * @} (end of mpu6050_tests group)
 */