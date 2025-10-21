/**
 * @file test_bme280.c
 * @brief Unit tests for BME280 temperature/pressure/humidity sensor driver
 *
 * This file contains comprehensive unit tests for the BME280 sensor driver,
 * covering initialization, configuration, calibration reading, data conversion,
 * and error handling scenarios.
 */

#include "unity.h"
#include "bme280.h"
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
 * @defgroup bme280_tests BME280 Temperature/Pressure/Humidity Sensor Tests
 * @brief Test cases for BME280 sensor driver
 * @{
 */

/**
 * @brief Test BME280 default configuration
 *
 * Verifies that default configuration values are set correctly
 * with appropriate oversampling and mode settings.
 */
TEST_CASE("BME280 get default config", "[bme280]") {
    bme280_config_t config;
    bme280_get_default_config(&config);
    
    TEST_ASSERT_EQUAL(BME280_OSRS_X1, config.osrs_t);
    TEST_ASSERT_EQUAL(BME280_OSRS_X1, config.osrs_p);
    TEST_ASSERT_EQUAL(BME280_OSRS_X1, config.osrs_h);
    TEST_ASSERT_EQUAL(BME280_FILTER_OFF, config.filter);
    TEST_ASSERT_EQUAL(BME280_STANDBY_1000, config.standby_time);
    TEST_ASSERT_EQUAL(BME280_MODE_SLEEP, config.mode);
}

/**
 * @brief Test BME280 initialization with valid parameters
 *
 * Verifies that sensor initializes correctly, reads chip ID,
 * loads calibration data, and applies configuration.
 */
TEST_CASE("BME280 init succeeds with valid params", "[bme280]") {
    bme280_handle_t handle = { 0 };
    bme280_config_t config;
    bme280_get_default_config(&config);
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock chip ID response */
    fake_buf[0] = BME280_CHIP_ID;  /* Correct chip ID */
    
    /* Mock calibration data (simplified) */
    memset(fake_buf, 0x01, 26);  /* First calibration block */
    
    esp_err_t err = bme280_init(&handle, I2C_NUM_0, BME280_I2C_ADDR, &config);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(handle.initialised);
    TEST_ASSERT_NOT_NULL(handle.dev);
}

/**
 * @brief Test BME280 initialization fails with null handle
 *
 * Verifies that initialization properly rejects null sensor handle
 * and returns appropriate error code.
 */
TEST_CASE("BME280 init fails with null handle", "[bme280]") {
    bme280_config_t config;
    bme280_get_default_config(&config);
    
    esp_err_t err = bme280_init(NULL, I2C_NUM_0, BME280_I2C_ADDR, &config);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test BME280 initialization fails with null config
 *
 * Verifies that initialization properly validates config pointer
 * and returns appropriate error for null input.
 */
TEST_CASE("BME280 init fails with null config", "[bme280]") {
    bme280_handle_t handle = { 0 };
    
    esp_err_t err = bme280_init(&handle, I2C_NUM_0, BME280_I2C_ADDR, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test BME280 initialization fails with wrong chip ID
 *
 * Verifies that initialization fails when chip ID doesn't match
 * expected BME280 chip ID, indicating wrong sensor or connection issues.
 */
TEST_CASE("BME280 init fails with wrong chip ID", "[bme280]") {
    bme280_handle_t handle = { 0 };
    bme280_config_t config;
    bme280_get_default_config(&config);
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock wrong chip ID */
    fake_buf[0] = 0xFF;  /* Wrong chip ID */
    
    esp_err_t err = bme280_init(&handle, I2C_NUM_0, BME280_I2C_ADDR, &config);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, err);
}

/**
 * @brief Test BME280 initialization fails with I2C error
 *
 * Verifies that I2C communication errors during initialization
 * are properly handled and appropriate error codes returned.
 */
TEST_CASE("BME280 init handles I2C communication error", "[bme280]") {
    bme280_handle_t handle = { 0 };
    bme280_config_t config;
    bme280_get_default_config(&config);
    
    stub_transmit_ret = ESP_ERR_TIMEOUT;  /* Simulate I2C error */
    stub_receive_ret = ESP_ERR_TIMEOUT;
    
    esp_err_t err = bme280_init(&handle, I2C_NUM_0, BME280_I2C_ADDR, &config);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @brief Test BME280 deinitialization
 *
 * Verifies that deinitialization properly removes I2C device
 * and resets initialization flag.
 */
TEST_CASE("BME280 deinit succeeds with initialized handle", "[bme280]") {
    bme280_handle_t handle = { 0 };
    
    /* Setup initialized handle */
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    esp_err_t err = bme280_deinit(&handle);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FALSE(handle.initialised);
}

/**
 * @brief Test BME280 deinit fails with null handle
 *
 * Verifies that deinitialization properly validates handle pointer
 * and returns appropriate error for null input.
 */
TEST_CASE("BME280 deinit fails with null handle", "[bme280]") {
    esp_err_t err = bme280_deinit(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test BME280 forced mode measurement
 *
 * Verifies that forced mode measurement works correctly,
 * triggering single measurement and returning to sleep mode.
 */
TEST_CASE("BME280 forced mode measurement succeeds", "[bme280]") {
    bme280_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    /* Setup mock calibration data */
    handle.calib.dig_T1 = 27504;
    handle.calib.dig_T2 = 26435;
    handle.calib.dig_T3 = -1000;
    handle.calib.dig_P1 = 36477;
    handle.calib.dig_P2 = -10685;
    handle.calib.dig_P3 = 3024;
    handle.calib.dig_P4 = 2855;
    handle.calib.dig_P5 = 140;
    handle.calib.dig_P6 = -7;
    handle.calib.dig_P7 = 15500;
    handle.calib.dig_P8 = -14600;
    handle.calib.dig_P9 = 6000;
    handle.calib.dig_H1 = 75;
    handle.calib.dig_H2 = 361;
    handle.calib.dig_H3 = 0;
    handle.calib.dig_H4 = 328;
    handle.calib.dig_H5 = 0;
    handle.calib.dig_H6 = 30;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock measurement data (temperature ~25°C, pressure ~1013hPa, humidity ~50%) */
    fake_buf[0] = 0x65;  /* Press MSB */
    fake_buf[1] = 0x5C;  /* Press LSB */
    fake_buf[2] = 0x00;  /* Press XLSB */
    fake_buf[3] = 0x7E;  /* Temp MSB */
    fake_buf[4] = 0x90;  /* Temp LSB */
    fake_buf[5] = 0x00;  /* Temp XLSB */
    fake_buf[6] = 0x66;  /* Hum MSB */
    fake_buf[7] = 0x7F;  /* Hum LSB */
    
    float temperature, pressure, humidity;
    esp_err_t err = bme280_read_forced(&handle, &temperature, &pressure, &humidity);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Check values are within reasonable ranges */
    TEST_ASSERT_FLOAT_WITHIN(10.0f, 25.0f, temperature);
    TEST_ASSERT_FLOAT_WITHIN(100.0f, 1013.25f, pressure);
    TEST_ASSERT_FLOAT_WITHIN(20.0f, 50.0f, humidity);
}

/**
 * @brief Test BME280 forced mode with uninitialized handle
 *
 * Verifies that forced mode measurement fails appropriately
 * when handle is not initialized.
 */
TEST_CASE("BME280 forced mode fails with uninitialized handle", "[bme280]") {
    bme280_handle_t handle = { 0 };
    handle.initialised = false;
    
    float temperature, pressure, humidity;
    esp_err_t err = bme280_read_forced(&handle, &temperature, &pressure, &humidity);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test BME280 forced mode with null output pointers
 *
 * Verifies that measurement works when only some output pointers
 * are provided (others can be NULL).
 */
TEST_CASE("BME280 forced mode works with null output pointers", "[bme280]") {
    bme280_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    /* Setup minimal calibration */
    handle.calib.dig_T1 = 27504;
    handle.calib.dig_T2 = 26435;
    handle.calib.dig_T3 = -1000;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock measurement data */
    fake_buf[0] = 0x65;  /* Press MSB */
    fake_buf[1] = 0x5C;  /* Press LSB */
    fake_buf[2] = 0x00;  /* Press XLSB */
    fake_buf[3] = 0x7E;  /* Temp MSB */
    fake_buf[4] = 0x90;  /* Temp LSB */
    fake_buf[5] = 0x00;  /* Temp XLSB */
    fake_buf[6] = 0x66;  /* Hum MSB */
    fake_buf[7] = 0x7F;  /* Hum LSB */
    
    float temperature;
    esp_err_t err = bme280_read_forced(&handle, &temperature, NULL, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(50.0f, 25.0f, temperature);
}

/**
 * @brief Test BME280 forced mode I2C communication error handling
 *
 * Verifies that I2C errors during measurement are properly handled
 * and appropriate error codes are returned.
 */
TEST_CASE("BME280 forced mode handles I2C error", "[bme280]") {
    bme280_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_ERR_TIMEOUT;  /* Simulate I2C error */
    stub_receive_ret = ESP_OK;
    
    float temperature, pressure, humidity;
    esp_err_t err = bme280_read_forced(&handle, &temperature, &pressure, &humidity);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @brief Test BME280 temperature-only reading
 *
 * Verifies that temperature-only convenience function works correctly
 * by internally calling forced mode with NULL pressure/humidity pointers.
 */
TEST_CASE("BME280 read temperature only succeeds", "[bme280]") {
    bme280_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    /* Setup minimal calibration */
    handle.calib.dig_T1 = 27504;
    handle.calib.dig_T2 = 26435;
    handle.calib.dig_T3 = -1000;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock temperature data */
    fake_buf[3] = 0x7E;  /* Temp MSB */
    fake_buf[4] = 0x90;  /* Temp LSB */
    fake_buf[5] = 0x00;  /* Temp XLSB */
    
    float temperature;
    esp_err_t err = bme280_read_temperature(&handle, &temperature);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(50.0f, 25.0f, temperature);
}

/**
 * @brief Test BME280 pressure-only reading
 *
 * Verifies that pressure-only convenience function works correctly
 * by internally calling forced mode with NULL temperature/humidity pointers.
 */
TEST_CASE("BME280 read pressure only succeeds", "[bme280]") {
    bme280_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    /* Setup calibration for pressure calculation */
    handle.calib.dig_T1 = 27504;
    handle.calib.dig_T2 = 26435;
    handle.calib.dig_T3 = -1000;
    handle.calib.dig_P1 = 36477;
    handle.calib.dig_P2 = -10685;
    handle.calib.dig_P3 = 3024;
    handle.calib.dig_P4 = 2855;
    handle.calib.dig_P5 = 140;
    handle.calib.dig_P6 = -7;
    handle.calib.dig_P7 = 15500;
    handle.calib.dig_P8 = -14600;
    handle.calib.dig_P9 = 6000;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock pressure data */
    fake_buf[0] = 0x65;  /* Press MSB */
    fake_buf[1] = 0x5C;  /* Press LSB */
    fake_buf[2] = 0x00;  /* Press XLSB */
    fake_buf[3] = 0x7E;  /* Temp MSB (needed for pressure calc) */
    fake_buf[4] = 0x90;  /* Temp LSB */
    fake_buf[5] = 0x00;  /* Temp XLSB */
    
    float pressure;
    esp_err_t err = bme280_read_pressure(&handle, &pressure);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(500.0f, 1013.25f, pressure);
}

/**
 * @brief Test BME280 humidity-only reading
 *
 * Verifies that humidity-only convenience function works correctly
 * by internally calling forced mode with NULL temperature/pressure pointers.
 */
TEST_CASE("BME280 read humidity only succeeds", "[bme280]") {
    bme280_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    /* Setup calibration for humidity calculation */
    handle.calib.dig_T1 = 27504;
    handle.calib.dig_T2 = 26435;
    handle.calib.dig_T3 = -1000;
    handle.calib.dig_H1 = 75;
    handle.calib.dig_H2 = 361;
    handle.calib.dig_H3 = 0;
    handle.calib.dig_H4 = 328;
    handle.calib.dig_H5 = 0;
    handle.calib.dig_H6 = 30;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock humidity data */
    fake_buf[3] = 0x7E;  /* Temp MSB (needed for humidity calc) */
    fake_buf[4] = 0x90;  /* Temp LSB */
    fake_buf[5] = 0x00;  /* Temp XLSB */
    fake_buf[6] = 0x66;  /* Hum MSB */
    fake_buf[7] = 0x7F;  /* Hum LSB */
    
    float humidity;
    esp_err_t err = bme280_read_humidity(&handle, &humidity);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(30.0f, 50.0f, humidity);
}

/**
 * @brief Test BME280 edge case temperature conversion
 *
 * Verifies temperature conversion accuracy at measurement range edges
 * using extreme raw values.
 */
TEST_CASE("BME280 temperature conversion edge cases", "[bme280]") {
    bme280_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    /* Setup calibration */
    handle.calib.dig_T1 = 27504;
    handle.calib.dig_T2 = 26435;
    handle.calib.dig_T3 = -1000;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Test minimum temperature raw value */
    fake_buf[3] = 0x00;  /* Temp MSB */
    fake_buf[4] = 0x00;  /* Temp LSB */
    fake_buf[5] = 0x00;  /* Temp XLSB */
    
    float temperature;
    esp_err_t err = bme280_read_temperature(&handle, &temperature);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    /* Should be a very low temperature */
    TEST_ASSERT_TRUE(temperature < -20.0f);
}

/**
 * @brief Test BME280 parameter validation in convenience functions
 *
 * Verifies that convenience functions properly validate input parameters
 * and return appropriate errors for null pointers.
 */
TEST_CASE("BME280 convenience functions validate parameters", "[bme280]") {
    bme280_handle_t handle = { 0 };
    float value;
    
    /* Test null handle */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bme280_read_temperature(NULL, &value));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bme280_read_pressure(NULL, &value));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bme280_read_humidity(NULL, &value));
    
    /* Test null output pointer */
    handle.initialised = true;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bme280_read_temperature(&handle, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bme280_read_pressure(&handle, NULL));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, bme280_read_humidity(&handle, NULL));
}

/**
 * @} (end of bme280_tests group)
 */