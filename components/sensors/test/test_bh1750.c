/**
 * @file test_bh1750.c
 * @brief Unit tests for BH1750 digital light sensor driver
 *
 * This file contains comprehensive unit tests for the BH1750 digital light
 * sensor driver, covering initialization, configuration, measurement modes,
 * MTreg sensitivity adjustment, and light reading functionality.
 */

#include "unity.h"
#include "bh1750.h"
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
 * @defgroup bh1750_tests BH1750 Digital Light Sensor Tests
 * @brief Test cases for BH1750 digital light sensor driver
 * @{
 */

/**
 * @brief Test BH1750 initialization with valid parameters
 *
 * Verifies that sensor initializes correctly with valid I2C port and address,
 * and that all handle fields are properly set to default values.
 */
TEST_CASE("BH1750 init succeeds with valid params", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    esp_err_t err = bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(handle.initialised);
    TEST_ASSERT_NOT_NULL(handle.bus);
    TEST_ASSERT_NOT_NULL(handle.dev);
    TEST_ASSERT_EQUAL(BH1750_DEFAULT_MTREG, handle.mtreg);
    TEST_ASSERT_EQUAL(UNCONFIGURED, handle.mode);
}

/**
 * @brief Test BH1750 initialization fails with null handle
 *
 * Verifies that initialization properly rejects null sensor handle
 * and returns appropriate error code for invalid argument.
 */
TEST_CASE("BH1750 init fails with null handle", "[bh1750]") {
    esp_err_t err = bh1750_init(NULL, I2C_NUM_0, BH1750_I2C_ADDR);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test BH1750 mode setting with valid continuous high resolution mode
 *
 * Verifies that measurement mode is set correctly and stored in handle,
 * with proper I2C command transmission to the sensor.
 */
TEST_CASE("BH1750 set mode continuous high res succeeds", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    
    stub_transmit_ret = ESP_OK;
    
    esp_err_t err = bh1750_set_mode(&handle, CONTINUOUS_HIGH_RES_MODE);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(CONTINUOUS_HIGH_RES_MODE, handle.mode);
}

/**
 * @brief Test BH1750 mode setting with valid one-time low resolution mode
 *
 * Verifies that one-time measurement mode is configured correctly
 * and mode value is properly stored in the sensor handle.
 */
TEST_CASE("BH1750 set mode one-time low res succeeds", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    
    stub_transmit_ret = ESP_OK;
    
    esp_err_t err = bh1750_set_mode(&handle, ONE_TIME_LOW_RES_MODE);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(ONE_TIME_LOW_RES_MODE, handle.mode);
}

/**
 * @brief Test BH1750 mode setting fails with uninitialized handle
 *
 * Verifies that mode setting properly validates initialization state
 * and rejects uninitialized handles with appropriate error.
 */
TEST_CASE("BH1750 set mode fails with uninitialized handle", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    handle.initialised = false;
    
    esp_err_t err = bh1750_set_mode(&handle, CONTINUOUS_HIGH_RES_MODE);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test BH1750 sensor reset functionality
 *
 * Verifies that reset command is sent correctly to clear
 * the sensor's data register and restore default state.
 */
TEST_CASE("BH1750 reset succeeds with initialized handle", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    
    stub_transmit_ret = ESP_OK;
    
    esp_err_t err = bh1750_reset(&handle);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test BH1750 reset fails with uninitialized handle
 *
 * Verifies that reset function properly validates initialization state
 * and rejects uninitialized handles with appropriate error.
 */
TEST_CASE("BH1750 reset fails with uninitialized handle", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    handle.initialised = false;
    
    esp_err_t err = bh1750_reset(&handle);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test BH1750 MTreg setting with valid minimum value
 *
 * Verifies that MTreg (measurement time register) can be set to
 * minimum allowed value and proper two-byte command sequence is sent.
 */
TEST_CASE("BH1750 set mtreg accepts minimum value", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    
    stub_transmit_ret = ESP_OK;
    
    esp_err_t err = bh1750_set_mtreg(&handle, BH1750_MTREG_MIN);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(BH1750_MTREG_MIN, handle.mtreg);
}

/**
 * @brief Test BH1750 MTreg setting with valid maximum value
 *
 * Verifies that MTreg can be set to maximum allowed value
 * and sensitivity adjustment is properly configured.
 */
TEST_CASE("BH1750 set mtreg accepts maximum value", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    
    stub_transmit_ret = ESP_OK;
    
    esp_err_t err = bh1750_set_mtreg(&handle, BH1750_MTREG_MAX);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(BH1750_MTREG_MAX, handle.mtreg);
}

/**
 * @brief Test BH1750 MTreg setting rejects values below minimum
 *
 * Verifies that MTreg validation properly rejects values below
 * the allowed range and returns invalid argument error.
 */
TEST_CASE("BH1750 set mtreg rejects value below minimum", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    
    esp_err_t err = bh1750_set_mtreg(&handle, BH1750_MTREG_MIN - 1);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
    TEST_ASSERT_EQUAL(BH1750_DEFAULT_MTREG, handle.mtreg); /* Should remain unchanged */
}

/**
 * @brief Test BH1750 MTreg setting rejects values above maximum
 *
 * Verifies that MTreg validation properly rejects values above
 * the allowed range and returns invalid argument error.
 */
TEST_CASE("BH1750 set mtreg rejects value above maximum", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    
    esp_err_t err = bh1750_set_mtreg(&handle, BH1750_MTREG_MAX + 1);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
    TEST_ASSERT_EQUAL(BH1750_DEFAULT_MTREG, handle.mtreg); /* Should remain unchanged */
}

/**
 * @brief Test BH1750 MTreg setting fails with uninitialized handle
 *
 * Verifies that MTreg setting properly validates initialization state
 * and rejects uninitialized handles with appropriate error.
 */
TEST_CASE("BH1750 set mtreg fails with uninitialized handle", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    handle.initialised = false;
    
    esp_err_t err = bh1750_set_mtreg(&handle, BH1750_DEFAULT_MTREG);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test BH1750 light reading in continuous mode
 *
 * Verifies that light level is read correctly in continuous mode
 * without triggering additional measurement commands.
 */
TEST_CASE("BH1750 read light continuous mode succeeds", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    handle.mode = CONTINUOUS_HIGH_RES_MODE;
    
    stub_receive_ret = ESP_OK;
    /* Mock sensor reading: 0x1234 raw = 4660 decimal */
    fake_buf[0] = 0x12;
    fake_buf[1] = 0x34;
    
    float lux;
    esp_err_t err = bh1750_read_light(&handle, &lux);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Expected lux = 4660 / (1.2 * (69/69)) = 4660 / 1.2 ≈ 3883.33 */
    float expected_lux = 4660.0f / BH1750_CONV_FACTOR;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_lux, lux);
}

/**
 * @brief Test BH1750 light reading in one-time mode
 *
 * Verifies that light level is read correctly in one-time mode
 * with automatic measurement triggering before data reading.
 */
TEST_CASE("BH1750 read light one-time mode succeeds", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    handle.mode = ONE_TIME_HIGH_RES_MODE;
    
    stub_transmit_ret = ESP_OK; /* For triggering measurement */
    stub_receive_ret = ESP_OK;
    /* Mock sensor reading: 0x0500 raw = 1280 decimal */
    fake_buf[0] = 0x05;
    fake_buf[1] = 0x00;
    
    float lux;
    esp_err_t err = bh1750_read_light(&handle, &lux);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Expected lux = 1280 / 1.2 ≈ 1066.67 */
    float expected_lux = 1280.0f / BH1750_CONV_FACTOR;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_lux, lux);
}

/**
 * @brief Test BH1750 light reading with custom MTreg value
 *
 * Verifies that lux calculation accounts for custom MTreg sensitivity
 * adjustment and applies the correct conversion factor.
 */
TEST_CASE("BH1750 read light with custom mtreg", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    handle.mode = CONTINUOUS_HIGH_RES_MODE;
    handle.mtreg = 138; /* Double the default sensitivity */
    
    stub_receive_ret = ESP_OK;
    /* Mock sensor reading: 0x1000 raw = 4096 decimal */
    fake_buf[0] = 0x10;
    fake_buf[1] = 0x00;
    
    float lux;
    esp_err_t err = bh1750_read_light(&handle, &lux);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Expected lux = 4096 / (1.2 * (69/138)) = 4096 / (1.2 * 0.5) = 4096 / 0.6 ≈ 6826.67 */
    float mtreg_factor = (float)BH1750_DEFAULT_MTREG / (float)handle.mtreg;
    float expected_lux = 4096.0f / (BH1750_CONV_FACTOR * mtreg_factor);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_lux, lux);
}

/**
 * @brief Test BH1750 light reading fails with uninitialized handle
 *
 * Verifies that light reading properly validates initialization state
 * and rejects uninitialized handles with appropriate error.
 */
TEST_CASE("BH1750 read light fails with uninitialized handle", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    handle.initialised = false;
    
    float lux;
    esp_err_t err = bh1750_read_light(&handle, &lux);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test BH1750 light reading fails with null lux pointer
 *
 * Verifies that light reading properly validates output pointer
 * and rejects null lux pointer with appropriate error.
 */
TEST_CASE("BH1750 read light fails with null lux pointer", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    handle.mode = CONTINUOUS_HIGH_RES_MODE;
    
    esp_err_t err = bh1750_read_light(&handle, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test BH1750 I2C communication error handling in mode setting
 *
 * Verifies that I2C communication errors during mode setting are
 * properly detected and appropriate error codes are returned.
 */
TEST_CASE("BH1750 handles I2C error in set mode", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    
    stub_transmit_ret = ESP_ERR_TIMEOUT;
    
    esp_err_t err = bh1750_set_mode(&handle, CONTINUOUS_HIGH_RES_MODE);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @brief Test BH1750 I2C communication error handling in MTreg setting
 *
 * Verifies that I2C communication errors during MTreg configuration
 * are properly detected and appropriate error codes are returned.
 */
TEST_CASE("BH1750 handles I2C error in set mtreg", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    
    stub_transmit_ret = ESP_ERR_TIMEOUT;
    
    esp_err_t err = bh1750_set_mtreg(&handle, 100);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
    TEST_ASSERT_EQUAL(BH1750_DEFAULT_MTREG, handle.mtreg); /* Should remain unchanged */
}

/**
 * @brief Test BH1750 I2C communication error handling in light reading
 *
 * Verifies that I2C communication errors during data reading are
 * properly detected and appropriate error codes are returned.
 */
TEST_CASE("BH1750 handles I2C error in read light", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    handle.mode = CONTINUOUS_HIGH_RES_MODE;
    
    stub_receive_ret = ESP_ERR_TIMEOUT;
    
    float lux;
    esp_err_t err = bh1750_read_light(&handle, &lux);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @brief Test BH1750 light reading with high resolution mode 2
 *
 * Verifies that high resolution mode 2 (0.5 lux resolution) works
 * correctly and provides accurate light level measurements.
 */
TEST_CASE("BH1750 read light high res mode 2 succeeds", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    handle.mode = CONTINUOUS_HIGH_RES_MODE_2;
    
    stub_receive_ret = ESP_OK;
    /* Mock sensor reading: 0x0800 raw = 2048 decimal */
    fake_buf[0] = 0x08;
    fake_buf[1] = 0x00;
    
    float lux;
    esp_err_t err = bh1750_read_light(&handle, &lux);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Expected lux = 2048 / 1.2 ≈ 1706.67 */
    float expected_lux = 2048.0f / BH1750_CONV_FACTOR;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_lux, lux);
}

/**
 * @brief Test BH1750 zero light reading (complete darkness)
 *
 * Verifies that zero raw value from sensor correctly converts
 * to zero lux, representing complete darkness condition.
 */
TEST_CASE("BH1750 read light zero value succeeds", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    handle.mode = CONTINUOUS_HIGH_RES_MODE;
    
    stub_receive_ret = ESP_OK;
    /* Mock sensor reading: 0x0000 raw = 0 decimal (darkness) */
    fake_buf[0] = 0x00;
    fake_buf[1] = 0x00;
    
    float lux;
    esp_err_t err = bh1750_read_light(&handle, &lux);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(0.01f, 0.0f, lux);
}

/**
 * @brief Test BH1750 maximum light reading (sensor saturation)
 *
 * Verifies that maximum raw value from sensor correctly converts
 * to maximum lux, representing bright light or sensor saturation.
 */
TEST_CASE("BH1750 read light maximum value succeeds", "[bh1750]") {
    bh1750_handle_t handle = { 0 };
    bh1750_init(&handle, I2C_NUM_0, BH1750_I2C_ADDR);
    handle.mode = CONTINUOUS_HIGH_RES_MODE;
    
    stub_receive_ret = ESP_OK;
    /* Mock sensor reading: 0xFFFF raw = 65535 decimal (maximum) */
    fake_buf[0] = 0xFF;
    fake_buf[1] = 0xFF;
    
    float lux;
    esp_err_t err = bh1750_read_light(&handle, &lux);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Expected lux = 65535 / 1.2 ≈ 54612.5 */
    float expected_lux = 65535.0f / BH1750_CONV_FACTOR;
    TEST_ASSERT_FLOAT_WITHIN(1.0f, expected_lux, lux);
}

/**
 * @} (end of bh1750_tests group)
 */
