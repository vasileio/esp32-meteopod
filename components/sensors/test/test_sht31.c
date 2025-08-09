/**
 * @file test_sht31.c
 * @brief Unit tests for SHT31 temperature/humidity sensor driver
 *
 * This file contains comprehensive unit tests for the SHT31 temperature/humidity
 * sensor driver, covering initialization, configuration, data reading, CRC validation,
 * and error handling scenarios.
 */

#include "unity.h"
#include "sht31.h"
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
 * @defgroup sht31_tests SHT31 Temperature/Humidity Sensor Tests
 * @brief Test cases for SHT31 temperature/humidity sensor driver
 * @{
 */

/**
 * @brief Test SHT31 initialization with valid parameters
 *
 * Verifies that sensor initializes correctly with valid I2C port and address,
 * performs status register validation, and sets all handle fields properly.
 */
TEST_CASE("SHT31 init succeeds with valid params", "[sht31]") {
    sht31_handle_t handle = { 0 };
    
    /* Mock successful status register read (not 0xFFFF) */
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0x00;  /* Status MSB */
    fake_buf[1] = 0x01;  /* Status LSB */
    fake_buf[2] = 0x00;  /* Status CRC (doesn't matter for this test) */
    
    esp_err_t err = sht31_init(&handle, I2C_NUM_0, SHT31_ADDR_DEFAULT);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(handle.initialised);
    TEST_ASSERT_NOT_NULL(handle.bus);
    TEST_ASSERT_NOT_NULL(handle.dev);
}

/**
 * @brief Test SHT31 initialization fails with null handle
 *
 * Verifies that initialization properly rejects null sensor handle
 * and returns appropriate error code for invalid argument.
 */
TEST_CASE("SHT31 init fails with null handle", "[sht31]") {
    esp_err_t err = sht31_init(NULL, I2C_NUM_0, SHT31_ADDR_DEFAULT);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test SHT31 initialization fails with invalid status register
 *
 * Verifies that initialization fails when status register reads 0xFFFF,
 * indicating sensor communication failure or absence.
 */
TEST_CASE("SHT31 init fails with invalid status", "[sht31]") {
    sht31_handle_t handle = { 0 };
    
    /* Mock invalid status register read (0xFFFF) */
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0xFF;  /* Status MSB */
    fake_buf[1] = 0xFF;  /* Status LSB */
    fake_buf[2] = 0x00;  /* Status CRC */
    
    esp_err_t err = sht31_init(&handle, I2C_NUM_0, SHT31_ADDR_DEFAULT);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test SHT31 deinitialization with initialized handle
 *
 * Verifies that deinitialization removes I2C device correctly
 * and resets the initialization flag.
 */
TEST_CASE("SHT31 deinit succeeds with initialized handle", "[sht31]") {
    sht31_handle_t handle = { 0 };
    
    /* Setup initialized handle */
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    esp_err_t err = sht31_deinit(&handle);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FALSE(handle.initialised);
}

/**
 * @brief Test SHT31 deinitialization fails with null handle
 *
 * Verifies that deinitialization properly validates handle pointer
 * and returns appropriate error for null input.
 */
TEST_CASE("SHT31 deinit fails with null handle", "[sht31]") {
    esp_err_t err = sht31_deinit(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test SHT31 deinitialization fails with uninitialized handle
 *
 * Verifies that deinitialization properly validates initialization state
 * and rejects handles that were not previously initialized.
 */
TEST_CASE("SHT31 deinit fails with uninitialized handle", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = false;
    
    esp_err_t err = sht31_deinit(&handle);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test SHT31 status register reading
 *
 * Verifies that status register is read correctly via I2C
 * and 16-bit value is properly assembled from received bytes.
 */
TEST_CASE("SHT31 read status succeeds", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    /* Mock status register response */
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0x12;  /* Status MSB */
    fake_buf[1] = 0x34;  /* Status LSB */
    fake_buf[2] = 0x56;  /* Status CRC */
    
    uint16_t status;
    esp_err_t err = sht31_read_status(&handle, &status);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_HEX16(0x1234, status);
}

/**
 * @brief Test SHT31 status reading parameter validation
 *
 * Verifies that status reading function properly validates input arguments
 * and rejects null pointers with appropriate error codes.
 */
TEST_CASE("SHT31 read status fails with null arguments", "[sht31]") {
    sht31_handle_t handle = { 0 };
    uint16_t status;
    
    /* Test null handle */
    esp_err_t err = sht31_read_status(NULL, &status);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
    
    /* Test null status pointer */
    handle.initialised = true;
    err = sht31_read_status(&handle, NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test SHT31 sensor reset functionality
 *
 * Verifies that soft reset command is sent correctly to restore
 * sensor to default configuration state.
 */
TEST_CASE("SHT31 reset succeeds with initialized handle", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    
    esp_err_t err = sht31_reset(&handle);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test SHT31 reset fails with uninitialized handle
 *
 * Verifies that reset function properly validates initialization state
 * and rejects uninitialized handles with appropriate error.
 */
TEST_CASE("SHT31 reset fails with uninitialized handle", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = false;
    
    esp_err_t err = sht31_reset(&handle);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test SHT31 heater enable functionality
 *
 * Verifies that internal heater can be enabled correctly
 * for condensation removal and sensor diagnostics.
 */
TEST_CASE("SHT31 heater enable succeeds", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    
    esp_err_t err = sht31_heater(&handle, true);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test SHT31 heater disable functionality
 *
 * Verifies that internal heater can be disabled correctly
 * to return to normal operation mode.
 */
TEST_CASE("SHT31 heater disable succeeds", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    
    esp_err_t err = sht31_heater(&handle, false);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test SHT31 heater control with uninitialized handle
 *
 * Verifies that heater control function properly validates initialization state
 * and rejects uninitialized handles with appropriate error.
 */
TEST_CASE("SHT31 heater fails with uninitialized handle", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = false;
    
    esp_err_t err = sht31_heater(&handle, true);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test SHT31 temperature and humidity reading with valid CRC
 *
 * Verifies that sensor data is read correctly, CRC validation passes,
 * and raw values are converted to proper temperature and humidity units.
 * Uses realistic test data with correct CRC values.
 */
TEST_CASE("SHT31 read temp hum succeeds with valid CRC", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock valid sensor data with correct CRCs */
    /* Temperature raw: 0x6666 (should give ~25°C) */
    /* Humidity raw: 0x8000 (should give ~50%RH) */
    fake_buf[0] = 0x66;  /* Temp MSB */
    fake_buf[1] = 0x66;  /* Temp LSB */
    fake_buf[2] = 0x92;  /* Temp CRC (calculated for 0x6666) */
    fake_buf[3] = 0x80;  /* Hum MSB */
    fake_buf[4] = 0x00;  /* Hum LSB */
    fake_buf[5] = 0xA2;  /* Hum CRC (calculated for 0x8000) */
    
    float temperature, humidity;
    esp_err_t err = sht31_read_temp_hum(&handle, &temperature, &humidity);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    /* Check temperature is approximately correct (25°C ±1°C) */
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 25.0f, temperature);
    
    /* Check humidity is approximately correct (50%RH ±5%) */
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 50.0f, humidity);
}

/**
 * @brief Test SHT31 reading with uninitialized handle
 *
 * Verifies that data reading function properly validates initialization state
 * and rejects uninitialized handles with appropriate error.
 */
TEST_CASE("SHT31 read temp hum fails with uninitialized handle", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = false;
    
    float temperature, humidity;
    esp_err_t err = sht31_read_temp_hum(&handle, &temperature, &humidity);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE, err);
}

/**
 * @brief Test SHT31 CRC validation and retry mechanism
 *
 * Verifies that invalid CRC data triggers retry mechanism and eventually
 * times out after maximum retry attempts, ensuring data integrity.
 */
TEST_CASE("SHT31 read temp hum retries on CRC failure", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock invalid CRC data (will cause retry and eventual timeout) */
    fake_buf[0] = 0x66;  /* Temp MSB */
    fake_buf[1] = 0x66;  /* Temp LSB */
    fake_buf[2] = 0x00;  /* Wrong Temp CRC */
    fake_buf[3] = 0x80;  /* Hum MSB */
    fake_buf[4] = 0x00;  /* Hum LSB */
    fake_buf[5] = 0x00;  /* Wrong Hum CRC */
    
    float temperature, humidity;
    esp_err_t err = sht31_read_temp_hum(&handle, &temperature, &humidity);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);  /* Should timeout after retries */
}

/**
 * @brief Test SHT31 I2C transmit error handling
 *
 * Verifies that I2C transmit errors during measurement command are
 * properly handled with retry mechanism and eventual timeout.
 */
TEST_CASE("SHT31 read temp hum handles I2C transmit error", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_ERR_TIMEOUT;  /* Simulate I2C error */
    stub_receive_ret = ESP_OK;
    
    float temperature, humidity;
    esp_err_t err = sht31_read_temp_hum(&handle, &temperature, &humidity);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);  /* Should timeout after retries */
}

/**
 * @brief Test SHT31 I2C receive error handling
 *
 * Verifies that I2C receive errors during data reading are
 * properly handled with retry mechanism and eventual timeout.
 */
TEST_CASE("SHT31 read temp hum handles I2C receive error", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_ERR_TIMEOUT;  /* Simulate I2C error */
    
    float temperature, humidity;
    esp_err_t err = sht31_read_temp_hum(&handle, &temperature, &humidity);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);  /* Should timeout after retries */
}

/**
 * @brief Test SHT31 humidity-only reading with null temperature pointer
 *
 * Verifies that sensor reading works correctly when only humidity is requested
 * and temperature pointer is null, demonstrating flexible API usage.
 */
TEST_CASE("SHT31 read temp hum works with null temperature pointer", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock valid sensor data with correct CRCs */
    fake_buf[0] = 0x66;  /* Temp MSB */
    fake_buf[1] = 0x66;  /* Temp LSB */
    fake_buf[2] = 0x92;  /* Temp CRC */
    fake_buf[3] = 0x80;  /* Hum MSB */
    fake_buf[4] = 0x00;  /* Hum LSB */
    fake_buf[5] = 0xA2;  /* Hum CRC */
    
    float humidity;
    esp_err_t err = sht31_read_temp_hum(&handle, NULL, &humidity);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 50.0f, humidity);
}

/**
 * @brief Test SHT31 temperature-only reading with null humidity pointer
 *
 * Verifies that sensor reading works correctly when only temperature is requested
 * and humidity pointer is null, demonstrating flexible API usage.
 */
TEST_CASE("SHT31 read temp hum works with null humidity pointer", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock valid sensor data with correct CRCs */
    fake_buf[0] = 0x66;  /* Temp MSB */
    fake_buf[1] = 0x66;  /* Temp LSB */
    fake_buf[2] = 0x92;  /* Temp CRC */
    fake_buf[3] = 0x80;  /* Hum MSB */
    fake_buf[4] = 0x00;  /* Hum LSB */
    fake_buf[5] = 0xA2;  /* Hum CRC */
    
    float temperature;
    esp_err_t err = sht31_read_temp_hum(&handle, &temperature, NULL);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 25.0f, temperature);
}

/**
 * @brief Test SHT31 dedicated temperature reading function
 *
 * Verifies that temperature-only convenience function works correctly
 * by internally calling the main reading function with NULL humidity pointer.
 */
TEST_CASE("SHT31 read temperature only succeeds", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock valid sensor data with correct CRCs */
    fake_buf[0] = 0x66;  /* Temp MSB */
    fake_buf[1] = 0x66;  /* Temp LSB */
    fake_buf[2] = 0x92;  /* Temp CRC */
    fake_buf[3] = 0x80;  /* Hum MSB */
    fake_buf[4] = 0x00;  /* Hum LSB */
    fake_buf[5] = 0xA2;  /* Hum CRC */
    
    float temperature;
    esp_err_t err = sht31_read_temperature(&handle, &temperature);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, 25.0f, temperature);
}

/**
 * @brief Test SHT31 dedicated humidity reading function
 *
 * Verifies that humidity-only convenience function works correctly
 * by internally calling the main reading function with NULL temperature pointer.
 */
TEST_CASE("SHT31 read humidity only succeeds", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Mock valid sensor data with correct CRCs */
    fake_buf[0] = 0x66;  /* Temp MSB */
    fake_buf[1] = 0x66;  /* Temp LSB */
    fake_buf[2] = 0x92;  /* Temp CRC */
    fake_buf[3] = 0x80;  /* Hum MSB */
    fake_buf[4] = 0x00;  /* Hum LSB */
    fake_buf[5] = 0xA2;  /* Hum CRC */
    
    float humidity;
    esp_err_t err = sht31_read_humidity(&handle, &humidity);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 50.0f, humidity);
}

/**
 * @brief Test SHT31 temperature conversion at minimum range
 *
 * Verifies temperature conversion accuracy at the low end of the measurement range
 * using minimum raw value (0x0000) which should convert to approximately -45°C.
 */
TEST_CASE("SHT31 temperature conversion edge cases", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Test minimum temperature raw value (0x0000 should give ~-45°C) */
    fake_buf[0] = 0x00;  /* Temp MSB */
    fake_buf[1] = 0x00;  /* Temp LSB */
    fake_buf[2] = 0xAC;  /* Temp CRC for 0x0000 */
    fake_buf[3] = 0x00;  /* Hum MSB */
    fake_buf[4] = 0x00;  /* Hum LSB */
    fake_buf[5] = 0xAC;  /* Hum CRC for 0x0000 */
    
    float temperature, humidity;
    esp_err_t err = sht31_read_temp_hum(&handle, &temperature, &humidity);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(1.0f, -45.0f, temperature);
}

/**
 * @brief Test SHT31 humidity conversion at maximum range
 *
 * Verifies humidity conversion accuracy at the high end of the measurement range
 * using maximum raw value (0xFFFF) which should convert to approximately 100%RH.
 */
TEST_CASE("SHT31 humidity conversion edge cases", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_OK;
    
    /* Test maximum humidity raw value (0xFFFF should give ~100%RH) */
    fake_buf[0] = 0x00;  /* Temp MSB */
    fake_buf[1] = 0x00;  /* Temp LSB */
    fake_buf[2] = 0xAC;  /* Temp CRC for 0x0000 */
    fake_buf[3] = 0xFF;  /* Hum MSB */
    fake_buf[4] = 0xFF;  /* Hum LSB */
    fake_buf[5] = 0xFF;  /* Hum CRC for 0xFFFF */
    
    float temperature, humidity;
    esp_err_t err = sht31_read_temp_hum(&handle, &temperature, &humidity);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(5.0f, 100.0f, humidity);
}

/**
 * @brief Test SHT31 I2C communication error handling in status reading
 *
 * Verifies that I2C communication errors during status register reading
 * are properly detected and appropriate error codes are returned.
 */
TEST_CASE("SHT31 handles I2C communication errors in status read", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_receive_ret = ESP_ERR_TIMEOUT;
    
    uint16_t status;
    esp_err_t err = sht31_read_status(&handle, &status);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @brief Test SHT31 I2C communication error handling in heater control
 *
 * Verifies that I2C communication errors during heater control commands
 * are properly detected and appropriate error codes are returned.
 */
TEST_CASE("SHT31 handles I2C communication errors in heater control", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_ERR_TIMEOUT;
    
    esp_err_t err = sht31_heater(&handle, true);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @brief Test SHT31 I2C communication error handling in reset
 *
 * Verifies that I2C communication errors during sensor reset command
 * are properly detected and appropriate error codes are returned.
 */
TEST_CASE("SHT31 handles I2C communication errors in reset", "[sht31]") {
    sht31_handle_t handle = { 0 };
    handle.initialised = true;
    handle.dev = (i2c_master_dev_handle_t)0x1234;
    
    stub_transmit_ret = ESP_ERR_TIMEOUT;
    
    esp_err_t err = sht31_reset(&handle);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @} (end of sht31_tests group)
 */