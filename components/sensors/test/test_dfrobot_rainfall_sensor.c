/**
 * @file test_dfrobot_rainfall_sensor.c
 * @brief Unit tests for DFRobot rainfall sensor driver
 *
 * This file contains comprehensive unit tests for the DFRobot rainfall sensor
 * driver, covering initialization, sensor identification, data reading, and
 * configuration functions.
 */

#include "unity.h"
#include "dfrobot_rainfall_sensor.h"
#include "esp_err.h"
#include <string.h>

/**
 * @brief External stub variables from i2c_test_stubs.c
 */
extern esp_err_t   stub_transmit_ret;  /**< I2C transmit return value */
extern esp_err_t   stub_receive_ret;   /**< I2C receive return value */
extern uint8_t     fake_buf[];         /**< Fake I2C data buffer */

/**
 * @defgroup rainfall_tests DFRobot Rainfall Sensor Tests
 * @brief Test cases for DFRobot rainfall sensor driver
 * @{
 */

/**
 * @brief Test DFRobot rainfall sensor initialization with valid parameters
 *
 * Verifies that sensor initializes correctly with valid I2C bus handle,
 * address, and communication speed.
 */
TEST_CASE("DFRobot init succeeds with valid params", "[rain]") {
    DFRobot_rainfall_sensor_t sensor = { 0 };
    esp_err_t err = DFRobot_rainfall_sensor_init(
        &sensor,
        (i2c_master_bus_handle_t)NULL,
        DFROBOT_RAINFALL_SENSOR_I2C_ADDR_DEFAULT,
        100000U
    );
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test DFRobot sensor identification with correct PID/VID
 *
 * Verifies that sensor begin function correctly identifies the sensor
 * when the expected Product ID (0x100C0) and Vendor ID (0x3343) are present.
 */
TEST_CASE("DFRobot begin detects correct PID/VID", "[rain]") {
    DFRobot_rainfall_sensor_t sensor;
    /* PID=0x100C0, VID=0x3343 -> bytes: C0,00,43,73 */
    fake_buf[0] = 0xC0;
    fake_buf[1] = 0x00;
    fake_buf[2] = 0x43;
    fake_buf[3] = 0x73;

    esp_err_t err = DFRobot_rainfall_sensor_begin(&sensor);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test DFRobot sensor identification failure with wrong PID/VID
 *
 * Verifies that sensor begin function properly rejects sensors with
 * incorrect or missing Product ID and Vendor ID values.
 */
TEST_CASE("DFRobot begin fails on wrong PID/VID", "[rain]") {
    DFRobot_rainfall_sensor_t sensor;
    fake_buf[0] = 0x00;
    fake_buf[1] = 0x00;
    fake_buf[2] = 0x00;
    fake_buf[3] = 0x00;

    esp_err_t err = DFRobot_rainfall_sensor_begin(&sensor);
    TEST_ASSERT_EQUAL(ESP_ERR_NOT_FOUND, err);
}

/**
 * @brief Test DFRobot sensor version reading
 *
 * Verifies that sensor version register is read correctly and the
 * 16-bit version value is properly assembled from received bytes.
 */
TEST_CASE("DFRobot get_version returns correct value", "[rain]") {
    DFRobot_rainfall_sensor_t sensor;
    /* version = 0x1234 -> bytes: 34,12 */
    fake_buf[0] = 0x34;
    fake_buf[1] = 0x12;

    uint16_t version;
    esp_err_t err = DFRobot_rainfall_sensor_get_version(&sensor, &version);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_HEX16(0x1234, version);
}

/**
 * @brief Test DFRobot cumulative rainfall reading with unit conversion
 *
 * Verifies that cumulative rainfall raw count is correctly converted
 * to millimeters using the sensor's conversion formula.
 */
TEST_CASE("DFRobot get_cumulative returns correct mm", "[rain]") {
    DFRobot_rainfall_sensor_t sensor;
    /* raw = 10000 -> mm = 1.0 -> bytes: 0x10,0x27,0x00,0x00 */
    fake_buf[0] = 0x10;
    fake_buf[1] = 0x27;
    fake_buf[2] = 0x00;
    fake_buf[3] = 0x00;

    float mm;
    esp_err_t err = DFRobot_rainfall_sensor_get_cumulative(&sensor, &mm);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 1.0f, mm);
}

/**
 * @brief Test DFRobot time-based rainfall reading parameter validation
 *
 * Verifies that time-based rainfall function properly validates the
 * hour parameter and rejects values outside the valid range (1-24).
 */
TEST_CASE("DFRobot get_cumulative_hours rejects invalid args", "[rain]") {
    DFRobot_rainfall_sensor_t sensor;
    float mm;
    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        DFRobot_rainfall_sensor_get_cumulative_hours(&sensor, 0, &mm)
    );
    TEST_ASSERT_EQUAL(
        ESP_ERR_INVALID_ARG,
        DFRobot_rainfall_sensor_get_cumulative_hours(&sensor, 25, &mm)
    );
}

/**
 * @brief Test DFRobot time-based rainfall reading with valid parameters
 *
 * Verifies that rainfall data for a specific time period (5 hours)
 * is read correctly and converted to millimeters.
 */
TEST_CASE("DFRobot get_cumulative_hours returns correct mm", "[rain]") {
    DFRobot_rainfall_sensor_t sensor;
    /* stub transmit then raw = 20000 -> mm = 2.0 */
    stub_transmit_ret = ESP_OK;
    fake_buf[0] = 0x20;
    fake_buf[1] = 0x4E;
    fake_buf[2] = 0x00;
    fake_buf[3] = 0x00;

    float mm;
    esp_err_t err = DFRobot_rainfall_sensor_get_cumulative_hours(
        &sensor, 5, &mm
    );
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 2.0f, mm);
}

/**
 * @brief Test DFRobot raw count reading with multi-byte assembly
 *
 * Verifies that raw bucket tip count is correctly assembled from
 * little-endian byte order and returned as 32-bit value.
 */
TEST_CASE("DFRobot get_raw_count returns correct count", "[rain]") {
    DFRobot_rainfall_sensor_t sensor;
    /* count = 0x01020304 -> bytes: 04,03,02,01 */
    fake_buf[0] = 0x04;
    fake_buf[1] = 0x03;
    fake_buf[2] = 0x02;
    fake_buf[3] = 0x01;

    uint32_t cnt;
    esp_err_t err = DFRobot_rainfall_sensor_get_raw_count(&sensor, &cnt);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_UINT32(0x01020304, cnt);
}

/**
 * @brief Test DFRobot bucket volume configuration error propagation
 *
 * Verifies that I2C communication errors during bucket volume
 * configuration are properly propagated to the caller.
 */
TEST_CASE("DFRobot set_bucket_volume propagates error", "[rain]") {
    DFRobot_rainfall_sensor_t sensor;
    stub_transmit_ret = ESP_ERR_TIMEOUT;
    TEST_ASSERT_EQUAL(
        ESP_ERR_TIMEOUT,
        DFRobot_rainfall_sensor_set_bucket_volume(&sensor, 0.5f)
    );
}

/**
 * @brief Test DFRobot sensor uptime reading with unit conversion
 *
 * Verifies that sensor uptime in minutes is correctly converted
 * to hours and returned as floating-point value.
 */
TEST_CASE("DFRobot get_uptime returns correct hours", "[rain]") {
    DFRobot_rainfall_sensor_t sensor;
    /* minutes = 120 -> hours = 2.0 -> bytes: 120,0 */
    fake_buf[0] = 120;
    fake_buf[1] = 0;

    float hours;
    esp_err_t err = DFRobot_rainfall_sensor_get_uptime(&sensor, &hours);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(0.001, 2.0f, hours);
}

/**
 * @} (end of rainfall_tests group)
 */
