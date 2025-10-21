/**
 * @file test_wind_sensor.c
 * @brief Unit tests for wind direction and speed sensor driver
 *
 * This file contains comprehensive unit tests for the wind sensor driver,
 * covering initialization, ADC reading, voltage-to-direction conversion,
 * voltage-to-speed conversion, and error handling scenarios.
 */

#include "unity.h"
#include "wind_sensor.h"
#include "esp_err.h"
#include <string.h>
#include <math.h>

/**
 * @brief External stub variables from i2c_test_stubs.c
 */
extern esp_err_t stub_adc_init_ret;    /**< ADC init return value */
extern esp_err_t stub_adc_config_ret;  /**< ADC config return value */
extern esp_err_t stub_adc_read_ret;    /**< ADC read return value */
extern int stub_adc_raw_value;         /**< Mock ADC raw reading value */

/**
 * @defgroup wind_sensor_tests Wind Direction and Speed Sensor Tests
 * @brief Test cases for wind sensor driver
 * @{
 */

/**
 * @brief Test wind sensor initialization with valid parameters
 *
 * Verifies that sensor initializes correctly, configures ADC channels,
 * and sets up both direction and speed sensing.
 */
TEST_CASE("Wind sensor init succeeds", "[wind_sensor]") {
    stub_adc_init_ret = ESP_OK;
    stub_adc_config_ret = ESP_OK;
    
    esp_err_t err = wind_sensor_init();
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test wind sensor initialization fails with ADC init error
 *
 * Verifies that ADC initialization errors during sensor setup
 * are properly handled and appropriate error codes returned.
 */
TEST_CASE("Wind sensor init handles ADC init error", "[wind_sensor]") {
    stub_adc_init_ret = ESP_ERR_NO_MEM;  /* Simulate ADC init failure */
    stub_adc_config_ret = ESP_OK;
    
    esp_err_t err = wind_sensor_init();
    TEST_ASSERT_EQUAL(ESP_ERR_NO_MEM, err);
}

/**
 * @brief Test wind sensor initialization fails with ADC config error
 *
 * Verifies that ADC channel configuration errors during sensor setup
 * are properly handled and appropriate error codes returned.
 */
TEST_CASE("Wind sensor init handles ADC config error", "[wind_sensor]") {
    stub_adc_init_ret = ESP_OK;
    stub_adc_config_ret = ESP_ERR_INVALID_ARG;  /* Simulate ADC config failure */
    
    esp_err_t err = wind_sensor_init();
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test wind sensor reading with North direction
 *
 * Verifies that low voltage readings are correctly interpreted
 * as North wind direction.
 */
TEST_CASE("Wind sensor reads North direction", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    /* Set ADC value for North direction (~400mV) */
    stub_adc_raw_value = 500;  /* ~400mV when scaled */
    
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("N", readings.direction);
    TEST_ASSERT_TRUE(readings.speed >= 0.0f);
}

/**
 * @brief Test wind sensor reading with Northeast direction
 *
 * Verifies that medium-low voltage readings are correctly interpreted
 * as Northeast wind direction.
 */
TEST_CASE("Wind sensor reads Northeast direction", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    /* Set ADC value for Northeast direction (~700mV) */
    stub_adc_raw_value = 868;  /* ~700mV when scaled */
    
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("NE", readings.direction);
    TEST_ASSERT_TRUE(readings.speed >= 0.0f);
}

/**
 * @brief Test wind sensor reading with East direction
 *
 * Verifies that medium voltage readings are correctly interpreted
 * as East wind direction.
 */
TEST_CASE("Wind sensor reads East direction", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    /* Set ADC value for East direction (~1000mV) */
    stub_adc_raw_value = 1240;  /* ~1000mV when scaled */
    
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("E", readings.direction);
    TEST_ASSERT_TRUE(readings.speed >= 0.0f);
}

/**
 * @brief Test wind sensor reading with Southeast direction
 *
 * Verifies that medium-high voltage readings are correctly interpreted
 * as Southeast wind direction.
 */
TEST_CASE("Wind sensor reads Southeast direction", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    /* Set ADC value for Southeast direction (~1300mV) */
    stub_adc_raw_value = 1612;  /* ~1300mV when scaled */
    
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("SE", readings.direction);
    TEST_ASSERT_TRUE(readings.speed >= 0.0f);
}

/**
 * @brief Test wind sensor reading with South direction
 *
 * Verifies that high voltage readings are correctly interpreted
 * as South wind direction.
 */
TEST_CASE("Wind sensor reads South direction", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    /* Set ADC value for South direction (~1600mV) */
    stub_adc_raw_value = 1983;  /* ~1600mV when scaled */
    
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("S", readings.direction);
    TEST_ASSERT_TRUE(readings.speed >= 0.0f);
}

/**
 * @brief Test wind sensor reading with Southwest direction
 *
 * Verifies that high voltage readings are correctly interpreted
 * as Southwest wind direction.
 */
TEST_CASE("Wind sensor reads Southwest direction", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    /* Set ADC value for Southwest direction (~1900mV) */
    stub_adc_raw_value = 2355;  /* ~1900mV when scaled */
    
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("SW", readings.direction);
    TEST_ASSERT_TRUE(readings.speed >= 0.0f);
}

/**
 * @brief Test wind sensor reading with West direction
 *
 * Verifies that very high voltage readings are correctly interpreted
 * as West wind direction.
 */
TEST_CASE("Wind sensor reads West direction", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    /* Set ADC value for West direction (~2300mV) */
    stub_adc_raw_value = 2851;  /* ~2300mV when scaled */
    
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("W", readings.direction);
    TEST_ASSERT_TRUE(readings.speed >= 0.0f);
}

/**
 * @brief Test wind sensor reading with Northwest direction
 *
 * Verifies that maximum voltage readings are correctly interpreted
 * as Northwest wind direction.
 */
TEST_CASE("Wind sensor reads Northwest direction", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    /* Set ADC value for Northwest direction (~2700mV) */
    stub_adc_raw_value = 3347;  /* ~2700mV when scaled */
    
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("NW", readings.direction);
    TEST_ASSERT_TRUE(readings.speed >= 0.0f);
}

/**
 * @brief Test wind speed calculation from voltage
 *
 * Verifies that wind speed is calculated correctly from analog voltage
 * using the 4-20mA to 0-30m/s conversion formula.
 */
TEST_CASE("Wind sensor calculates speed correctly", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    
    /* Test zero wind speed (4mA = 0V) */
    stub_adc_raw_value = 0;  /* 0mV */
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(0.5f, 0.0f, readings.speed);
    
    /* Test medium wind speed (~1000mV should give ~6.6 m/s) */
    stub_adc_raw_value = 1240;  /* ~1000mV */
    err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(2.0f, 6.6f, readings.speed);
    
    /* Test high wind speed (~2000mV should give ~13.2 m/s) */
    stub_adc_raw_value = 2479;  /* ~2000mV */
    err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(3.0f, 13.2f, readings.speed);
}

/**
 * @brief Test wind sensor reading with ADC read error
 *
 * Verifies that ADC read errors during measurement are properly handled
 * and appropriate error codes are returned.
 */
TEST_CASE("Wind sensor handles ADC read error", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_ERR_TIMEOUT;  /* Simulate ADC read failure */
    
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @brief Test wind sensor reading with null data pointer
 *
 * Verifies that reading function properly validates data pointer
 * and returns appropriate error for null input.
 */
TEST_CASE("Wind sensor read fails with null data pointer", "[wind_sensor]") {
    esp_err_t err = wind_sensor_read(NULL);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test wind direction boundary conditions
 *
 * Verifies that direction mapping works correctly at voltage boundaries
 * between different wind directions.
 */
TEST_CASE("Wind sensor direction boundary conditions", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    
    /* Test boundary between North and Northeast (~560mV) */
    stub_adc_raw_value = 694;  /* ~559mV - should be N */
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("N", readings.direction);
    
    stub_adc_raw_value = 695;  /* ~561mV - should be NE */
    err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("NE", readings.direction);
    
    /* Test boundary between Northeast and East (~830mV) */
    stub_adc_raw_value = 1029;  /* ~829mV - should be NE */
    err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("NE", readings.direction);
    
    stub_adc_raw_value = 1030;  /* ~831mV - should be E */
    err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("E", readings.direction);
}

/**
 * @brief Test wind speed edge cases
 *
 * Verifies that wind speed calculation handles edge cases correctly,
 * including negative voltages and maximum range values.
 */
TEST_CASE("Wind sensor speed edge cases", "[wind_sensor]") {
    wind_data_t readings;
    
    stub_adc_read_ret = ESP_OK;
    
    /* Test maximum ADC value (~3.3V) */
    stub_adc_raw_value = 4095;  /* Maximum 12-bit ADC value */
    esp_err_t err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_TRUE(readings.speed >= 0.0f);  /* Should never be negative */
    TEST_ASSERT_TRUE(readings.speed < 50.0f);  /* Should be reasonable */
    
    /* Test minimum ADC value (should clamp to 0 speed) */
    stub_adc_raw_value = 0;
    err = wind_sensor_read(&readings);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_FLOAT_WITHIN(0.1f, 0.0f, readings.speed);
}

/**
 * @brief Test wind sensor averaging behavior
 *
 * Verifies that ADC averaging produces consistent results
 * and improves measurement stability.
 */
TEST_CASE("Wind sensor averaging stability", "[wind_sensor]") {
    wind_data_t readings1, readings2;
    
    stub_adc_read_ret = ESP_OK;
    stub_adc_raw_value = 1240;  /* Fixed value for consistency test */
    
    /* Take two consecutive readings */
    esp_err_t err1 = wind_sensor_read(&readings1);
    esp_err_t err2 = wind_sensor_read(&readings2);
    
    TEST_ASSERT_EQUAL(ESP_OK, err1);
    TEST_ASSERT_EQUAL(ESP_OK, err2);
    
    /* Results should be identical with fixed ADC value */
    TEST_ASSERT_EQUAL_STRING(readings1.direction, readings2.direction);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, readings1.speed, readings2.speed);
}

/**
 * @brief Test comprehensive wind sensor functionality
 *
 * Verifies that all 8 wind directions can be correctly identified
 * and that speed calculation works across the full range.
 */
TEST_CASE("Wind sensor comprehensive direction test", "[wind_sensor]") {
    wind_data_t readings;
    stub_adc_read_ret = ESP_OK;
    
    /* Test all 8 cardinal and intercardinal directions */
    const struct {
        int adc_value;
        const char* expected_direction;
    } direction_tests[] = {
        {400, "N"},    /* ~320mV */
        {750, "NE"},   /* ~600mV */ 
        {1100, "E"},   /* ~880mV */
        {1500, "SE"},  /* ~1200mV */
        {2000, "S"},   /* ~1600mV */
        {2400, "SW"},  /* ~1920mV */
        {2800, "W"},   /* ~2240mV */
        {3400, "NW"}   /* ~2720mV */
    };
    
    for (size_t i = 0; i < sizeof(direction_tests) / sizeof(direction_tests[0]); i++) {
        stub_adc_raw_value = direction_tests[i].adc_value;
        esp_err_t err = wind_sensor_read(&readings);
        TEST_ASSERT_EQUAL(ESP_OK, err);
        TEST_ASSERT_EQUAL_STRING(direction_tests[i].expected_direction, readings.direction);
        TEST_ASSERT_TRUE(readings.speed >= 0.0f);
    }
}

/**
 * @} (end of wind_sensor_tests group)
 */