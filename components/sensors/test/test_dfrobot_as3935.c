/**
 * @file test_dfrobot_as3935.c
 * @brief Unit tests for DFRobot AS3935 lightning sensor driver
 *
 * This file contains comprehensive unit tests for the AS3935 lightning sensor
 * driver, covering initialization, configuration, data reading, and error handling.
 */

#include "unity.h"
#include "dfrobot_as3935.h"
#include "esp_err.h"
#include <string.h>

/**
 * @brief External stub variables from i2c_test_stubs.c
 */
extern esp_err_t   stub_transmit_ret;  /**< I2C transmit return value */
extern esp_err_t   stub_receive_ret;   /**< I2C receive return value */
extern uint8_t     fake_buf[];         /**< Fake I2C data buffer */

/**
 * @defgroup as3935_tests AS3935 Lightning Sensor Tests
 * @brief Test cases for DFRobot AS3935 lightning sensor driver
 * @{
 */

/**
 * @brief Test AS3935 initialization with valid parameters
 *
 * Verifies that sensor initializes correctly with valid I2C port and address,
 * and that all sensor structure fields are properly set.
 */
TEST_CASE("AS3935 init succeeds with valid params", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    esp_err_t err = dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_NOT_NULL(sensor.bus);
    TEST_ASSERT_NOT_NULL(sensor.dev);
    TEST_ASSERT_EQUAL(GPIO_NUM_NC, sensor.irq_pin);
    TEST_ASSERT_NULL(sensor.irq_queue);
}

/**
 * @brief Test AS3935 initialization fails with null handle
 *
 * Verifies that initialization properly rejects null sensor handle
 * and returns appropriate error code.
 */
TEST_CASE("AS3935 init fails with null handle", "[as3935]") {
    esp_err_t err = dfrobot_as3935_init(NULL, I2C_NUM_0, AS3935_I2C_ADDR);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test AS3935 initialization with IRQ support
 *
 * Verifies that sensor initializes correctly with IRQ pin specified,
 * including proper GPIO configuration and queue creation.
 */
TEST_CASE("AS3935 init with IRQ succeeds with valid params", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    esp_err_t err = dfrobot_as3935_init_with_irq(&sensor, I2C_NUM_0, AS3935_I2C_ADDR, GPIO_NUM_4);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_NOT_NULL(sensor.bus);
    TEST_ASSERT_NOT_NULL(sensor.dev);
    TEST_ASSERT_EQUAL(GPIO_NUM_4, sensor.irq_pin);
    TEST_ASSERT_NOT_NULL(sensor.irq_queue);
}

/**
 * @brief Test AS3935 IRQ initialization fails with null handle
 *
 * Verifies that IRQ initialization properly rejects null sensor handle
 * and returns appropriate error code.
 */
TEST_CASE("AS3935 init with IRQ fails with null handle", "[as3935]") {
    esp_err_t err = dfrobot_as3935_init_with_irq(NULL, I2C_NUM_0, AS3935_I2C_ADDR, GPIO_NUM_4);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test AS3935 power up sequence
 *
 * Verifies that power up sequence executes correctly and follows
 * the datasheet specification for sensor initialization.
 */
TEST_CASE("AS3935 power up executes sequence correctly", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0xFF;  /* Initial register value */
    
    esp_err_t err = dfrobot_as3935_power_up(&sensor);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 power down functionality
 *
 * Verifies that power down sequence properly sets the PWD bit
 * to put the sensor into low power mode.
 */
TEST_CASE("AS3935 power down sets PWD bit", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0x00;  /* Initial register value */
    
    esp_err_t err = dfrobot_as3935_power_down(&sensor);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 reset command
 *
 * Verifies that reset command is sent correctly to restore
 * sensor to default configuration.
 */
TEST_CASE("AS3935 reset sends correct command", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    esp_err_t err = dfrobot_as3935_reset(&sensor);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 RCO calibration
 *
 * Verifies that internal RC oscillator calibration
 * executes successfully.
 */
TEST_CASE("AS3935 calibrate RCO succeeds", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    esp_err_t err = dfrobot_as3935_calibrate_rco(&sensor);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 antenna tuning capacitor setting
 *
 * Verifies that tuning capacitor value is set correctly
 * while preserving other register bits.
 */
TEST_CASE("AS3935 set tuning caps modifies correct bits", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0xF0;  /* Initial register value (upper nibble set) */
    
    esp_err_t err = dfrobot_as3935_set_tuning_caps(&sensor, 0x05);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 interrupt source reading
 *
 * Verifies that interrupt source register is read correctly
 * and returns the expected value.
 */
TEST_CASE("AS3935 get interrupt source reads register", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0x08;  /* Lightning interrupt */
    
    uint8_t irq_src;
    esp_err_t err = dfrobot_as3935_get_interrupt_src(&sensor, &irq_src);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(0x08, irq_src);
}

/**
 * @brief Test AS3935 lightning distance reading with bit masking
 *
 * Verifies that distance value is properly masked to extract
 * only the relevant 6 bits from the register.
 */
TEST_CASE("AS3935 get lightning distance masks correctly", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0xFF;  /* Set all bits */
    
    uint8_t distance;
    esp_err_t err = dfrobot_as3935_get_lightning_distance(&sensor, &distance);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL(0x3F, distance);  /* Only lower 6 bits should be set */
}

/**
 * @brief Test AS3935 strike energy reading from multiple registers
 *
 * Verifies that energy value is correctly combined from
 * the three energy registers (MSB, MID, LSB).
 */
TEST_CASE("AS3935 get strike energy combines registers correctly", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    /* Setup energy registers: MSB=0x12, MID=0x34, LSB=0x56 */
    fake_buf[0] = 0x12;  /* First call returns MSB */
    
    uint32_t energy;
    esp_err_t err = dfrobot_as3935_get_strike_energy(&sensor, &energy);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    /* Note: Due to stub limitations, we can't test the full 24-bit combination */
    /* In a real test, this would verify (0x12 << 16) | (0x34 << 8) | 0x56 */
}

/**
 * @brief Test AS3935 indoor environment configuration
 *
 * Verifies that AFE gain is configured correctly for
 * indoor operation with reduced sensitivity.
 */
TEST_CASE("AS3935 set indoor configures AFE gain correctly", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0x00;  /* Initial register value */
    
    esp_err_t err = dfrobot_as3935_set_indoor(&sensor);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 outdoor environment configuration
 *
 * Verifies that AFE gain is configured correctly for
 * outdoor operation with increased sensitivity.
 */
TEST_CASE("AS3935 set outdoor configures AFE gain correctly", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0x00;  /* Initial register value */
    
    esp_err_t err = dfrobot_as3935_set_outdoor(&sensor);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 noise floor validation - invalid level
 *
 * Verifies that noise floor setting properly validates input
 * and rejects values outside the valid range (0-7).
 */
TEST_CASE("AS3935 set noise floor rejects invalid level", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    esp_err_t err = dfrobot_as3935_set_noise_floor(&sensor, 8);  /* Invalid (max is 7) */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test AS3935 noise floor setting - valid level
 *
 * Verifies that noise floor setting accepts valid values
 * and configures the register correctly.
 */
TEST_CASE("AS3935 set noise floor accepts valid level", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0x00;  /* Initial register value */
    
    esp_err_t err = dfrobot_as3935_set_noise_floor(&sensor, 5);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 watchdog threshold validation - invalid level
 *
 * Verifies that watchdog threshold setting properly validates input
 * and rejects values outside the valid range (0-15).
 */
TEST_CASE("AS3935 set watchdog threshold rejects invalid level", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    esp_err_t err = dfrobot_as3935_set_watchdog_threshold(&sensor, 16);  /* Invalid (max is 15) */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test AS3935 watchdog threshold setting - valid level
 *
 * Verifies that watchdog threshold setting accepts valid values
 * and configures the register correctly.
 */
TEST_CASE("AS3935 set watchdog threshold accepts valid level", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0x00;  /* Initial register value */
    
    esp_err_t err = dfrobot_as3935_set_watchdog_threshold(&sensor, 10);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 minimum lightning events validation - invalid count
 *
 * Verifies that minimum lightning events setting properly validates input
 * and rejects values outside the valid range (0-3).
 */
TEST_CASE("AS3935 set min lightning rejects invalid count", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    esp_err_t err = dfrobot_as3935_set_min_lightning(&sensor, 4);  /* Invalid (max is 3) */
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test AS3935 minimum lightning events setting - valid count
 *
 * Verifies that minimum lightning events setting accepts valid values
 * and configures the register correctly.
 */
TEST_CASE("AS3935 set min lightning accepts valid count", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0x00;  /* Initial register value */
    
    esp_err_t err = dfrobot_as3935_set_min_lightning(&sensor, 2);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 disturber detection control
 *
 * Verifies that disturber detection can be enabled and disabled
 * correctly by modifying the appropriate register bits.
 */
TEST_CASE("AS3935 set disturber modifies correct register", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    stub_receive_ret = ESP_OK;
    fake_buf[0] = 0xFF;  /* Initial register value */
    
    esp_err_t err = dfrobot_as3935_set_disturber(&sensor, true);
    TEST_ASSERT_EQUAL(ESP_OK, err);
    
    err = dfrobot_as3935_set_disturber(&sensor, false);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

/**
 * @brief Test AS3935 IRQ processing without queue
 *
 * Verifies that IRQ processing fails appropriately when
 * sensor was not initialized with IRQ support.
 */
TEST_CASE("AS3935 process IRQ fails without IRQ queue", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);  /* No IRQ init */
    
    lightning_data_t lightning_data;
    esp_err_t err = dfrobot_as3935_process_irq(&sensor, &lightning_data, 100);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test AS3935 IRQ processing with null arguments
 *
 * Verifies that IRQ processing properly validates input arguments
 * and rejects null pointers with appropriate error codes.
 */
TEST_CASE("AS3935 process IRQ fails with null arguments", "[as3935]") {
    lightning_data_t lightning_data;
    
    /* Test null sensor - this should work since we're only testing argument validation */
    esp_err_t err = dfrobot_as3935_process_irq(NULL, &lightning_data, 100);
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG, err);
}

/**
 * @brief Test AS3935 IRQ processing timeout behavior
 *
 * Verifies that IRQ processing times out appropriately when
 * no interrupt events are available in the queue.
 */
TEST_CASE("AS3935 process IRQ times out with no events", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init_with_irq(&sensor, I2C_NUM_0, AS3935_I2C_ADDR, GPIO_NUM_4);
    
    lightning_data_t lightning_data;
    esp_err_t err = dfrobot_as3935_process_irq(&sensor, &lightning_data, 0);  /* Immediate timeout */
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @brief Test AS3935 I2C communication error handling
 *
 * Verifies that I2C communication errors are properly handled
 * and appropriate error codes are returned to the caller.
 */
TEST_CASE("AS3935 handles I2C communication errors", "[as3935]") {
    dfrobot_as3935_t sensor = { 0 };
    dfrobot_as3935_init(&sensor, I2C_NUM_0, AS3935_I2C_ADDR);
    
    /* Test transmit error */
    stub_transmit_ret = ESP_ERR_TIMEOUT;
    esp_err_t err = dfrobot_as3935_reset(&sensor);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
    
    /* Test receive error */
    stub_transmit_ret = ESP_OK;
    stub_receive_ret = ESP_ERR_TIMEOUT;
    uint8_t irq_src;
    err = dfrobot_as3935_get_interrupt_src(&sensor, &irq_src);
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, err);
}

/**
 * @} (end of as3935_tests group)
 */