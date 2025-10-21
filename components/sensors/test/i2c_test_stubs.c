/**
 * @file i2c_test_stubs.c
 * @brief I2C and system function stubs for sensor unit tests
 *
 * This file provides stub implementations of I2C master functions, GPIO functions,
 * and FreeRTOS functions to enable unit testing of sensor drivers without requiring
 * actual hardware or full ESP-IDF environment.
 */

#include "unity.h"
#include <string.h>
#include "esp_err.h"
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/task.h"
#include "esp_adc/adc_oneshot.h"

/**
 * @brief Shared stub variables (visible to all test CUs)
 */
esp_err_t stub_transmit_ret;  /**< Return value for I2C transmit operations */
esp_err_t stub_receive_ret;   /**< Return value for I2C receive operations */
uint8_t    fake_buf[8];       /**< Buffer for simulated I2C data (big enough for all tests) */

/* ADC stub variables */
esp_err_t stub_adc_init_ret = ESP_OK;  /**< Return value for ADC init operations */
esp_err_t stub_adc_config_ret = ESP_OK; /**< Return value for ADC config operations */
esp_err_t stub_adc_read_ret = ESP_OK;   /**< Return value for ADC read operations */
int stub_adc_raw_value = 2048;          /**< Mock ADC raw reading value (0-4095) */

/**
 * @brief Stub implementation of i2c_master_bus_add_device
 *
 * @param bus I2C bus handle (ignored)
 * @param cfg Device configuration (ignored)
 * @param out_handle Pointer to store device handle
 * @return ESP_OK always
 */
esp_err_t i2c_master_bus_add_device(
    i2c_master_bus_handle_t bus,
    const i2c_device_config_t *cfg,
    i2c_master_dev_handle_t *out_handle
) {
    (void)bus;
    (void)cfg;
    *out_handle = (i2c_master_dev_handle_t)0x1234;
    return ESP_OK;
}

/**
 * @brief Stub implementation of i2c_master_transmit
 *
 * @param dev I2C device handle (ignored)
 * @param data Data to transmit (ignored)
 * @param len Length of data (ignored)
 * @param xfer_timeout_ms Transfer timeout in milliseconds (ignored)
 * @return Value of stub_transmit_ret
 */
esp_err_t i2c_master_transmit(
    i2c_master_dev_handle_t dev,
    const uint8_t *data,
    size_t len,
    int xfer_timeout_ms
) {
    (void)dev;
    (void)data;
    (void)len;
    (void)xfer_timeout_ms;
    return stub_transmit_ret;
}

/**
 * @brief Stub implementation of i2c_master_receive
 *
 * @param dev I2C device handle (ignored)
 * @param data Buffer to store received data
 * @param len Length of data to receive
 * @param xfer_timeout_ms Transfer timeout in milliseconds (ignored)
 * @return Value of stub_receive_ret
 */
esp_err_t i2c_master_receive(
    i2c_master_dev_handle_t dev,
    uint8_t *data,
    size_t len,
    int xfer_timeout_ms
) {
    (void)dev;
    (void)xfer_timeout_ms;
    memcpy(data, fake_buf, len);
    return stub_receive_ret;
}

/**
 * @brief Stub implementation of i2c_master_transmit_receive
 *
 * @param dev I2C device handle (ignored)
 * @param write_data Data to transmit (ignored)
 * @param write_size Size of write data (ignored)
 * @param read_data Buffer to store received data
 * @param read_size Size of read data
 * @param xfer_timeout_ms Transfer timeout in milliseconds (ignored)
 * @return Value of stub_receive_ret
 */
esp_err_t i2c_master_transmit_receive(
    i2c_master_dev_handle_t dev,
    const uint8_t *write_data,
    size_t write_size,
    uint8_t *read_data,
    size_t read_size,
    int xfer_timeout_ms
) {
    (void)dev;
    (void)write_data;
    (void)write_size;
    (void)xfer_timeout_ms;
    memcpy(read_data, fake_buf, read_size);
    return stub_receive_ret;
}

/**
 * @brief Stub implementation of i2c_master_get_bus_handle
 *
 * @param port_num I2C port number (ignored)
 * @param ret_handle Pointer to store bus handle
 * @return ESP_OK always
 */
esp_err_t i2c_master_get_bus_handle(i2c_port_num_t port_num, i2c_master_bus_handle_t *ret_handle) {
    (void)port_num;
    *ret_handle = (i2c_master_bus_handle_t)0x5678;
    return ESP_OK;
}

/**
 * @brief Stub implementation of i2c_master_bus_rm_device
 *
 * @param handle I2C device handle (ignored)
 * @return ESP_OK always
 */
esp_err_t i2c_master_bus_rm_device(i2c_master_dev_handle_t handle) {
    (void)handle;
    return ESP_OK;
}

/**
 * @brief Mock data for queue simulation (used by AS3935 tests)
 */
uint8_t mock_queue_data[32];        /**< Mock queue data buffer */
bool mock_queue_has_data = false;   /**< Flag indicating if mock queue has data */

/**
 * @brief Unity test setup function
 *
 * Called before each test case to initialize stub variables
 * and reset mock data to known state.
 */
void setUp(void) {
    stub_transmit_ret = ESP_OK;
    stub_receive_ret  = ESP_OK;
    memset(fake_buf, 0, sizeof(fake_buf));
    mock_queue_has_data = false;
    memset(mock_queue_data, 0, sizeof(mock_queue_data));
    
    /* Reset ADC stub variables */
    stub_adc_init_ret = ESP_OK;
    stub_adc_config_ret = ESP_OK;
    stub_adc_read_ret = ESP_OK;
    stub_adc_raw_value = 2048;  /* Mid-range value */
}

/**
 * @brief Unity test teardown function
 *
 * Called after each test case. Currently no cleanup required.
 */
void tearDown(void) {
    /* nothing */
}

/**
 * @brief Stub implementation of adc_oneshot_new_unit
 *
 * @param init_config ADC unit initialization config (ignored)
 * @param ret_unit Pointer to store ADC unit handle
 * @return Value of stub_adc_init_ret
 */
esp_err_t adc_oneshot_new_unit(const adc_oneshot_unit_init_config_t *init_config, adc_oneshot_unit_handle_t *ret_unit) {
    (void)init_config;
    *ret_unit = (adc_oneshot_unit_handle_t)0xADC1;
    return stub_adc_init_ret;
}

/**
 * @brief Stub implementation of adc_oneshot_config_channel
 *
 * @param handle ADC unit handle (ignored)
 * @param channel ADC channel (ignored)  
 * @param config Channel configuration (ignored)
 * @return Value of stub_adc_config_ret
 */
esp_err_t adc_oneshot_config_channel(adc_oneshot_unit_handle_t handle, adc_channel_t channel, const adc_oneshot_chan_cfg_t *config) {
    (void)handle;
    (void)channel;
    (void)config;
    return stub_adc_config_ret;
}

/**
 * @brief Stub implementation of adc_oneshot_read
 *
 * @param handle ADC unit handle (ignored)
 * @param channel ADC channel (ignored)
 * @param out_raw Pointer to store raw ADC value
 * @return Value of stub_adc_read_ret
 */
esp_err_t adc_oneshot_read(adc_oneshot_unit_handle_t handle, adc_channel_t channel, int *out_raw) {
    (void)handle;
    (void)channel;
    *out_raw = stub_adc_raw_value;
    return stub_adc_read_ret;
}

/**
 * @brief Stub implementation of adc_oneshot_del_unit
 *
 * @param handle ADC unit handle (ignored)
 * @return ESP_OK always
 */
esp_err_t adc_oneshot_del_unit(adc_oneshot_unit_handle_t handle) {
    (void)handle;
    return ESP_OK;
}
