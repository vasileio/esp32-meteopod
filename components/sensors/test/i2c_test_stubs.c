// i2c_test_stubs.c

#include "unity.h"
#include <string.h>
#include "esp_err.h"
#include "driver/i2c_master.h"

/* Shared stub variables (visible to all test CUs) */
esp_err_t stub_transmit_ret;
esp_err_t stub_receive_ret;
uint8_t    fake_buf[8];  // big enough for all tests

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

void setUp(void) {
    stub_transmit_ret = ESP_OK;
    stub_receive_ret  = ESP_OK;
    memset(fake_buf, 0, sizeof(fake_buf));
}

void tearDown(void) {
    /* nothing */
}
