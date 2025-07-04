#pragma once
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_PIN    GPIO_NUM_21
#define I2C_SCL_PIN    GPIO_NUM_22
#define I2C_GLITCH_CNT 7

esp_err_t i2c_init(i2c_master_bus_handle_t *out_bus_handle);
