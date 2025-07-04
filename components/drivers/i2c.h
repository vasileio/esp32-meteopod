#pragma once
#include "driver/i2c_master.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"

#define I2C_PORT       I2C_NUM_0
#define I2C_SDA_PIN    GPIO_NUM_21
#define I2C_SCL_PIN    GPIO_NUM_22
#define I2C_GLITCH_CNT 7

#define I2C_SPEED_STANDARD_MODE    (100000U)   /* 100 kHz, up to 100 kbit/s */
#define I2C_SPEED_FAST_MODE        (400000U)   /* 400 kHz, up to 400 kbit/s */
#define I2C_SPEED_FAST_PLUS_MODE  (1000000U)   /*   1 MHz, up to 1 Mbit/s */
#define I2C_SPEED_HIGH_SPEED_MODE (3400000U)   /*   3.4 MHz, up to 3.4 Mbit/s */

esp_err_t i2c_init(i2c_master_bus_handle_t *out_bus_handle);
