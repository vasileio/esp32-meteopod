#include "bh1750.h"
#include "esp_log.h"
#include "esp_check.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "BH1750";

/**
 * @brief Initialize BH1750 sensor and attach it to I2C bus.
 *
 * @param h      Pointer to the BH1750 handle to initialize.
 * @param port   I2C port number.
 * @param addr   I2C address of the BH1750 sensor.
 * @return esp_err_t
 *         - ESP_OK on success
 *         - ESP_ERR_INVALID_ARG if input is invalid
 *         - Other error codes from I2C driver
 */
esp_err_t bh1750_init(bh1750_handle_t *h, i2c_port_t port, uint8_t addr) {
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");

    esp_err_t err = i2c_master_get_bus_handle(port, &h->bus);
    ESP_RETURN_ON_ERROR(err, TAG, "get bus");

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = addr,
        .scl_speed_hz = 100000,
    };
    err = i2c_master_bus_add_device(h->bus, &dev_cfg, &h->dev);
    ESP_RETURN_ON_ERROR(err, TAG, "add device");

    h->mtreg = BH1750_DEFAULT_MTREG;
    h->mode = UNCONFIGURED;
    h->initialised = true;

    return ESP_OK;
}

/**
 * @brief Send a single command byte to the BH1750.
 *
 * @param h      Pointer to the BH1750 handle.
 * @param cmd    Command byte to send.
 * @return esp_err_t
 */
static esp_err_t bh1750_send_command(bh1750_handle_t *h, uint8_t cmd) {
    return i2c_master_transmit(h->dev, &cmd, 1, -1);
}

/**
 * @brief Set the measurement mode for the BH1750 sensor.
 *
 * @param h      Pointer to the BH1750 handle.
 * @param mode   Measurement mode to set.
 * @return esp_err_t
 */
esp_err_t bh1750_set_mode(bh1750_handle_t *h, bh1750_mode_t mode) {
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not initialised");
    h->mode = mode;
    return bh1750_send_command(h, mode);
}

/**
 * @brief Reset the data register of the BH1750 sensor.
 *
 * This must be called after power on if POWER_ON command was sent first.
 *
 * @param h      Pointer to the BH1750 handle.
 * @return esp_err_t
 */
esp_err_t bh1750_reset(bh1750_handle_t *h) {
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not initialised");
    return bh1750_send_command(h, 0x07);  // BH1750_RESET
}

/**
 * @brief Set the measurement time register (MTreg) for sensitivity adjustment.
 *
 * @param h      Pointer to the BH1750 handle.
 * @param mtreg  MTreg value to set (range: 31 to 254).
 * @return esp_err_t
 */
esp_err_t bh1750_set_mtreg(bh1750_handle_t *h, uint8_t mtreg) {
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not initialised");
    ESP_RETURN_ON_FALSE(mtreg >= BH1750_MTREG_MIN && mtreg <= BH1750_MTREG_MAX,
                        ESP_ERR_INVALID_ARG, TAG, "mtreg out of range");

    uint8_t hi = 0x40 | (mtreg >> 5);
    uint8_t lo = 0x60 | (mtreg & 0x1F);

    esp_err_t err = bh1750_send_command(h, hi);
    if (err != ESP_OK) return err;
    err = bh1750_send_command(h, lo);
    if (err != ESP_OK) return err;

    h->mtreg = mtreg;
    return ESP_OK;
}

/**
 * @brief Read raw 16-bit light measurement from BH1750 sensor.
 *
 * @param h      Pointer to the BH1750 handle.
 * @param raw    Pointer to variable to hold raw value.
 * @return esp_err_t
 */
static esp_err_t bh1750_read_raw(bh1750_handle_t *h, uint16_t *raw) {
    uint8_t buf[2];
    esp_err_t err = i2c_master_receive(h->dev, buf, 2, -1);
    if (err != ESP_OK) return err;

    *raw = (buf[0] << 8) | buf[1];
    return ESP_OK;
}

/**
 * @brief Perform a light level measurement and convert it to lux.
 *
 * If in one-time mode, the measurement will be triggered before reading.
 *
 * @param h      Pointer to the BH1750 handle.
 * @param lux    Pointer to float to hold resulting lux value.
 * @return esp_err_t
 */
esp_err_t bh1750_read_light(bh1750_handle_t *h, float *lux) {
    ESP_RETURN_ON_FALSE(h && h->initialised && lux, ESP_ERR_INVALID_ARG, TAG, "invalid state");

    /* Trigger one-time mode if applicable */
    if (h->mode == ONE_TIME_HIGH_RES_MODE ||
        h->mode == ONE_TIME_HIGH_RES_MODE_2 ||
        h->mode == ONE_TIME_LOW_RES_MODE) {
        esp_err_t err = bh1750_send_command(h, h->mode);
        if (err != ESP_OK) return err;
    }

    /* Delay for conversion time based on mode and MTreg */
    uint32_t delay_ms = (h->mode == CONTINUOUS_LOW_RES_MODE || h->mode == ONE_TIME_LOW_RES_MODE) ? 16 : 120;
    delay_ms = delay_ms * h->mtreg / BH1750_DEFAULT_MTREG;
    vTaskDelay(pdMS_TO_TICKS(delay_ms + 10));  // Add margin

    uint16_t raw = 0;
    esp_err_t err = bh1750_read_raw(h, &raw);
    if (err != ESP_OK) return err;

    /* Convert raw value to lux */
    *lux = (float)raw / (BH1750_CONV_FACTOR * (BH1750_DEFAULT_MTREG / (float)h->mtreg));
    return ESP_OK;
}
