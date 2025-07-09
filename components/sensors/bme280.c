/**
 * @file bme280.c
 * @brief Implementation of BME280 sensor driver functions.
 */

#include "bme280.h"

static const char *TAG = "bme280";

/**
 * @brief Populate default configuration values.
 *
 * @param config Pointer to bme280_config_t to fill with defaults.
 */
void bme280_get_default_config(bme280_config_t *config)
{
    config->osrs_t       = BME280_OSRS_X1;      /* Default 1x temperature oversampling */
    config->osrs_p       = BME280_OSRS_X1;      /* Default 1x pressure oversampling */
    config->osrs_h       = BME280_OSRS_X1;      /* Default 1x humidity oversampling */
    config->filter       = BME280_FILTER_OFF;   /* No IIR filtering */
    config->standby_time = BME280_STANDBY_1000; /* 1000 ms between readings */
    config->mode         = BME280_MODE_SLEEP;   /* Sleep mode */
}

/**
 * @brief Write a single byte to a sensor register.
 */
static esp_err_t write_reg(bme280_handle_t *h, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };

    return i2c_master_transmit(h->dev, buf, sizeof(buf), portMAX_DELAY);
}

/**
 * @brief Read multiple bytes starting at a sensor register.
 */
static esp_err_t read_regs(bme280_handle_t *h, uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(h->dev, &reg, 1, dst, len, portMAX_DELAY);
}

/**
 * @brief Retrieve factory calibration data from the sensor.
 */
static esp_err_t read_calibration(bme280_handle_t *h)
{
    uint8_t c1[26];
    uint8_t c2[7];
    esp_err_t err;

    err = read_regs(h, BME280_REG_CALIB00, c1, sizeof(c1));
    if (err != ESP_OK) {
        return err;
    }

    err = read_regs(h, BME280_REG_CALIB26, c2, sizeof(c2));
    if (err != ESP_OK) {
        return err;
    }

    // Parse temperature calibration
    h->calib.dig_T1 = (uint16_t)(c1[1] << 8 | c1[0]);
    h->calib.dig_T2 = (int16_t) (c1[3] << 8 | c1[2]);
    h->calib.dig_T3 = (int16_t) (c1[5] << 8 | c1[4]);

    // Parse pressure calibration
    h->calib.dig_P1 = (uint16_t)(c1[7] << 8 | c1[6]);
    h->calib.dig_P2 = (int16_t) (c1[9] << 8 | c1[8]);
    h->calib.dig_P3 = (int16_t) (c1[11] << 8 | c1[10]);
    h->calib.dig_P4 = (int16_t) (c1[13] << 8 | c1[12]);
    h->calib.dig_P5 = (int16_t) (c1[15] << 8 | c1[14]);
    h->calib.dig_P6 = (int16_t) (c1[17] << 8 | c1[16]);
    h->calib.dig_P7 = (int16_t) (c1[19] << 8 | c1[18]);
    h->calib.dig_P8 = (int16_t) (c1[21] << 8 | c1[20]);
    h->calib.dig_P9 = (int16_t) (c1[23] << 8 | c1[22]);

    // Parse humidity calibration
    h->calib.dig_H1 = c1[25];
    h->calib.dig_H2 = (int16_t)(c2[1] << 8 | c2[0]);
    h->calib.dig_H3 = c2[2];
    h->calib.dig_H4 = (int16_t)((c2[3] << 4) | (c2[4] & 0x0F));
    h->calib.dig_H5 = (int16_t)((c2[5] << 4) | (c2[4] >> 4));
    h->calib.dig_H6 = (int8_t) c2[6];

    return ESP_OK;
}

/**
 * @brief Apply configuration registers (humidity, measurement, filter).
 */
static esp_err_t write_configuration(bme280_handle_t *h)
{
    uint8_t v;

    // Set humidity oversampling
    v = (uint8_t)h->config.osrs_h;
    ESP_RETURN_ON_ERROR(write_reg(h, BME280_REG_CTRL_HUM, v), TAG, "ctrl_hum");

    // Set temperature/pressure oversampling and mode
    v = (h->config.osrs_t << 5) |
        (h->config.osrs_p << 2) |
        (h->config.mode & 0x03);
    ESP_RETURN_ON_ERROR(write_reg(h, BME280_REG_CTRL_MEAS, v), TAG, "ctrl_meas");

    // Set standby time and filter
    v = (h->config.standby_time << 5) |
        (h->config.filter << 2);
    ESP_RETURN_ON_ERROR(write_reg(h, BME280_REG_CONFIG, v), TAG, "config");

    return ESP_OK;
}

/**
 * @brief Convert raw temperature to °C and update t_fine.
 */
static float compensate_temp(bme280_handle_t *h, int32_t adc_T)
{
    float var1 = (((float)adc_T / 16384.0f) - ((float)h->calib.dig_T1 / 1024.0f)) *
                  (float)h->calib.dig_T2;

    float var2 = (((float)adc_T / 131072.0f - (float)h->calib.dig_T1 / 8192.0f) *
                  ((float)adc_T / 131072.0f - (float)h->calib.dig_T1 / 8192.0f)) *
                  (float)h->calib.dig_T3;

    h->t_fine = (int32_t)(var1 + var2);

    return (var1 + var2) / 5120.0f;
}

/**
 * @brief Convert raw pressure to Pa.
 */
static float compensate_press(bme280_handle_t *h, int32_t adc_P)
{
    int64_t var1 = (int64_t)h->t_fine - 128000;

    int64_t var2 = var1 * var1 * (int64_t)h->calib.dig_P6;
    var2 += (var1 * (int64_t)h->calib.dig_P5) << 17;
    var2 += ((int64_t)h->calib.dig_P4) << 35;

    var1 = ((var1 * var1 * (int64_t)h->calib.dig_P3) >> 8) +
           ((var1 * (int64_t)h->calib.dig_P2) << 12);

    var1 = (((((int64_t)1 << 47) + var1) * (int64_t)h->calib.dig_P1) >> 33);
    if (var1 == 0) {
        return 0.0f; // avoid division by zero
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;

    var1 = ((int64_t)h->calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)h->calib.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)h->calib.dig_P7 << 4);

    return (float)p / 256.0f;
}

/**
 * @brief Convert raw humidity to %RH.
 */
static float compensate_hum(bme280_handle_t *h, int32_t adc_H)
{
    float var_H = (float)h->t_fine - 76800.0f;

    // Apply humidity compensation formula
    var_H = (adc_H - ((float)h->calib.dig_H4 * 64.0f +
             ((float)h->calib.dig_H5 / 16384.0f) * var_H)) *
             ((float)h->calib.dig_H2 / 65536.0f) *
             (1.0f + ((float)h->calib.dig_H6 / 67108864.0f) * var_H *
              (1.0f + ((float)h->calib.dig_H3 / 67108864.0f) * var_H));

    // Final adjustment and clamp
    var_H = var_H * (1.0f - ((float)h->calib.dig_H1 * var_H / 524288.0f));

    if (var_H > 100.0f) {
        var_H = 100.0f;
    } else if (var_H < 0.0f) {
        var_H = 0.0f;
    }

    return var_H;
}

esp_err_t bme280_init(bme280_handle_t *h, i2c_port_t port, uint8_t addr, const bme280_config_t *cfg)
{
    // Validate handle
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");

    // Get I2C bus handle
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(port, &h->bus));

    // Add device to bus
    i2c_device_config_t dc = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(h->bus, &dc, &h->dev));

    // Create mutex for thread-safe operations
    h->mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(h->mutex, ESP_ERR_NO_MEM, TAG, "mutex");

    // Copy or set default configuration
    if (cfg) {
        h->config = *cfg;
    } else {
        bme280_get_default_config(&h->config);
    }

    // Check chip ID
    uint8_t id;
    ESP_ERROR_CHECK(read_regs(h, BME280_REG_CHIP_ID, &id, 1));
    ESP_RETURN_ON_FALSE(id == BME280_CHIP_ID, ESP_ERR_NOT_FOUND, TAG, "bad ID");

    // Reset sensor and delay for startup
    ESP_ERROR_CHECK(write_reg(h, BME280_REG_RESET, BME280_RESET_VALUE));
    vTaskDelay(pdMS_TO_TICKS(10));

    // Read calibration data
    ESP_ERROR_CHECK(read_calibration(h));

    // Apply user configuration
    ESP_ERROR_CHECK(write_configuration(h));

    h->initialized = true;

    return ESP_OK;
}

esp_err_t bme280_deinit(bme280_handle_t *h)
{
    ESP_RETURN_ON_FALSE(h && h->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    ESP_ERROR_CHECK(i2c_master_bus_rm_device(h->dev));

    vSemaphoreDelete(h->mutex);
    h->initialized = false;

    return ESP_OK;
}

esp_err_t bme280_configure(bme280_handle_t *h, const bme280_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(h && cfg, ESP_ERR_INVALID_ARG, TAG, "bad args");

    h->config = *cfg;

    return write_configuration(h);
}

esp_err_t bme280_trigger_measurement(bme280_handle_t *h)
{
    ESP_RETURN_ON_FALSE(h && h->initialized, ESP_ERR_INVALID_STATE, TAG, "not initialized");

    return write_configuration(h);
}

esp_err_t bme280_is_meas_ready(bme280_handle_t *h, bool *ready)
{
    ESP_RETURN_ON_FALSE(h && ready, ESP_ERR_INVALID_ARG, TAG, "bad args");

    uint8_t status;
    ESP_ERROR_CHECK(read_regs(h, BME280_REG_STATUS, &status, 1));

    *ready = ((status & 0x08) == 0);

    return ESP_OK;
}

esp_err_t bme280_read_data(bme280_handle_t *h, bme280_data_t *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "null ptr");

    uint8_t buf[8];
    ESP_ERROR_CHECK(read_regs(h, BME280_REG_DATA, buf, sizeof(buf)));

    int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);

    int32_t adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);

    int32_t adc_H = (buf[6] << 8)  | buf[7];

    // Compensate readings
    out->temperature = compensate_temp(h, adc_T);

    out->pressure    = compensate_press(h, adc_P) / 100.0f;

    out->humidity    = compensate_hum(h, adc_H);

    return ESP_OK;
}

esp_err_t bme280_read_data_isr(bme280_handle_t *h, bme280_data_t *out)
{
    return bme280_read_data(h, out);
}

esp_err_t bme280_reset(bme280_handle_t *h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");

    return write_reg(h, BME280_REG_RESET, BME280_RESET_VALUE);
}

esp_err_t bme280_get_chip_id(bme280_handle_t *h, uint8_t *id)
{
    ESP_RETURN_ON_FALSE(h && id, ESP_ERR_INVALID_ARG, TAG, "null ptr");

    return read_regs(h, BME280_REG_CHIP_ID, id, 1);
}

float bme280_calculate_sea_level_pressure(float pressure, float altitude)
{
    // Calculate sea-level pressure from current pressure and altitude
    return pressure / powf(1.0f - (altitude / 44330.0f), 5.255f);
}
