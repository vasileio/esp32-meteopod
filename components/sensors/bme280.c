/*
 * @file    bme280.c
 * @brief   BME280 sensor driver implementation.
 *
 * Provides initialization, configuration, measurement triggering,
 * data reading, and compensation routines for Bosch BME280 pressure,
 * temperature, and humidity sensor. All comments are in C-style block format.
 */

#include "bme280.h"

static const char *TAG = "bme280";

/**
 * @brief  Load default configuration values into provided struct.
 * @param  config  Pointer to bme280_config_t to populate.
 */
void bme280_get_default_config(bme280_config_t *config)
{
    /* Temperature oversampling: 1x */
    config->osrs_t       = BME280_OSRS_X1;
    /* Pressure oversampling: 1x */
    config->osrs_p       = BME280_OSRS_X1;
    /* Humidity oversampling: 1x */
    config->osrs_h       = BME280_OSRS_X1;
    /* IIR filter disabled */
    config->filter       = BME280_FILTER_OFF;
    /* Standby time between measurements: 1000 ms */
    config->standby_time = BME280_STANDBY_1000;
    /* Sensor mode: sleep (no continuous measurement) */
    config->mode         = BME280_MODE_SLEEP;
}

/**
 * @brief  Write a single register over I2C.
 * @param  h    Device handle containing I2C context.
 * @param  reg  Register address to write.
 * @param  val  Byte value to write into register.
 * @return ESP_OK on success or appropriate error code.
 */
static esp_err_t write_reg(bme280_handle_t *h, uint8_t reg, uint8_t val)
{
    uint8_t buf[2] = { reg, val };
    return i2c_master_transmit(h->dev, buf, sizeof(buf), portMAX_DELAY);
}

/**
 * @brief  Read multiple bytes starting at a register over I2C.
 * @param  h     Device handle containing I2C context.
 * @param  reg   Starting register address.
 * @param  dst   Destination buffer for incoming data.
 * @param  len   Number of bytes to read.
 * @return ESP_OK on success or appropriate error code.
 */
static esp_err_t read_regs(bme280_handle_t *h, uint8_t reg, uint8_t *dst, size_t len)
{
    return i2c_master_transmit_receive(h->dev, &reg, 1, dst, len, portMAX_DELAY);
}

/**
 * @brief  Retrieve factory calibration data from sensor.
 *
 * The BME280 stores calibration coefficients in non-volatile registers.
 * These are used in compensation formulas for temperature,
 * pressure, and humidity.
 *
 * @param  h  Device handle to populate calibration fields.
 * @return ESP_OK on success or appropriate error code.
 */
static esp_err_t read_calibration(bme280_handle_t *h)
{
    uint8_t calib_block1[26];
    uint8_t calib_block2[7];
    esp_err_t err;

    /* Read first block of calibration registers (0x88..0xA1) */
    err = read_regs(h, BME280_REG_CALIB00, calib_block1, sizeof(calib_block1));
    if (err != ESP_OK) {
        return err;
    }

    /* Read second block of calibration registers (0xE1..0xE7) */
    err = read_regs(h, BME280_REG_CALIB26, calib_block2, sizeof(calib_block2));
    if (err != ESP_OK) {
        return err;
    }

    /* Parse temperature calibration parameters */
    h->calib.dig_T1 = (uint16_t)(calib_block1[1] << 8 | calib_block1[0]);
    h->calib.dig_T2 = (int16_t) (calib_block1[3] << 8 | calib_block1[2]);
    h->calib.dig_T3 = (int16_t) (calib_block1[5] << 8 | calib_block1[4]);

    /* Parse pressure calibration parameters */
    h->calib.dig_P1 = (uint16_t)(calib_block1[7] << 8 | calib_block1[6]);
    h->calib.dig_P2 = (int16_t) (calib_block1[9] << 8 | calib_block1[8]);
    h->calib.dig_P3 = (int16_t) (calib_block1[11] << 8 | calib_block1[10]);
    h->calib.dig_P4 = (int16_t) (calib_block1[13] << 8 | calib_block1[12]);
    h->calib.dig_P5 = (int16_t) (calib_block1[15] << 8 | calib_block1[14]);
    h->calib.dig_P6 = (int16_t) (calib_block1[17] << 8 | calib_block1[16]);
    h->calib.dig_P7 = (int16_t) (calib_block1[19] << 8 | calib_block1[18]);
    h->calib.dig_P8 = (int16_t) (calib_block1[21] << 8 | calib_block1[20]);
    h->calib.dig_P9 = (int16_t) (calib_block1[23] << 8 | calib_block1[22]);

    /* Parse humidity calibration parameters */
    h->calib.dig_H1 = calib_block1[25];
    h->calib.dig_H2 = (int16_t)(calib_block2[1] << 8 | calib_block2[0]);
    h->calib.dig_H3 = calib_block2[2];
    h->calib.dig_H4 = (int16_t)((calib_block2[3] << 4) | (calib_block2[4] & 0x0F));
    h->calib.dig_H5 = (int16_t)((calib_block2[5] << 4) | (calib_block2[4] >> 4));
    h->calib.dig_H6 = (int8_t) calib_block2[6];

    return ESP_OK;
}

/**
 * @brief  Write sensor configuration to control registers.
 *
 * This sets humidity oversampling, temperature/pressure oversampling,
 * sensor mode, IIR filter, and standby duration.
 *
 * @param  h  Device handle with desired config values.
 * @return ESP_OK on success or appropriate error code.
 */
static esp_err_t write_configuration(bme280_handle_t *h)
{
    uint8_t regval;

    /* Humidity oversampling */
    regval = (uint8_t)h->config.osrs_h;
    ESP_RETURN_ON_ERROR(write_reg(h, BME280_REG_CTRL_HUM, regval), TAG, "ctrl_hum");

    /* Temperature & pressure oversampling + sensor mode */
    regval = (h->config.osrs_t << 5) |
             (h->config.osrs_p << 2) |
             (h->config.mode & 0x03);
    ESP_RETURN_ON_ERROR(write_reg(h, BME280_REG_CTRL_MEAS, regval), TAG, "ctrl_meas");

    /* Standby time & IIR filter setting */
    regval = (h->config.standby_time << 5) |
             (h->config.filter << 2);
    ESP_RETURN_ON_ERROR(write_reg(h, BME280_REG_CONFIG, regval), TAG, "config");

    return ESP_OK;
}

/**
 * @brief  Compensate raw temperature reading to degrees Celsius.
 *
 * Updates intermediate t_fine value used by other compensations.
 *
 * @param  h      Device handle containing calibration data and t_fine.
 * @param  adc_T  Raw 20-bit temperature register value.
 * @return Floating-point temperature in °C.
 */
static float compensate_temp(bme280_handle_t *h, int32_t adc_T)
{
    float var1 = (((float)adc_T / 16384.0f) - ((float)h->calib.dig_T1 / 1024.0f))
                 * (float)h->calib.dig_T2;
    float var2 = (((float)adc_T / 131072.0f - (float)h->calib.dig_T1 / 8192.0f)
                 * ((float)adc_T / 131072.0f - (float)h->calib.dig_T1 / 8192.0f))
                 * (float)h->calib.dig_T3;

    h->t_fine = (int32_t)(var1 + var2);
    return (var1 + var2) / 5120.0f;
}

/**
 * @brief  Compensate raw pressure reading to Pascals.
 *
 * Uses intermediate t_fine from temperature compensation.
 *
 * @param  h      Device handle containing calibration data and t_fine.
 * @param  adc_P  Raw 20-bit pressure register value.
 * @return Floating-point pressure in Pa.
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
        return 0.0f; /* Prevent division by zero */
    }

    int64_t p = 1048576 - adc_P;
    p = (((p << 31) - var2) * 3125) / var1;
    var1 = ((int64_t)h->calib.dig_P9 * (p >> 13) * (p >> 13)) >> 25;
    var2 = ((int64_t)h->calib.dig_P8 * p) >> 19;
    p = ((p + var1 + var2) >> 8) + ((int64_t)h->calib.dig_P7 << 4);

    return (float)p / 256.0f;
}

/**
 * @brief  Compensate raw humidity reading to relative humidity (% RH).
 *
 * Uses intermediate t_fine from temperature compensation.
 * Clamps output between 0 and 100%.
 *
 * @param  h      Device handle containing calibration data and t_fine.
 * @param  adc_H  Raw 16-bit humidity register value.
 * @return Floating-point relative humidity in %.
 */
static float compensate_hum(bme280_handle_t *h, int32_t adc_H)
{
    float var_H = (float)h->t_fine - 76800.0f;
    var_H = (adc_H - ((float)h->calib.dig_H4 * 64.0f +
             ((float)h->calib.dig_H5 / 16384.0f) * var_H)) *
             ((float)h->calib.dig_H2 / 65536.0f) *
             (1.0f + ((float)h->calib.dig_H6 / 67108864.0f) * var_H *
             (1.0f + ((float)h->calib.dig_H3 / 67108864.0f) * var_H));
    var_H = var_H * (1.0f - ((float)h->calib.dig_H1 * var_H / 524288.0f));

    if (var_H > 100.0f) {
        var_H = 100.0f;
    } else if (var_H < 0.0f) {
        var_H = 0.0f;
    }

    return var_H;
}

/**
 * @brief  Initialize BME280 sensor on specified I2C port and address.
 *
 * Performs device registration, reset, calibration data read,
 * and applies initial configuration.
 *
 * @param  h     Pointer to uninitialised handle struct.
 * @param  port  I2C port number (e.g. I2C_NUM_0).
 * @param  addr  7-bit I2C address of sensor (e.g. 0x76 or 0x77).
 * @param  cfg   Optional configuration pointer; if NULL, defaults used.
 * @return ESP_OK on success or appropriate error code.
 */
esp_err_t bme280_init(bme280_handle_t *h, i2c_port_t port, uint8_t addr, const bme280_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    ESP_ERROR_CHECK(i2c_master_get_bus_handle(port, &h->bus));

    i2c_device_config_t dev_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address  = addr,
        .scl_speed_hz    = 100000
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(h->bus, &dev_cfg, &h->dev));

    h->mutex = xSemaphoreCreateMutex();
    ESP_RETURN_ON_FALSE(h->mutex, ESP_ERR_NO_MEM, TAG, "mutex create failed");

    if (cfg) {
        h->config = *cfg;
    } else {
        bme280_get_default_config(&h->config);
    }

    uint8_t chip_id;
    ESP_ERROR_CHECK(read_regs(h, BME280_REG_CHIP_ID, &chip_id, 1));
    ESP_RETURN_ON_FALSE(chip_id == BME280_CHIP_ID, ESP_ERR_NOT_FOUND, TAG, "chip ID mismatch");

    ESP_ERROR_CHECK(write_reg(h, BME280_REG_RESET, BME280_RESET_VALUE));
    vTaskDelay(pdMS_TO_TICKS(10));

    ESP_ERROR_CHECK(read_calibration(h));
    ESP_ERROR_CHECK(write_configuration(h));

    h->initialised = true;
    return ESP_OK;
}

/**
 * @brief  Deinitialize sensor and free resources.
 * @param  h  Initialized handle to deinit.
 * @return ESP_OK on success or error if not initialised.
 */
esp_err_t bme280_deinit(bme280_handle_t *h)
{
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not initialised");
    ESP_ERROR_CHECK(i2c_master_bus_rm_device(h->dev));
    vSemaphoreDelete(h->mutex);
    h->initialised = false;
    return ESP_OK;
}

/**
 * @brief  Apply new configuration settings to sensor.
 * @param  h    Initialized handle.
 * @param  cfg  Pointer to new configuration struct.
 * @return ESP_OK on success or error on bad arguments.
 */
esp_err_t bme280_configure(bme280_handle_t *h, const bme280_config_t *cfg)
{
    ESP_RETURN_ON_FALSE(h && cfg, ESP_ERR_INVALID_ARG, TAG, "invalid args");
    h->config = *cfg;
    return write_configuration(h);
}

/**
 * @brief  Trigger a single measurement in forced mode.
 * @param  h  Initialized handle.
 * @return ESP_OK on success or error if not initialised.
 */
esp_err_t bme280_trigger_measurement(bme280_handle_t *h)
{
    ESP_RETURN_ON_FALSE(h && h->initialised, ESP_ERR_INVALID_STATE, TAG, "not initialised");
    return write_configuration(h);
}

/**
 * @brief  Check if the sensor has completed the last measurement.
 * @param  h      Initialized handle.
 * @param  ready  Pointer to bool set to true when measurement is done.
 * @return ESP_OK on success.
 */
esp_err_t bme280_is_meas_ready(bme280_handle_t *h, bool *ready)
{
    ESP_RETURN_ON_FALSE(h && ready, ESP_ERR_INVALID_ARG, TAG, "invalid args");
    uint8_t status;
    ESP_ERROR_CHECK(read_regs(h, BME280_REG_STATUS, &status, 1));
    *ready = ((status & 0x08) == 0);
    return ESP_OK;
}

/**
 * @brief  Read compensated temperature, pressure, humidity.
 *
 * Reads raw registers, applies compensation, and fills output struct.
 *
 * @param  h    Initialized handle.
 * @param  out  Pointer to bme280_data_t to receive results.
 * @return ESP_OK on success.
 */
esp_err_t bme280_read_data(bme280_handle_t *h, bme280_data_t *out)
{
    ESP_RETURN_ON_FALSE(h && out, ESP_ERR_INVALID_ARG, TAG, "null pointer");

    uint8_t buf[8];
    ESP_ERROR_CHECK(read_regs(h, BME280_REG_DATA, buf, sizeof(buf)));

    int32_t adc_P = (buf[0] << 12) | (buf[1] << 4) | (buf[2] >> 4);
    int32_t adc_T = (buf[3] << 12) | (buf[4] << 4) | (buf[5] >> 4);
    int32_t adc_H = (buf[6] << 8)  | buf[7];

    out->temperature = compensate_temp(h, adc_T);
    out->pressure    = compensate_press(h, adc_P) / 100.0f;
    out->humidity    = compensate_hum(h, adc_H);

    return ESP_OK;
}

/**
 * @brief  Alias for ISR-safe data read (identical to normal read).
 */
esp_err_t bme280_read_data_isr(bme280_handle_t *h, bme280_data_t *out)
{
    return bme280_read_data(h, out);
}

/**
 * @brief  Software reset of the sensor.
 * @param  h  Device handle.
 * @return ESP_OK on success.
 */
esp_err_t bme280_reset(bme280_handle_t *h)
{
    ESP_RETURN_ON_FALSE(h, ESP_ERR_INVALID_ARG, TAG, "null handle");
    return write_reg(h, BME280_REG_RESET, BME280_RESET_VALUE);
}

/**
 * @brief  Retrieve the sensor's chip ID register.
 * @param  h   Device handle.
 * @param  id  Pointer to uint8_t to receive ID.
 * @return ESP_OK on success.
 */
esp_err_t bme280_get_chip_id(bme280_handle_t *h, uint8_t *id)
{
    ESP_RETURN_ON_FALSE(h && id, ESP_ERR_INVALID_ARG, TAG, "null pointer");
    return read_regs(h, BME280_REG_CHIP_ID, id, 1);
}

/**
 * @brief  Compute sea-level equivalent pressure.
 *
 * Applies barometric formula to convert measured pressure
 * at altitude to equivalent sea-level pressure.
 *
 * @param  pressure  Measured pressure in hPa.
 * @param  altitude  Altitude above sea-level in meters.
 * @return Sea-level pressure in hPa.
 */
float bme280_calculate_sea_level_pressure(float pressure, float altitude)
{
    return pressure / powf(1.0f - (altitude / 44330.0f), 5.255f);
}
