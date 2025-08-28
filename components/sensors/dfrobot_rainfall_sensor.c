#include "dfrobot_rainfall_sensor.h"

static const char *TAG = "DFR_RAINFALL";

/*
 * @brief  Write one register + data bytes over I2C
 */
static esp_err_t write_regs_i2c(i2c_master_dev_handle_t dev,
                                uint8_t reg,
                                const uint8_t *buf,
                                size_t len)
{
    uint8_t data[1 + len];
    data[0] = reg;
    memcpy(&data[1], buf, len);
    esp_err_t err = i2c_master_transmit(dev, data, sizeof(data), pdMS_TO_TICKS(200));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

/*
 * @brief  Read len bytes starting at register reg
 */
static esp_err_t read_regs_i2c(i2c_master_dev_handle_t dev,
                               uint8_t reg,
                               uint8_t *buf,
                               size_t len)
{
    esp_err_t err = i2c_master_transmit(dev, &reg, 1, pdMS_TO_TICKS(200));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C write-reg for read 0x%02X failed: %s", reg, esp_err_to_name(err));
        return err;
    }
    err = i2c_master_receive(dev, buf, len, pdMS_TO_TICKS(200));
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C read reg 0x%02X failed: %s", reg, esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t DFRobot_rainfall_sensor_init(DFRobot_rainfall_sensor_t *sensor,
                                       i2c_master_bus_handle_t bus,
                                       uint8_t addr_7bit,
                                       uint32_t clk_speed_hz)
{
    i2c_device_config_t cfg = {
        .device_address  = addr_7bit,
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .scl_speed_hz    = clk_speed_hz
    };
    esp_err_t err = i2c_master_bus_add_device(bus, &cfg, &sensor->dev);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "I2C device add failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}

esp_err_t DFRobot_rainfall_sensor_begin(DFRobot_rainfall_sensor_t *sensor)
{
    uint8_t buf[4];
    esp_err_t err = read_regs_i2c(sensor->dev, DFROBOT_RAINFALL_SENSOR_REG_PID, buf, 4);
    if (err != ESP_OK) {
        return err;
    }

    /* decode PID/VID per vendor driver */
    sensor->pid =  (uint32_t)buf[0]
                 | ((uint32_t)buf[1] << 8)
                 | ((uint32_t)(buf[3] & 0xC0) << 10);
    sensor->vid =  (uint32_t)buf[2]
                 | ((uint32_t)(buf[3] & 0x3F) << 8);

    if (sensor->pid == 0x100C0 && sensor->vid == 0x3343) {
        return ESP_OK;
    }
    return ESP_ERR_NOT_FOUND;
}

esp_err_t DFRobot_rainfall_sensor_get_version(DFRobot_rainfall_sensor_t *sensor,
                                              uint16_t *version)
{
    uint8_t buf[2];
    esp_err_t err = read_regs_i2c(sensor->dev, DFROBOT_RAINFALL_SENSOR_REG_VERSION, buf, 2);
    if (err != ESP_OK) {
        return err;
    }
    *version = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    return ESP_OK;
}

esp_err_t DFRobot_rainfall_sensor_get_cumulative(DFRobot_rainfall_sensor_t *sensor,
                                                 float *out_mm)
{
    uint8_t buf[4];
    esp_err_t err = read_regs_i2c(sensor->dev, DFROBOT_RAINFALL_SENSOR_REG_CUMULATIVE, buf, 4);
    if (err != ESP_OK) {
        return err;
    }
    uint32_t raw =  (uint32_t)buf[0]
                  | ((uint32_t)buf[1] << 8)
                  | ((uint32_t)buf[2] << 16)
                  | ((uint32_t)buf[3] << 24);
    *out_mm = raw / 10000.0f;
    return ESP_OK;
}

esp_err_t DFRobot_rainfall_sensor_get_cumulative_hours(DFRobot_rainfall_sensor_t *sensor,
                                                       uint8_t hours,
                                                       float *out_mm)
{
    if (hours < 1 || hours > 24) {
        return ESP_ERR_INVALID_ARG;
    }

    esp_err_t err = write_regs_i2c(sensor->dev, DFROBOT_RAINFALL_SENSOR_REG_RAIN_HOUR, &hours, 1);
    if (err != ESP_OK) {
        return err;
    }

    // Small delay to allow sensor to process the hour parameter
    vTaskDelay(pdMS_TO_TICKS(10));

    uint8_t buf[4];
    err = read_regs_i2c(sensor->dev, DFROBOT_RAINFALL_SENSOR_REG_TIME_RAINFALL, buf, 4);
    if (err != ESP_OK) {
        return err;
    }

    uint32_t raw =  (uint32_t)buf[0]
                  | ((uint32_t)buf[1] << 8)
                  | ((uint32_t)buf[2] << 16)
                  | ((uint32_t)buf[3] << 24);
    *out_mm = raw / 10000.0f;
    return ESP_OK;
}

esp_err_t DFRobot_rainfall_sensor_get_raw_count(DFRobot_rainfall_sensor_t *sensor,
                                                uint32_t *out_cnt)
{
    uint8_t buf[4];
    esp_err_t err = read_regs_i2c(sensor->dev, DFROBOT_RAINFALL_SENSOR_REG_RAW_DATA, buf, 4);
    if (err != ESP_OK) {
        return err;
    }
    *out_cnt =  (uint32_t)buf[0]
              | ((uint32_t)buf[1] << 8)
              | ((uint32_t)buf[2] << 16)
              | ((uint32_t)buf[3] << 24);
    return ESP_OK;
}

esp_err_t DFRobot_rainfall_sensor_set_bucket_volume(DFRobot_rainfall_sensor_t *sensor,
                                                    float mm_per_tip)
{
    uint16_t v = (uint16_t)(mm_per_tip * 10000.0f);
    uint8_t buf[2] = { (uint8_t)(v & 0xFF), (uint8_t)(v >> 8) };
    return write_regs_i2c(sensor->dev, DFROBOT_RAINFALL_SENSOR_REG_BASE_RAINFALL, buf, 2);
}

esp_err_t DFRobot_rainfall_sensor_get_uptime(DFRobot_rainfall_sensor_t *sensor,
                                             float *out_hr)
{
    uint8_t buf[2];
    esp_err_t err = read_regs_i2c(sensor->dev, DFROBOT_RAINFALL_SENSOR_REG_SYS_TIME, buf, 2);
    if (err != ESP_OK) {
        return err;
    }
    uint16_t mins = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
    *out_hr = mins / 60.0f;
    return ESP_OK;
}
