#include "i2c.h"

static const char *TAG = "I2C";


esp_err_t i2c_init(i2c_master_bus_handle_t *out_bus_handle)
{
    i2c_master_bus_config_t bus_cfg = {
        .i2c_port          = I2C_PORT,
        .sda_io_num        = I2C_SDA_PIN,
        .scl_io_num        = I2C_SCL_PIN,
        .clk_source        = I2C_CLK_SRC_DEFAULT,      // ← must specify
        .glitch_ignore_cnt = I2C_GLITCH_CNT,
        .flags.enable_internal_pullup = true,
    };

    esp_err_t err = i2c_new_master_bus(&bus_cfg, out_bus_handle);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "i2c_new_master_bus failed: %s", esp_err_to_name(err));
        return err;
    }
    return ESP_OK;
}
