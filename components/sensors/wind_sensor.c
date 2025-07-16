/**
 * @file wind_sensor.c
 * @brief Wind direction sensor driver using analog voltage via ADC
 *
 * Reads voltage from ADC, converts to current (mA) using shunt resistor,
 * and optionally applies a linear calibration mapping.
 */

#include "wind_sensor.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"


#define TAG "WIND"

#define WIND_SENSOR_DIRECTION_ADC_UNIT     ADC_UNIT_1
#define WIND_SENSOR_DIRECTION_ADC_CHANNEL  ADC_CHANNEL_6  // GPIO34
#define WIND_SENSOR_DIRECTION_ATTEN        ADC_ATTEN_DB_12
#define WIND_SENSOR_DIRECTION_VREF         3.3f
#define WIND_SENSOR_DIRECTION_ADC_RES      4095.0f

#define R_SHUNT_OHMS             100.0f
#define RAW_MIN_VOLTAGE          0.4f
#define RAW_MAX_VOLTAGE          2.0f
#define CAL_MIN_CURRENT_MA       4.0f
#define CAL_MAX_CURRENT_MA       20.0f

#define NUM_SAMPLES              16

static const char *wind_direction_from_mA(float mA)
{
    if (mA < 5.0f) return "N";
    if (mA < 7.4f) return "NE";
    if (mA < 10.0f) return "E";
    if (mA < 12.5f) return "SE";
    if (mA < 14.8f) return "S";
    if (mA < 16.9f) return "SW";
    if (mA < 19.0f) return "W";
    return "NW";
}

static adc_oneshot_unit_handle_t adc_handle;

esp_err_t wind_sensor_direction_init(void)
{
    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = WIND_SENSOR_DIRECTION_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &adc_handle), TAG, "Failed to init ADC unit");

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = WIND_SENSOR_DIRECTION_ATTEN,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(adc_handle, WIND_SENSOR_DIRECTION_ADC_CHANNEL, &chan_cfg),
                        TAG, "Failed to config ADC channel");

    ESP_LOGI(TAG, "Wind direction sensor ADC initialised");
    return ESP_OK;
}

esp_err_t wind_sensor_direction_read(const char **wind_dir)
{
    int total = 0;
    esp_err_t err = ESP_OK;

    for (int i = 0; i < NUM_SAMPLES; ++i) {
        int sample = 0;
        esp_err_t err = adc_oneshot_read(adc_handle, WIND_SENSOR_DIRECTION_ADC_CHANNEL, &sample);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(err));
            return err;
        }
        total += sample;
        vTaskDelay(pdMS_TO_TICKS(2));  // small delay between samples
    }

    float avg_raw = total / (float)NUM_SAMPLES;
    float voltage = (avg_raw / WIND_SENSOR_DIRECTION_ADC_RES) * WIND_SENSOR_DIRECTION_VREF;
    float current_mA = (voltage / R_SHUNT_OHMS) * 1000.0f;

    float calibrated_mA = current_mA;
    if (voltage >= RAW_MIN_VOLTAGE && voltage <= RAW_MAX_VOLTAGE) {
        float slope = (CAL_MAX_CURRENT_MA - CAL_MIN_CURRENT_MA) /
                      (RAW_MAX_VOLTAGE - RAW_MIN_VOLTAGE);
        calibrated_mA = CAL_MIN_CURRENT_MA + slope * (voltage - RAW_MIN_VOLTAGE);
    }

    *wind_dir = wind_direction_from_mA(calibrated_mA);

    ESP_LOGI(TAG, "ADC avg: %.1f | V = %.3f V | I = %.2f mA | Calibrated = %.2f mA | Direction: %s",
             avg_raw, voltage, current_mA, calibrated_mA, *wind_dir);

    return err;
}
