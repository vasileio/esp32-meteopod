/**
 * @file wind_sensor.c
 * @brief Wind direction sensor driver using analog voltage via ADC
 *
 * This driver reads an analog signal from a wind direction sensor,
 * converts the ADC voltage to current (assuming a shunt resistor),
 * applies a linear calibration, and maps the result to a wind direction.
 */

#include "wind_sensor.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"

#define TAG "WIND"

/* ADC configuration constants */
#define WIND_SENSOR_DIRECTION_ADC_UNIT     ADC_UNIT_1
#define WIND_SENSOR_DIRECTION_ADC_CHANNEL  ADC_CHANNEL_6  /* GPIO34 */
#define WIND_SENSOR_DIRECTION_ATTEN        ADC_ATTEN_DB_12
#define WIND_SENSOR_DIRECTION_VREF         3.3f
#define WIND_SENSOR_DIRECTION_ADC_RES      4095.0f

#define NUM_SAMPLES              16

/**
 * Convert sensor current in mV to wind direction as a string.
 *
 * @param mV Sensor output current in mV.
 * @return Pointer to wind direction string.
 */
static const char *wind_direction_from_mV(float mV)
{
    if (mV < 560.0f) return "N";
    if (mV < 830.0f) return "NE";
    if (mV < 1100.0f) return "E";
    if (mV < 1350.0f) return "SE";
    if (mV < 1700.0f) return "S";
    if (mV < 2000.0f) return "SW";
    if (mV < 2400.0f) return "W";
    return "NW";
}

static adc_oneshot_unit_handle_t adc_handle;

/**
 * Initialize the ADC used for reading the wind direction sensor.
 *
 * @return ESP_OK on success or an error code on failure.
 */
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

/**
 * Read the wind direction from the sensor.
 *
 * This function averages multiple ADC samples, a wind direction string.
 *
 * @param wind_dir Output pointer to wind direction string.
 * @return ESP_OK on success or an error code on failure.
 */
esp_err_t wind_sensor_direction_read(const char **wind_dir)
{
    int total = 0;
    esp_err_t err = ESP_OK;

    /* Average multiple ADC readings for noise reduction */
    for (int i = 0; i < NUM_SAMPLES; ++i) {
        int sample = 0;
        esp_err_t err = adc_oneshot_read(adc_handle, WIND_SENSOR_DIRECTION_ADC_CHANNEL, &sample);
        if (err != ESP_OK) {
            ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(err));
            return err;
        }
        total += sample;
        vTaskDelay(pdMS_TO_TICKS(2));  /* Small delay between samples */
    }

    float avg_raw = total / (float)NUM_SAMPLES;

    /* Convert raw ADC value to voltage and apply offset */
    float voltage = (avg_raw / WIND_SENSOR_DIRECTION_ADC_RES) * WIND_SENSOR_DIRECTION_VREF;
    voltage += CONFIG_WIND_DIRECTION_SENSOR_VOLTAGE_OFFSET_MV / 1000.0f;

    /* Convert to mV */
    float voltage_mV = voltage * 1000.0f;

    /* Determine wind direction from calibrated current */
    *wind_dir = wind_direction_from_mV(voltage_mV);

    ESP_LOGI(TAG, "ADC avg: %.1f | V = %.3f V | Direction: %s",
             avg_raw, voltage, *wind_dir);

    return err;
}
