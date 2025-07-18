/**
 * @file wind_sensor.c
 * @brief Wind direction sensor driver using analog voltage via ADC
 *
 * This driver reads an analog signal from a wind direction sensor,
 * converts the ADC voltage to millivolts, applies calibration,
 * and maps the result to a wind direction. Wind speed is currently stubbed.
 */

#include "wind_sensor.h"
#include "esp_log.h"
#include "esp_check.h"
#include "esp_adc/adc_oneshot.h"
#include "freertos/FreeRTOS.h"

#define TAG "WIND"

/* ADC configuration constants */
#define WIND_SENSOR_VREF         3.3f
#define WIND_SENSOR_ADC_RES      4095.0f

/* Direction sensor specific */
#define WIND_SENSOR_DIRECTION_ADC_UNIT     ADC_UNIT_1
#define WIND_SENSOR_DIRECTION_ADC_CHANNEL  ADC_CHANNEL_6  /* GPIO34 */
#define WIND_SENSOR_DIRECTION_ATTEN        ADC_ATTEN_DB_12

#define NUM_SAMPLES              16  /* Number of ADC samples for averaging */

static adc_oneshot_unit_handle_t adc_handle_direction;

/**
 * @brief Convert sensor voltage (in mV) to wind direction string.
 *
 * @param mV Voltage in millivolts.
 * @return Pointer to a static string representing the wind direction.
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

/**
 * @brief Perform averaged ADC sampling and convert to voltage in millivolts.
 *
 * @param adc_handle ADC unit handle.
 * @param channel ADC channel to read.
 * @param offset Voltage offset in millivolts (applied after scaling).
 * @param[out] voltage_mV Resulting voltage in millivolts.
 * @return ESP_OK on success, error code on failure.
 */
static esp_err_t wind_sensor_adc_read(adc_oneshot_unit_handle_t adc_handle, int channel, int offset, float *voltage_mV)
{
    int total = 0, i = 0, sample = 0;
    float voltage = 0;
    esp_err_t err = ESP_OK;

    for (i = 0; i < NUM_SAMPLES; ++i) {
        sample = 0;
        err = adc_oneshot_read(adc_handle, channel, &sample);
        if (ESP_OK != err) {
            ESP_LOGE(TAG, "ADC read failed: %s", esp_err_to_name(err));
            return err;
        }
        total += sample;
        vTaskDelay(pdMS_TO_TICKS(2));
    }

    float avg_raw = total / (float)NUM_SAMPLES;
    voltage = (avg_raw / WIND_SENSOR_ADC_RES) * WIND_SENSOR_VREF;
    voltage += offset / 1000.0f;  /* Apply offset in volts */

    *voltage_mV = voltage * 1000.0f;

    ESP_LOGI(TAG, "ADC Channel %d | ADC avg: %.1f | V = %.3f V", channel, avg_raw, voltage);

    return err;
}

/**
 * @brief Initialize ADC channel and configuration.
 *
 * @param channel ADC channel number.
 * @param attenuation ADC attenuation setting.
 * @param[out] adc_handle Output ADC handle.
 * @return ESP_OK on success, error code on failure.
 */
static esp_err_t wind_sensor_adc_init(int channel, int attenuation, adc_oneshot_unit_handle_t *adc_handle)
{
    adc_oneshot_unit_handle_t handle;

    adc_oneshot_unit_init_cfg_t unit_cfg = {
        .unit_id = WIND_SENSOR_DIRECTION_ADC_UNIT,
        .ulp_mode = ADC_ULP_MODE_DISABLE,
    };

    adc_oneshot_chan_cfg_t chan_cfg = {
        .atten = attenuation,
        .bitwidth = ADC_BITWIDTH_DEFAULT,
    };

    ESP_RETURN_ON_ERROR(adc_oneshot_new_unit(&unit_cfg, &handle), TAG, "Failed to init ADC unit");
    ESP_RETURN_ON_ERROR(adc_oneshot_config_channel(handle, channel, &chan_cfg),
                        TAG, "Failed to config ADC channel");

    *adc_handle = handle;
    return ESP_OK;
}

/**
 * @brief Initialise the wind direction and speed sensor subsystem.
 *
 * @return ESP_OK on success, or an error code on failure.
 */
esp_err_t wind_sensor_init(void)
{
    esp_err_t err = ESP_OK;

    err = wind_sensor_adc_init(WIND_SENSOR_DIRECTION_ADC_CHANNEL,
                               WIND_SENSOR_DIRECTION_ATTEN,
                               &adc_handle_direction);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Failed to init ADC for wind direction sensor: %s", esp_err_to_name(err));
        return err;
    }

    return err;
}

/**
 * @brief Read the current wind direction and speed.
 *
 * Direction is derived from ADC voltage using a lookup. Speed is stubbed.
 *
 * @param[out] wind_readings Pointer to a wind_data_t struct to populate.
 * @return ESP_OK on success, or error code on failure.
 */
esp_err_t wind_sensor_read(wind_data_t *wind_readings)
{
    esp_err_t err = ESP_OK;
    float voltage_direction_mV;

    err = wind_sensor_adc_read(adc_handle_direction,
                               WIND_SENSOR_DIRECTION_ADC_CHANNEL,
                               CONFIG_WIND_DIRECTION_SENSOR_VOLTAGE_OFFSET_MV,
                               &voltage_direction_mV);
    if (ESP_OK != err) {
        ESP_LOGE(TAG, "Failed to get reading for wind direction sensor: %s", esp_err_to_name(err));
    }

    wind_readings->direction = wind_direction_from_mV(voltage_direction_mV);

    /* TODO: Replace with real wind speed reading */
    wind_readings->speed = 3;

    return err;
}
