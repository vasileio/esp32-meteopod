// test_sht31.c

#include "unity.h"
#include "sensors.h"        /* your public sensor API header */
#include "esp_err.h"
#include <string.h>

/*--------------------------------------------------
 * Stubs for IDF / FreeRTOS symbols
 *-------------------------------------------------*/
static esp_err_t stub_transmit_ret;
static esp_err_t stub_receive_ret;
static uint8_t    fake_buf[6];

/* global handle used by driver */
i2c_master_dev_handle_t dev_handle;

/* pretend adding the device always succeeds */
esp_err_t i2c_master_bus_add_device(i2c_master_bus_handle_t bus,
                                    const i2c_device_config_t *cfg,
                                    i2c_master_dev_handle_t *out_handle)
{
    (void)bus; (void)cfg;
    *out_handle = (i2c_master_dev_handle_t)0x1234;
    return ESP_OK;
}

/* transmit stub: return whatever test sets */
esp_err_t i2c_master_transmit(i2c_master_dev_handle_t dev,
                              const uint8_t *data,
                              size_t len,
                              int xfer_timeout_ms)
{
    (void)dev; (void)data; (void)len; (void)xfer_timeout_ms;
    return stub_transmit_ret;
}

/* receive stub: copy from fake_buf, return whatever test sets */
esp_err_t i2c_master_receive(i2c_master_dev_handle_t dev,
                             uint8_t *data,
                             size_t len,
                             int xfer_timeout_ms)
{
    (void)dev; (void)xfer_timeout_ms;
    memcpy(data, fake_buf, len);
    return stub_receive_ret;
}

/* simple CRC‐8-D poly(0x31), init 0xFF for stubbing esp_rom_crc8_be */
static uint8_t __crc8_be(uint8_t crc, const uint8_t *buf, uint32_t len)
{
    const uint8_t poly = 0x31;
    for (uint32_t i = 0; i < len; i++) {
        crc ^= buf[i];
        for (int b = 0; b < 8; b++) {
            if (crc & 0x80) crc = (crc << 1) ^ poly;
            else           crc <<= 1;
        }
    }
    return crc;
}

/* override ROM routine */
uint8_t esp_rom_crc8_be(uint8_t crc, const uint8_t *buf, uint32_t len)
{
    return __crc8_be(crc, buf, len);
}

/*--------------------------------------------------
 * Unity test setup/teardown
 *-------------------------------------------------*/
void setUp(void)
{
    stub_transmit_ret = ESP_OK;
    stub_receive_ret  = ESP_OK;
    memset(fake_buf, 0, sizeof(fake_buf));
}

void tearDown(void) { }

/*--------------------------------------------------
 * Test cases
 *-------------------------------------------------*/

TEST_CASE("sht31_init succeeds with valid params", "[sht31]") {
    esp_err_t err = sht31_init(NULL, 0x44, 100000);
    TEST_ASSERT_EQUAL(ESP_OK, err);
}

TEST_CASE("sht31_read propagates transmit error", "[sht31]") {
    stub_transmit_ret = ESP_ERR_TIMEOUT;
    float t, h;
    TEST_ASSERT_EQUAL(ESP_ERR_TIMEOUT, sht31_read(&t, &h));
}

TEST_CASE("sht31_read propagates receive error", "[sht31]") {
    stub_receive_ret = ESP_ERR_INVALID_RESPONSE;
    float t, h;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_RESPONSE, sht31_read(&t, &h));
}

TEST_CASE("sht31_read detects CRC mismatch", "[sht31]") {
    /* leave fake_buf zeroed → CRC will fail */
    stub_transmit_ret = ESP_OK;
    stub_receive_ret  = ESP_OK;

    float t, h;
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_CRC, sht31_read(&t, &h));
}

TEST_CASE("sht31_read returns valid values for mid-range raw data", "[sht31]") {
    uint16_t raw_t = 32768;
    uint16_t raw_h = 32768;

    fake_buf[0] = raw_t >> 8;
    fake_buf[1] = raw_t & 0xFF;
    fake_buf[3] = raw_h >> 8;
    fake_buf[4] = raw_h & 0xFF;

    /* compute valid CRCs */
    fake_buf[2] = __crc8_be(SHT31_CRC8_INIT, fake_buf,    2);
    fake_buf[5] = __crc8_be(SHT31_CRC8_INIT, fake_buf + 3, 2);

    stub_transmit_ret = ESP_OK;
    stub_receive_ret  = ESP_OK;

    float temperature, humidity;
    TEST_ASSERT_EQUAL(ESP_OK, sht31_read(&temperature, &humidity));

    /* expected conversions */
    float exp_t = -45.0f + 175.0f * ((float)raw_t / 65535.0f);
    float exp_h = 100.0f  * ((float)raw_h / 65535.0f);

    TEST_ASSERT_FLOAT_WITHIN(0.01, exp_t, temperature);
    TEST_ASSERT_FLOAT_WITHIN(0.01, exp_h, humidity);
}
