// test_sht31.c

#include "unity.h"
#include "sht31.h"
#include "esp_err.h"
#include <string.h>

/* pull in the shared stubs from i2c_test_stubs.c */
extern esp_err_t   stub_transmit_ret;
extern esp_err_t   stub_receive_ret;
extern uint8_t     fake_buf[];

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
