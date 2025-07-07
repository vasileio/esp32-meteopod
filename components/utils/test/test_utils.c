// components/utils/test_utils.c

#include "unity.h"
#include "utils.h"
#include "esp_err.h"
#include <string.h>

/// A reasonable max for our tests
#define BUF_LEN 32

TEST_CASE("utils_build_topic concatenates prefix and suffix", "[utils][mqtt]")
{
    char buf[BUF_LEN];

    esp_err_t err = utils_build_topic("meteopod/ABCDEF012345", "status",
                                      buf, sizeof(buf));
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("meteopod/ABCDEF012345/status", buf);
}

TEST_CASE("utils_build_topic with empty prefix produces '/suffix'", "[utils][mqtt]")
{
    char buf[BUF_LEN];

    esp_err_t err = utils_build_topic("", "foo", buf, sizeof(buf));
    TEST_ASSERT_EQUAL(ESP_OK, err);
    TEST_ASSERT_EQUAL_STRING("/foo", buf);
}

TEST_CASE("utils_build_topic null arguments are rejected", "[utils][mqtt]")
{
    char buf[BUF_LEN];

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        utils_build_topic(NULL,   "bar", buf, sizeof(buf)));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        utils_build_topic("pre",  NULL,  buf, sizeof(buf)));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        utils_build_topic("pre",  "bar", NULL,    sizeof(buf)));

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
        utils_build_topic("pre",  "bar", buf,     0));
}

TEST_CASE("utils_build_topic reports buffer too small", "[utils][mqtt]")
{
    char small[8]; // deliberately too small
    // prefix "abc", slash, suffix "longsuffix" -> total >8
    esp_err_t err = utils_build_topic("abc", "longsuffix", small, sizeof(small));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_SIZE, err);
    // buffer should still be NUL-terminated
    TEST_ASSERT_EQUAL_CHAR('\0', small[sizeof(small)-1]);
}
