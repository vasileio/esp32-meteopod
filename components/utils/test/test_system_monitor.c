#include "unity.h"
#include "system_monitor.h"
#include "esp_log.h"
#include <stdarg.h>
#include <string.h>

#define LOG_BUF_SIZE 512

static char   s_log_buf[LOG_BUF_SIZE];
static size_t s_log_buf_len;
static int  (*s_original_vprintf)(const char*, va_list);

// This replacement vprintf just appends into s_log_buf
static int test_vprintf(const char *fmt, va_list ap)
{
    int remaining = LOG_BUF_SIZE - s_log_buf_len - 1;
    if (remaining <= 0) {
        return 0; // buffer full
    }
    int written = vsnprintf(s_log_buf + s_log_buf_len, remaining, fmt, ap);
    if (written > 0) {
        s_log_buf_len += written;
    }
    return written;
}

TEST_CASE("log_system_metrics emits the three expected lines", "[system_monitor][logging]")
{
    // 1) Swap in our hook, saving the original
    s_log_buf_len      = 0;
    s_log_buf[0]       = '\0';
    s_original_vprintf = esp_log_set_vprintf(test_vprintf);

    // 2) Call the function under test
    system_metrics_t m = {
        .free_heap       = 1000,
        .min_free_heap   =  900,
        .uptime_ms       =   50,
        .stack_watermark =   20
    };
    log_system_metrics(&m);

    // 3) Restore the original writer so other tests aren’t affected
    esp_log_set_vprintf(s_original_vprintf);

    // 4) Now assert that our buffer contains each expected substring
    TEST_ASSERT_TRUE_MESSAGE(
        strstr(s_log_buf, "Free Heap: 1000 bytes, Min Free Heap: 900 bytes") != NULL,
        "Missing or incorrect first log line"
    );
    TEST_ASSERT_TRUE_MESSAGE(
        strstr(s_log_buf, "Uptime: 50 ms") != NULL,
        "Missing or incorrect uptime line"
    );
    TEST_ASSERT_TRUE_MESSAGE(
        strstr(s_log_buf, "Task Stack High Water Mark: 20 bytes") != NULL,
        "Missing or incorrect stack watermark line"
    );
}

TEST_CASE("get_system_metrics returns sane, non crashing values", "[system_monitor]")
{
    // Snapshot #1
    system_metrics_t m1 = get_system_metrics();

    // Basic expectations
    TEST_ASSERT_TRUE_MESSAGE(m1.free_heap > 0,
        "Free heap must be > 0");
    TEST_ASSERT_TRUE_MESSAGE(m1.min_free_heap > 0,
        "Min free heap must be > 0");
    TEST_ASSERT_LESS_OR_EQUAL_UINT_MESSAGE(
        m1.min_free_heap, m1.free_heap,
        "min_free_heap should be <= free_heap");
    TEST_ASSERT_TRUE_MESSAGE(m1.uptime_ms >= 0,
        "uptime_ms must be non-negative");
    TEST_ASSERT_TRUE_MESSAGE(m1.stack_watermark > 0,
        "stack_watermark must be > 0");

    // Wait a bit and ensure uptime is non-decreasing
    vTaskDelay(pdMS_TO_TICKS(10));
    system_metrics_t m2 = get_system_metrics();
    TEST_ASSERT_TRUE_MESSAGE(
        m2.uptime_ms >= m1.uptime_ms,
        "uptime_ms should not go backwards");
}
