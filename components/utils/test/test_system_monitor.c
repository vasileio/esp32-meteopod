#include "unity.h"
#include "system_monitor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_timer.h"

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

TEST_CASE("log_system_metrics runs without crashing", "[system_monitor]")
{
    system_metrics_t m = {
        .free_heap       = 1234,
        .min_free_heap   =  567,
        .uptime_ms       =  890,
        .stack_watermark =   42
    };

    // We won't capture the UART output here,
    // but we at least verify that calling it doesn't fault.
    log_system_metrics(&m);

    TEST_PASS(); // always succeed if we get here
}
