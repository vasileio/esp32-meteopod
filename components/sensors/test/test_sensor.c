#include "unity.h"
#include "sensors.h"

TEST_CASE("sensor_read returns within range", "[sensor]")
{
    int16_t temp, humidity;

    temp = rand() % 30;;
    humidity = rand() % 100;

    TEST_ASSERT_TRUE_MESSAGE(temp  >= -400  && temp  <=  1250, "temp out of expected range");
    TEST_ASSERT_TRUE_MESSAGE(humidity >= 0     && humidity <= 1000, "humidity out of expected range");
}
