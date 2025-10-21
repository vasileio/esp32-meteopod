idf_component_register(
    SRCS
        i2c_test_stubs.c
        test_dfrobot_rainfall_sensor.c
        test_dfrobot_as3935.c
        test_sht31.c
        test_bh1750.c
        test_bme280.c
        test_mpu6050.c
        test_wind_sensor.c
    INCLUDE_DIRS "../"  # include the parent directory's headers
    REQUIRES unity driver drivers freertos esp_driver_gpio esp_adc
)
