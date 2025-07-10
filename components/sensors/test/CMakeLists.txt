idf_component_register(
    SRC_DIRS "."
    SRCS
        i2c_test_stubs.c
        test_dfrobot_rainfall_sensor.c
    INCLUDE_DIRS "../"  # include the parent directory's headers
    REQUIRES unity driver drivers
)
