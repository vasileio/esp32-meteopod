idf_component_register(
  SRCS        "dfrobot_rainfall_sensor.c" "sensors.c" "sht31.c" "bme280.c" "wind_sensor.c" "bh1750.c" "mpu6050.c"
  INCLUDE_DIRS "." "../../main/include"
  REQUIRES     log driver freertos drivers esp_rom esp_timer esp_common esp_adc
)
