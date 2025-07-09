idf_component_register(
  SRCS        "dfrobot_rainfall_sensor.c" "sensors.c" "sht31.c"
  INCLUDE_DIRS "." "../../main/include"
  REQUIRES     log driver freertos drivers esp_rom esp_timer
)
