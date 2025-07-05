idf_component_register(
  SRCS    "sht31.c" "dfrobot_rainfall_sensor.c"
  INCLUDE_DIRS "." "../../main/include"
  REQUIRES     log driver freertos drivers esp_rom esp_timer
)
