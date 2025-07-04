idf_component_register(
  SRCS    "sensors.c" "sht31.c"
  INCLUDE_DIRS "."
  REQUIRES     log driver freertos drivers esp_rom
)
