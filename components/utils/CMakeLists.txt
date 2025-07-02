idf_component_register(
  SRCS    "system_monitor.c"
  INCLUDE_DIRS "."
  REQUIRES     log esp_timer driver esp_system freertos
)
