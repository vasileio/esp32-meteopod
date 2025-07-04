idf_component_register(
  SRCS    "wifi.c" "i2c.c"
  INCLUDE_DIRS "."
  REQUIRES
    driver                      # for common driver_*/ APIs
    esp_event
    esp_wifi
    esp_system                  # if you use any system APIs (watchdog, etc.)
    log                         # if you use ESP_LOG*
    nvs_flash
)
