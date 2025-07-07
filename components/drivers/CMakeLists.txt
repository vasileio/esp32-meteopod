idf_component_register(
  SRCS    "wifi.c" "i2c.c" "uart.c" "mqtt.c"
  INCLUDE_DIRS "." "../../main/include"
  REQUIRES
    driver                      # for common driver_*/ APIs
    esp_event
    esp_timer
    esp_wifi
    esp_system                  # if you use any system APIs (watchdog, etc.)
    log                         # if you use ESP_LOG*
    nvs_flash
    sensors
    mqtt
    utils
)
