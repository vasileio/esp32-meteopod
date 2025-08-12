idf_component_register(SRCS "watchdog.c"
                       INCLUDE_DIRS "."
                       REQUIRES freertos esp_system esp_timer
                       PRIV_REQUIRES main)