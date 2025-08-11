idf_component_register(SRCS "main.c"
                       INCLUDE_DIRS "include"
                       REQUIRES esp_app_format esp_timer sensors utils watchdog)
