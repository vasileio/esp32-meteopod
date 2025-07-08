idf_component_register(
  SRCS    "ota_update.c"
  INCLUDE_DIRS "." "../../main/include"
  REQUIRES     log esp_system esp_https_ota spi_flash esp_partition app_update esp_https_ota drivers
)
