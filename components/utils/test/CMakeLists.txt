idf_component_register(
    SRC_DIRS "."
    INCLUDE_DIRS "../" "../../../main/include"  # include the parent directory's headers
    REQUIRES unity esp_timer driver sensors
)
