idf_component_register(
    SRCS "can_driver.c" "obd2_pids.c" "vehicle_data.c" "can_manager.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_common freertos
)
