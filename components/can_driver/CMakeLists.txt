idf_component_register(
    SRCS "can_driver.c" "obd2_pids.c" "vehicle_data.c" "can_manager.c" "can_sniffer.c" "mercedes_decode.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_common freertos
)
