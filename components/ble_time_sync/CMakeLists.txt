if(CONFIG_BT_ENABLED)
    idf_component_register(
        SRCS "ble_time_sync.c"
        INCLUDE_DIRS "include"
        REQUIRES bt esp_common freertos nvs_flash
    )
else()
    idf_component_register(
        INCLUDE_DIRS "include"
    )
endif()
