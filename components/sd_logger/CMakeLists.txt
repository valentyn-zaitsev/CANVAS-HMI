idf_component_register(
    SRCS "sd_logger.c"
    INCLUDE_DIRS "include"
    REQUIRES driver fatfs vfs sdmmc freertos
)
