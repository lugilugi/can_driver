idf_component_register(
    SRCS "can_driver.c"
    INCLUDE_DIRS "include"
    REQUIRES driver esp_driver_twai freertos log
)