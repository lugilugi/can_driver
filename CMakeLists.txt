# =============================================================================
# CAN Driver Component — ESP-IDF CMakeLists.txt
# =============================================================================

# Core source files — always compiled.
# can_logger.c and can_usb_forward.c gate themselves internally via
# CAN_LOGGER_ENABLED / CAN_USB_FORWARD_ENABLED in can_config.h.
set(can_srcs
    "can_state.c"
    "can_driver.c"
    "can_manager.c"
    "can_selftest.c"
    "can_logger.c"
    "can_usb_forward.c"
)

# Core dependencies — always required.
set(can_requires
    esp_driver_twai
    freertos
    log
    driver
)

# Optional: SD card logger dependencies.
# Enable this when CAN_LOGGER_ENABLED is set to 1 in include/can_config.h.
# When the logger is disabled (default), these deps are not needed and skipping
# them reduces build time and dependency surface.
#
# To enable, pass -DCAN_LOGGER_PULL_DEPS=ON to idf.py or set it in your
# project-level CMakeLists.txt before this component is processed.
option(CAN_LOGGER_PULL_DEPS "Pull SD card dependencies for CAN logger" OFF)

if(CAN_LOGGER_PULL_DEPS)
    list(APPEND can_requires
        esp_driver_sdmmc
        esp_driver_sdspi
        fatfs
    )
endif()

idf_component_register(
    SRCS
        ${can_srcs}
    INCLUDE_DIRS
        "include"
    REQUIRES
        ${can_requires}
)
