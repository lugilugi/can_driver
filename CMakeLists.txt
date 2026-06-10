# =============================================================================
# CAN Driver Component — ESP-IDF CMakeLists.txt
# =============================================================================

set(can_srcs
    "can_driver.c"
    "network.c"
)

set(can_requires
    esp_driver_twai
    freertos
    log
)

idf_component_register(
    SRCS
        ${can_srcs}
    INCLUDE_DIRS
        "include"
    REQUIRES
        ${can_requires}
)
