idf_component_register(
    SRCS
        "unit_test_main.c"
        "../../../test/test_can_payloads.c"
        "../../../test/test_can_catalog.c"
        "../../../test/test_can_state.c"
        "../../../test/test_can_driver_contract.c"
    INCLUDE_DIRS "."
    REQUIRES can_driver unity
)
