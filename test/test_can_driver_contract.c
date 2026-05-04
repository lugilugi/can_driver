#include "unity.h"

#include "can_driver.h"
#include "can_payloads.h"

TEST_CASE("driver API returns invalid state before init", "[can][driver]")
{
    uint8_t b = 0x5A;
    CanRxEvent_t evt = {0};

    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      can_driver_transmit(CAN_ID_PEDAL, &b, 1));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      can_driver_receive(&evt, 0));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      can_driver_recover());
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      can_driver_deinit());
}

TEST_CASE("driver receive validates null event argument", "[can][driver]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      can_driver_receive(NULL, 0));
}

TEST_CASE("driver filter API argument validation works", "[can][driver]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      can_driver_apply_single_filter_auto(NULL, 1, false));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_ARG,
                      can_driver_apply_dual_filter_auto(NULL, 1, false));
}

TEST_CASE("driver filter APIs reject uninitialized state", "[can][driver]")
{
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      can_driver_apply_single_filter(0x123, 0x7FF, false));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      can_driver_apply_dual_filter(0x100, 0x700, 0x200, 0x700, false));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      can_driver_apply_single_filter_auto(NULL, 0, false));
    TEST_ASSERT_EQUAL(ESP_ERR_INVALID_STATE,
                      can_driver_apply_dual_filter_auto(NULL, 0, false));
}

TEST_CASE("driver ISR counters reset to zero", "[can][driver]")
{
    can_driver_reset_isr_counters();

    TEST_ASSERT_EQUAL_UINT32(0, can_driver_get_isr_rx_calls());
    TEST_ASSERT_EQUAL_UINT32(0, can_driver_get_isr_rx_fail());
    TEST_ASSERT_EQUAL_UINT32(0, can_driver_get_isr_rx_dropped());
}
