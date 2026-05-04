#include "unity.h"

#include "can_state.h"

#include <string.h>

static EnergyPayload make_energy_payload(uint8_t b0,
                                         uint8_t b1,
                                         uint8_t b2,
                                         uint8_t b3,
                                         uint8_t b4)
{
    EnergyPayload p = { .rawEnergy = { b0, b1, b2, b3, b4 } };
    return p;
}

TEST_CASE("energy state setter/getter roundtrip", "[can][state]")
{
    const TickType_t expected_tick = (TickType_t)1234;
    const EnergyPayload expected = make_energy_payload(0x10, 0x22, 0x34, 0x56, 0x78);
    EnergyPayload actual = {0};

    memset(&g_can_energy, 0, sizeof(g_can_energy));

    can_state_set_energy_raw(&expected, expected_tick);
    can_state_get_energy_raw(&actual);

    TEST_ASSERT_EQUAL_UINT32((uint32_t)expected_tick, (uint32_t)g_can_energy.last_rx_tick);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(expected.rawEnergy, actual.rawEnergy, 5);
}

TEST_CASE("energy state null pointer access is ignored", "[can][state]")
{
    const TickType_t baseline_tick = (TickType_t)777;
    const EnergyPayload baseline = make_energy_payload(1, 2, 3, 4, 5);
    EnergyPayload snapshot = {0};

    memset(&g_can_energy, 0, sizeof(g_can_energy));
    can_state_set_energy_raw(&baseline, baseline_tick);

    can_state_set_energy_raw(NULL, (TickType_t)9999);
    can_state_get_energy_raw(NULL);
    can_state_get_energy_raw(&snapshot);

    TEST_ASSERT_EQUAL_UINT32((uint32_t)baseline_tick, (uint32_t)g_can_energy.last_rx_tick);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(baseline.rawEnergy, snapshot.rawEnergy, 5);
}

TEST_CASE("energy state keeps latest payload", "[can][state]")
{
    const EnergyPayload first = make_energy_payload(0xAA, 0xBB, 0xCC, 0xDD, 0xEE);
    const EnergyPayload second = make_energy_payload(0x11, 0x22, 0x33, 0x44, 0x55);
    EnergyPayload snapshot = {0};

    memset(&g_can_energy, 0, sizeof(g_can_energy));

    can_state_set_energy_raw(&first, (TickType_t)10);
    can_state_set_energy_raw(&second, (TickType_t)20);
    can_state_get_energy_raw(&snapshot);

    TEST_ASSERT_EQUAL_UINT32(20U, (uint32_t)g_can_energy.last_rx_tick);
    TEST_ASSERT_EQUAL_UINT8_ARRAY(second.rawEnergy, snapshot.rawEnergy, 5);
}
