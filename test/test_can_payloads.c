#include "unity.h"
#include "can_payloads.h"
#include "can_message_catalog.h"

#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char *name;
    uint32_t id;
    size_t payload_size;
} CanCatalogMeta_t;

#define MAKE_CATALOG_META(_name, _id, _payload_t, _state_field, _tick_field) \
    { (_name), (_id), sizeof(_payload_t) }

static const CanCatalogMeta_t s_copy_routes[] = {
    CAN_MESSAGE_COPY_ROUTE_TABLE(MAKE_CATALOG_META)
};

TEST_CASE("payload wire sizes are stable", "[can][payload]")
{
    TEST_ASSERT_EQUAL_UINT32(6, sizeof(PedalPayload));
    TEST_ASSERT_EQUAL_UINT32(1, sizeof(AuxControlPayload));
    TEST_ASSERT_EQUAL_UINT32(4, sizeof(PowerPayload));
    TEST_ASSERT_EQUAL_UINT32(5, sizeof(EnergyPayload));
}

TEST_CASE("payload helper conversions are deterministic", "[can][payload]")
{
    PedalPayload pedal = {0};
    const uint16_t raw_throttle = 16384;
    pedal.filtered_throttle = (uint16_t)(0x8000U | raw_throttle);

    TEST_ASSERT_TRUE(PedalPayload_isKillActive(&pedal));
    TEST_ASSERT_EQUAL_UINT16(raw_throttle, PedalPayload_getThrottleRaw(&pedal));
    TEST_ASSERT_FLOAT_WITHIN(0.01f,
                             (float)raw_throttle * (100.0f / 32767.0f),
                             PedalPayload_getThrottle(&pedal));

    PowerPayload power = {0};
    PowerPayload_setRaw(&power, 16000, 2000);
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 50.0f, PowerPayload_getVoltage(&power));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 4.8f, PowerPayload_getCurrent_780(&power));
    TEST_ASSERT_FLOAT_WITHIN(0.001f, 2.4f, PowerPayload_getCurrent_740(&power));

    EnergyPayload energy = {0};
    const uint64_t raw_energy = 1000000ULL;
    EnergyPayload_setRaw(&energy, raw_energy);
    TEST_ASSERT_DOUBLE_WITHIN(0.001,
                              (double)raw_energy * ENERGY_LSB_JOULES_780,
                              EnergyPayload_getJoules_780(&energy));
    TEST_ASSERT_DOUBLE_WITHIN(0.001,
                              (double)raw_energy * ENERGY_LSB_JOULES_740,
                              EnergyPayload_getJoules_740(&energy));
}

TEST_CASE("catalog copy routes are unique and CAN-safe", "[can][catalog]")
{
    TEST_ASSERT_EQUAL_UINT32((uint32_t)CAN_MESSAGE_COPY_ROUTE_COUNT,
                             (uint32_t)(sizeof(s_copy_routes) / sizeof(s_copy_routes[0])));

    for (size_t i = 0; i < (sizeof(s_copy_routes) / sizeof(s_copy_routes[0])); i++) {
        TEST_ASSERT_TRUE(s_copy_routes[i].payload_size > 0);
        TEST_ASSERT_TRUE(s_copy_routes[i].payload_size <= 8);

        for (size_t j = i + 1; j < (sizeof(s_copy_routes) / sizeof(s_copy_routes[0])); j++) {
            TEST_ASSERT_NOT_EQUAL(s_copy_routes[i].id, s_copy_routes[j].id);
        }
    }
}
