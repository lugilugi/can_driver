#include "unity.h"

#include "can_message_catalog.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

typedef struct {
    const char *name;
    uint32_t id;
    size_t payload_size;
} CopyRouteMeta_t;

typedef struct {
    const char *name;
    uint32_t id;
} SpecialRouteMeta_t;

#define MAKE_COPY_META(_name, _id, _payload_t, _state_field, _tick_field) \
    { (_name), (_id), sizeof(_payload_t) }

#define MAKE_SPECIAL_META(_name, _id) \
    { (_name), (_id) }

static const CopyRouteMeta_t s_copy_routes[] = {
    CAN_MESSAGE_COPY_ROUTE_TABLE(MAKE_COPY_META)
};

static const SpecialRouteMeta_t s_special_routes[] = {
    CAN_MESSAGE_SPECIAL_ROUTE_TABLE(MAKE_SPECIAL_META)
};

static bool id_exists_in_catalog(uint32_t id)
{
    for (size_t i = 0; i < (sizeof(s_copy_routes) / sizeof(s_copy_routes[0])); i++) {
        if (s_copy_routes[i].id == id) {
            return true;
        }
    }
    for (size_t i = 0; i < (sizeof(s_special_routes) / sizeof(s_special_routes[0])); i++) {
        if (s_special_routes[i].id == id) {
            return true;
        }
    }
    return false;
}

TEST_CASE("catalog table counts match generated rows", "[can][catalog]")
{
    TEST_ASSERT_EQUAL_UINT32((uint32_t)CAN_MESSAGE_COPY_ROUTE_COUNT,
                             (uint32_t)(sizeof(s_copy_routes) / sizeof(s_copy_routes[0])));
    TEST_ASSERT_EQUAL_UINT32((uint32_t)CAN_MESSAGE_SPECIAL_ROUTE_COUNT,
                             (uint32_t)(sizeof(s_special_routes) / sizeof(s_special_routes[0])));
}

TEST_CASE("catalog IDs are unique across all routes", "[can][catalog]")
{
    for (size_t i = 0; i < (sizeof(s_copy_routes) / sizeof(s_copy_routes[0])); i++) {
        for (size_t j = i + 1; j < (sizeof(s_copy_routes) / sizeof(s_copy_routes[0])); j++) {
            TEST_ASSERT_NOT_EQUAL(s_copy_routes[i].id, s_copy_routes[j].id);
        }
        for (size_t j = 0; j < (sizeof(s_special_routes) / sizeof(s_special_routes[0])); j++) {
            TEST_ASSERT_NOT_EQUAL(s_copy_routes[i].id, s_special_routes[j].id);
        }
    }

    for (size_t i = 0; i < (sizeof(s_special_routes) / sizeof(s_special_routes[0])); i++) {
        for (size_t j = i + 1; j < (sizeof(s_special_routes) / sizeof(s_special_routes[0])); j++) {
            TEST_ASSERT_NOT_EQUAL(s_special_routes[i].id, s_special_routes[j].id);
        }
    }
}

TEST_CASE("catalog route metadata is valid", "[can][catalog]")
{
    for (size_t i = 0; i < (sizeof(s_copy_routes) / sizeof(s_copy_routes[0])); i++) {
        TEST_ASSERT_NOT_NULL(s_copy_routes[i].name);
        TEST_ASSERT_TRUE(s_copy_routes[i].name[0] != '\0');
        TEST_ASSERT_TRUE(s_copy_routes[i].id <= 0x7FFU);
        TEST_ASSERT_TRUE(s_copy_routes[i].payload_size > 0);
        TEST_ASSERT_TRUE(s_copy_routes[i].payload_size <= 8);
    }

    for (size_t i = 0; i < (sizeof(s_special_routes) / sizeof(s_special_routes[0])); i++) {
        TEST_ASSERT_NOT_NULL(s_special_routes[i].name);
        TEST_ASSERT_TRUE(s_special_routes[i].name[0] != '\0');
        TEST_ASSERT_TRUE(s_special_routes[i].id <= 0x7FFU);
    }
}

TEST_CASE("all public message IDs are represented in catalog", "[can][catalog]")
{
    const uint32_t expected_ids[] = {
        CAN_ID_PEDAL,
        CAN_ID_AUX_CTRL,
        CAN_ID_PWR_MONITOR_780,
        CAN_ID_PWR_MONITOR_740,
        CAN_ID_PWR_ENERGY,
        CAN_ID_DASH_STAT,
    };

    for (size_t i = 0; i < (sizeof(expected_ids) / sizeof(expected_ids[0])); i++) {
        TEST_ASSERT_TRUE(id_exists_in_catalog(expected_ids[i]));
    }
}
