#pragma once

#include "can_payloads.h"

// =============================================================================
// can_message_catalog.h — central message-routing catalog.
//
// This table is the single source of truth for normal memcpy-style manager
// routing: ID, payload type, destination state field, and timestamp field.
//
// Add new standard messages by adding one row here and the corresponding state
// struct in can_state.h/can_state.c.
//
// NOTE: The X-macro table intentionally omits commas between entries.
// Usage macros (MAKE_COPY_ROUTE_DESC, etc.) must include a trailing comma
// when used in array initializers, but NOT for count expressions or switch cases.
// =============================================================================

// name, id, payload_type, state_field, last_rx_tick_field
#define CAN_MESSAGE_COPY_ROUTE_TABLE(X) \
    X("PEDAL",   CAN_ID_PEDAL,           PedalPayload,      g_can_pedal.data,  g_can_pedal.last_rx_tick) \
    X("AUX_CTRL",CAN_ID_AUX_CTRL,        AuxControlPayload, g_can_aux.data,    g_can_aux.last_rx_tick) \
    X("PWR_780", CAN_ID_PWR_MONITOR_780, PowerPayload,      g_can_pwr780.data, g_can_pwr780.last_rx_tick) \
    X("PWR_740", CAN_ID_PWR_MONITOR_740, PowerPayload,      g_can_pwr740.data, g_can_pwr740.last_rx_tick)

// Special routes are still cataloged here for uniqueness checks and
// onboarding visibility, but dispatched via custom handlers in can_manager.c.
// name, id
#define CAN_MESSAGE_SPECIAL_ROUTE_TABLE(X) \
    X("PWR_ENERGY", CAN_ID_PWR_ENERGY) \
    X("DASH_STAT",  CAN_ID_DASH_STAT)

// Compile-time row counts for coverage checks.
// Both counters share a single named enum so _Static_assert comparisons
// between them and other enumerators don't trigger -Werror=enum-compare.
#define CAN_MESSAGE_COUNT_COPY(_name, _id, _payload_t, _state_field, _tick_field) +1
#define CAN_MESSAGE_COUNT_SPECIAL(_name, _id) +1

enum CanMessageCatalogCounts {
    CAN_MESSAGE_COPY_ROUTE_COUNT = 0
        CAN_MESSAGE_COPY_ROUTE_TABLE(CAN_MESSAGE_COUNT_COPY),
    CAN_MESSAGE_SPECIAL_ROUTE_COUNT = 0
        CAN_MESSAGE_SPECIAL_ROUTE_TABLE(CAN_MESSAGE_COUNT_SPECIAL)
};

#undef CAN_MESSAGE_COUNT_COPY
#undef CAN_MESSAGE_COUNT_SPECIAL
