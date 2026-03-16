#pragma once

#include "can_payloads.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdbool.h>

// =============================================================================
// can_state.h — Live runtime state for each CAN node.
//
// Separation of concerns:
//   can_payloads.h  — wire format (what goes on the bus)
//   can_state.h     — decoded runtime state (what the application reads)
//
// These structs are written by can_manager and read by the application.
// Each struct carries a last_rx_tick timestamp so the application can detect
// a silent/dead node. The staleness threshold is always the application's
// responsibility — the driver has no opinion on what "too old" means.
//
// Thread safety on ESP32-C3 (single core):
//   The manager task writes to these structs. Application tasks read from them.
//   For telemetry fields (float, bool, TickType_t), reads are safe without locks
//   because each field is ≤32 bits and individually atomic on 32-bit RISC-V.
//
//   EXCEPTION: CanStateEnergy_t::joules_780 and joules_740 are 64-bit doubles.
//   A context-switch mid-write can produce a torn read. The manager task wraps
//   energy writes in a critical section. If your application reads energy in a
//   time-critical path, wrap the read in portENTER_CRITICAL / portEXIT_CRITICAL
//   as well.
// =============================================================================

// -----------------------------------------------------------------------------
// CAN_ID_PEDAL (0x110) — Throttle and brake state from the pedal node.
// -----------------------------------------------------------------------------
typedef struct {
    PedalPayload data;         // 2 byte (throttle + brake)
    TickType_t   last_rx_tick;
} CanStatePedal_t;

// -----------------------------------------------------------------------------
// CAN_ID_AUX_CTRL (0x210) — Body control state (lights, wipers, horn).
// Access individual signals via the bitfield: g_can_aux.ctrl.headlights etc.
// -----------------------------------------------------------------------------
typedef struct {
    AuxControlPayload data;    // 1 byte bitfield
    TickType_t        last_rx_tick;
} CanStateAux_t;

// -----------------------------------------------------------------------------
// CAN_ID_PWR_MONITOR_780 (0x310) — INA780 high-current traction monitor.
// -----------------------------------------------------------------------------
typedef struct {
    PowerPayload data;         // 4 bytes (raw ADC counts)
    TickType_t   last_rx_tick;
} CanStatePwr780_t;

// -----------------------------------------------------------------------------
// CAN_ID_PWR_MONITOR_740 (0x311) — INA740 low-current auxiliary monitor.
// -----------------------------------------------------------------------------
typedef struct {
    PowerPayload data;         // 4 bytes (raw ADC counts)
    TickType_t last_rx_tick;
} CanStatePwr740_t;

// -----------------------------------------------------------------------------
// CAN_ID_PWR_ENERGY (0x312) — Accumulated energy from the INA sensors.
// Both INA780 and INA740 interpretations are stored; the application picks
// whichever is appropriate for its context.
// NOTE: doubles are 64-bit — see thread safety note above.
// -----------------------------------------------------------------------------
typedef struct {
    EnergyPayload data;        // 5 bytes (raw 40-bit accumulator)
    TickType_t    last_rx_tick;
} CanStateEnergy_t;
// -----------------------------------------------------------------------------
// CAN_ID_DASH_STAT (0x400) — Dashboard status transmitted by this node.
// Receiving this ID is not expected in normal operation; the struct is reserved
// for configurations where another node echoes dash state back onto the bus.
// -----------------------------------------------------------------------------
typedef struct {
    uint8_t    data[8];          // Raw bytes — decoded by the application
    uint8_t    len;
    TickType_t last_rx_tick;
} CanStateDash_t;

// =============================================================================
// Global state instances — defined in can_state.c, zero-initialized at boot.
// Include this header and read these directly from any application task.
// =============================================================================
extern CanStatePedal_t   g_can_pedal;
extern CanStateAux_t     g_can_aux;
extern CanStatePwr780_t  g_can_pwr780;
extern CanStatePwr740_t  g_can_pwr740;
extern CanStateEnergy_t  g_can_energy;
extern CanStateDash_t    g_can_dash;

// -----------------------------------------------------------------------------
// Thread-Safe Getters - for fields that have more than 32 bits. (doubles, longs, etc.)
// Prevents hazards. For smaller fields (bools, floats) you can read directly from the struct without locks.
// -----------------------------------------------------------------------------

/**
 * @brief Safely copies the 5-byte energy payload to prevent torn reads.
 * Use this instead of accessing g_can_energy.raw directly.
 */
void can_state_get_energy_raw(EnergyPayload *dest);
