#pragma once

#include "esp_err.h"

// =============================================================================
// can_manager.h — RX dispatch and state update layer.
//
// Responsibilities:
//   - Owns the FreeRTOS manager task (static allocation, no heap).
//   - Drains the RX queue provided by can_driver.
//   - Decodes raw CanRxEvent_t frames using can_payloads.h helpers.
//   - Writes decoded values into the global state structs in can_state.h.
//   - Polls the bus-off flag and calls can_driver_recover() when needed.
//
// The application never calls anything in this file after init.
// It simply reads from the global state structs in can_state.h.
// =============================================================================

// Start the manager task. Must be called after can_driver_init().
esp_err_t can_manager_init(void);

// Stop the manager task. Call before can_driver_deinit().
esp_err_t can_manager_deinit(void);
