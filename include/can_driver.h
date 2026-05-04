#pragma once

#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>
#include "hal/gpio_types.h"

// =============================================================================
// can_driver.h — TWAI hardware abstraction layer.
//
// Responsibilities:
//   - Owns the TWAI node handle and peripheral lifecycle (init / deinit).
//   - Owns the static TX pool. Frames are zero-copy after claiming a slot.
//   - ISR callbacks: timestamps arriving frames, feeds the RX queue, marks TX
//     slots free, sets the bus-off flag.
//   - Exposes can_driver_receive() for the manager task to drain the RX queue.
//   - Exposes can_driver_recover() for bus-off recovery from task context.
//
// This layer is payload-agnostic. Encoding and decoding is the manager's job.
// =============================================================================

// -----------------------------------------------------------------------------
// Init flags
//
// Passed to can_driver_init() as a compound literal:
//
//   can_driver_init((CanInitFlags_t){0});                    // normal
//   can_driver_init((CanInitFlags_t){ .loopback = 1 });      // bench/selftest
//   can_driver_init((CanInitFlags_t){ .listen_only = 1 });   // passive sniffer
//
// Flag semantics map directly to the TWAI driver's node config flags.
// Mutually exclusive combinations (e.g. loopback + listen_only) are rejected
// by can_driver_init() with ESP_ERR_INVALID_ARG.
// -----------------------------------------------------------------------------
typedef struct {
    // Route transmitted frames back through the RX path so the controller
    // receives its own messages. Automatically forces self_test = 1 because
    // loopback without self-test causes TX failures if no other node is present
    // to send the ACK.
    uint32_t loopback    : 1;

    // Disable ACK checking on transmitted frames. Useful for single-node bench
    // testing without a second node present. Implied by loopback.
    uint32_t self_test   : 1;

    // Node never asserts dominant bits — no ACKs, no error frames. Pure passive
    // observer. Useful for a dedicated bus monitor / sniffer node.
    // Incompatible with loopback.
    uint32_t listen_only : 1;

    // Discard incoming remote frames (RTR) at the hardware filter level.
    // RTR frames are rarely used in modern CAN systems; enabling this keeps
    // the RX queue clean without any software overhead per-frame.
    uint32_t no_rtr      : 1;

    uint32_t reserved    : 28;
} CanInitFlags_t;

// -----------------------------------------------------------------------------
// RX event
//
// Copied by value from the ISR into the static queue — no pointers, no
// lifetime concerns. rx_tick is captured by the ISR at the moment the frame
// arrives, giving an accurate basis for staleness calculations in the app.
// -----------------------------------------------------------------------------
typedef struct {
    uint32_t   id;          // CAN arbitration ID
    uint8_t    data[8];     // Frame payload (up to 8 bytes for classic CAN)
    uint8_t    len;         // Actual payload length (DLC, 0–8)
    TickType_t rx_tick;     // FreeRTOS tick at the moment the ISR fired
} CanRxEvent_t;

// -----------------------------------------------------------------------------
// Lifecycle
// -----------------------------------------------------------------------------

/**
 * @brief Initialize the TWAI (CAN) peripheral and start the node.
 * * Must be called once at startup, before can_manager_init().
 *
 * @param tx_io  The GPIO pin number mapped to the CAN Transceiver TX.
 * @param rx_io  The GPIO pin number mapped to the CAN Transceiver RX.
 * @param baud   The CAN bus baud rate in bits per second (e.g., 500000 for 500 kbps).
 * @param flags  Initialization flags (loopback, listen_only, etc.).
 *
 * @return
 * - ESP_OK:                Initialized and running.
 * - ESP_ERR_INVALID_STATE: Already initialized.
 * - ESP_ERR_INVALID_ARG:   Incompatible flag combination (e.g., loopback + 
 * listen_only) or invalid GPIO selection.
 */
esp_err_t can_driver_init(gpio_num_t tx_io, gpio_num_t rx_io, uint32_t baud, CanInitFlags_t flags);

// Disable and delete the TWAI node, releasing hardware resources.
// Call can_manager_deinit() before this to stop the manager task first.
esp_err_t can_driver_deinit(void);

// -----------------------------------------------------------------------------
// Transmit
//
// Claims a slot from the static TX pool, copies the payload, and submits the
// frame to the hardware TX queue. Non-blocking — if the hardware queue is full
// the frame is dropped and the pool slot is released immediately.
//
// Returns:
//   ESP_OK                — frame queued successfully.
//   ESP_ERR_INVALID_ARG   — len > 8 or data is NULL when len > 0.
//   ESP_ERR_NO_MEM        — TX pool exhausted; all slots are in-flight.
//   ESP_ERR_INVALID_STATE — driver not initialized.
//
// Thread-safe. Safe to call from task or ISR context.
// The implementation detects the calling context at runtime via
// xPortInIsrContext() and uses the matching critical-section primitive
// (portENTER_CRITICAL vs portENTER_CRITICAL_ISR) for the TX pool spinlock.
// -----------------------------------------------------------------------------
esp_err_t can_driver_transmit(uint32_t id, const uint8_t *data, uint8_t len);

// -----------------------------------------------------------------------------
// Receive
//
// Block until an RX event is available or timeout_ticks elapses.
// Intended to be called only by the can_manager task.
//
// Returns:
//   ESP_OK                — received one event.
//   ESP_ERR_TIMEOUT       — queue was empty until timeout.
//   ESP_ERR_INVALID_ARG   — evt is NULL.
//   ESP_ERR_INVALID_STATE — driver not initialized/deinitialized.
// -----------------------------------------------------------------------------
esp_err_t can_driver_receive(CanRxEvent_t *evt, TickType_t timeout_ticks);

// -----------------------------------------------------------------------------
// Diagnostics
// -----------------------------------------------------------------------------

// Number of TX pool slots currently in-flight (0..CAN_TX_POOL_SIZE).
// For diagnostics and the self-test only — not for production logic.
int can_driver_get_pool_used(void);

// -----------------------------------------------------------------------------
// Bus-off recovery
//
// The driver sets a flag in on_state_change when the controller enters bus-off.
// The manager task polls it and calls can_driver_recover() from task context.
// twai_node_recover() must NOT be called from an ISR.
// -----------------------------------------------------------------------------
bool      can_driver_is_bus_off(void);
void      can_driver_clear_bus_off(void);
esp_err_t can_driver_recover(void);

// -----------------------------------------------------------------------------
// ISR diagnostic counters
//
// can_driver_get_isr_rx_calls() — number of times on_rx_done was invoked.
//   If this is 0 after transmitting in loopback mode, the filter is the issue.
//
// can_driver_get_isr_rx_fail()  — number of times twai_node_receive_from_isr
//   returned non-ESP_OK inside on_rx_done.
//   If this is > 0, the callback fires but the receive call itself is failing.

// can_driver_get_isr_rx_dropped() — number of RX events dropped because the
// static RX queue was full when posted from ISR.
//
// can_driver_reset_isr_counters() — reset all ISR counters to zero
// (rx callback calls, receive failures, queue-full drops).
// Also called automatically on can_driver_init().
// -----------------------------------------------------------------------------
uint32_t can_driver_get_isr_rx_calls(void);
uint32_t can_driver_get_isr_rx_fail(void);
uint32_t can_driver_get_isr_rx_dropped(void);
void     can_driver_reset_isr_counters(void);

// -----------------------------------------------------------------------------
// RX filter configuration
//
// These APIs configure the TWAI mask filter. The driver performs a
// disable->reconfigure->enable cycle internally as required by ESP-IDF.
//
// Single filter: one ID/mask pair.
// Dual filter:   two ID/mask pairs packed by TWAI dual-filter mode.
// Auto variants compute mask filter(s) from an allowlist of IDs.
// -----------------------------------------------------------------------------
esp_err_t can_driver_apply_single_filter(uint32_t id, uint32_t mask, bool is_ext);
esp_err_t can_driver_apply_dual_filter(uint32_t id1, uint32_t mask1,
                                       uint32_t id2, uint32_t mask2,
                                       bool is_ext);
esp_err_t can_driver_apply_single_filter_auto(const uint32_t *ids,
                                              size_t count,
                                              bool is_ext);
esp_err_t can_driver_apply_dual_filter_auto(const uint32_t *ids,
                                            size_t count,
                                            bool is_ext);