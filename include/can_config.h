#pragma once

// =============================================================================
// can_config.h — All compile-time constants for the CAN component.
// Tune these to match your hardware and application requirements.
// =============================================================================

// -----------------------------------------------------------------------------
// Hardware
// -----------------------------------------------------------------------------
#define CAN_TX_GPIO             0       // TWAI TX GPIO pin
#define CAN_RX_GPIO             1       // TWAI RX GPIO pin
#define CAN_BAUD_RATE           500000  // 500 kbps — standard automotive rate

// -----------------------------------------------------------------------------
// TX Pool
// Number of static frame slots available for in-flight transmissions.
// A slot is held from twai_driver_transmit() until the on_tx_done ISR fires.
// If all slots are claimed, transmit returns ESP_ERR_NO_MEM immediately.
// Must be >= CAN_TX_QUEUE_DEPTH to avoid unnecessary pool exhaustion.
// -----------------------------------------------------------------------------
#define CAN_TX_POOL_SIZE        8

// Hardware TX queue depth passed to the TWAI driver.
#define CAN_TX_QUEUE_DEPTH      8

// Retry count on TX failure.
// -1 = infinite retries until success or bus-off.
//  0 = single-shot, no retry.
//  N = retry N times.
#define CAN_TX_RETRY_COUNT      3

// -----------------------------------------------------------------------------
// RX Queue
// Depth of the static ISR -> manager task queue.
// Each slot holds one CanRxEvent_t (id + 8 bytes data + len = ~14 bytes).
// Increase if the manager task can be starved by a burst of high-rate frames.
// -----------------------------------------------------------------------------
#define CAN_RX_QUEUE_DEPTH      16

// -----------------------------------------------------------------------------
// Manager Task
// The task that drains the RX queue and updates the global state structs.
// -----------------------------------------------------------------------------
#define CAN_MANAGER_TASK_PRIORITY   5
#define CAN_MANAGER_TASK_STACK_SIZE 2048

// tskNO_AFFINITY is correct for ESP32-C3 (single core) and multi-core alike.
#define CAN_MANAGER_TASK_CORE       tskNO_AFFINITY
