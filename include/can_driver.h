#pragma once

#include "esp_err.h"
#include "esp_twai.h"
#include "hal/twai_types.h"
#include "hal/gpio_types.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>
#include <stddef.h>

#ifdef __cplusplus
extern "C" {
#endif

// =============================================================================
// can_driver.h — Simplified TWAI (CAN) wrapper, built on the ESP-IDF 5.5
// esp_driver_twai node API.
//
// Key properties:
//   - Hardware acceptance filter that accepts a LIST of IDs (the driver
//     computes the smallest maskable region containing them). Frames that do
//     not match never wake the CPU, which keeps idle power low. Optionally
//     .software_filter discards over-accepted frames so only listed IDs are
//     delivered.
//   - Callback-driven receive (no polling): a matching frame wakes the CPU,
//     is queued by the driver, and is read by can_driver_receive().
//   - Automatic bus-off recovery deferred to the FreeRTOS timer-service task.
//   - Designed to pair with auto-generated DBC parsers (e.g., cantools).
//
// This component supports classic CAN frames only. CAN-FD frames are excluded
// by the hardware filter and rejected by the transmit/receive validation.
// =============================================================================

typedef struct {
    uint32_t loopback    : 1;
    uint32_t listen_only : 1;
} CanInitFlags_t;

/**
 * @brief Acceptance filter configuration.
 *
 * The driver computes a maskable region (code/mask) that covers all listed
 * IDs. If the IDs do not form an exact maskable pattern, the driver uses the
 * smallest region containing them and logs a warning with the number of
 * additional IDs that will also pass (hardware limitation: one filter on most
 * classic chips).
 *
 * Set `software_filter` to discard over-accepted frames in software: only the
 * listed IDs are then ever delivered to can_driver_receive(). Note this does
 * not save power — the CPU still wakes for every frame the hardware region
 * accepts (filter by class, see README, to keep the region exact).
 *
 * A single filter can only target one ID format: standard (11-bit) or
 * extended (29-bit). With accept-all + software_filter, the hardware accepts
 * both formats but the software whitelist still matches only `extd`.
 * `accept_all` selects an open hardware filter; when `software_filter` is also
 * set and `id_count` is nonzero, the ID list becomes the exact delivery
 * whitelist. All nonzero ID lists must be valid, unique, and have a non-null
 * `ids` pointer, even when hardware accept-all is selected.
 */
typedef struct {
    uint32_t accept_all     : 1;   /**< Open hardware filter; with software_filter the list remains active */
    uint32_t extd           : 1;   /**< The IDs are extended (29-bit) IDs */
    uint32_t software_filter : 1;  /**< Discard non-listed IDs in software (exact delivery) */
    uint32_t id_count;             /**< Number of IDs (0 = accept all) */
    const uint32_t *ids;           /**< IDs to accept (unique, sorted not required) */
} CanFilterConfig_t;

/** Accept-all filter (also the behaviour when can_driver_init gets NULL). */
#define CAN_FILTER_ACCEPT_ALL() ((CanFilterConfig_t){ .accept_all = 1 })

/**
 * @brief Initialize the TWAI (CAN) peripheral and start the node.
 *
 * @param tx_io  The GPIO pin number mapped to the CAN Transceiver TX.
 * @param rx_io  The GPIO pin number mapped to the CAN Transceiver RX.
 * @param baud   The CAN bus baud rate in bps (e.g., 500000 for 500 kbps).
 *               The driver computes bit timing for the requested rate.
 * @param flags  Init flags (loopback, listen_only).
 * @param filter Acceptance filter, or NULL to accept all frames.
 *
 * @return ESP_OK on success.
 */
esp_err_t can_driver_init(gpio_num_t tx_io, gpio_num_t rx_io, uint32_t baud, CanInitFlags_t flags, const CanFilterConfig_t *filter);

/**
 * @brief Stop and free the TWAI node.
 *
 * The caller must stop/join every task using this component before calling
 * this function. In particular, do not call while another task is blocked in
 * can_driver_receive() or can_driver_transmit(). The driver rejects deinit
 * while any operation is active and never deletes queues underneath waiters.
 * Init and deinit must be externally serialized with all other API calls.
 * Also call this before entering light sleep: while enabled, the node holds a
 * PM lock (prevents low CPU frequencies / light sleep on some targets).
 */
esp_err_t can_driver_deinit(void);

/**
 * @brief Transmit a TWAI frame.
 *
 * The driver copies the frame into its own TX slots, so the caller's frame
 * and buffer may be stack-allocated and do not need to outlive the call.
 * This API accepts classic CAN only: FDF/BRS frames and payloads longer than
 * eight bytes are rejected. Remote frames may specify a DLC but have no data
 * payload.
 *
 * @param frame Frame to transmit (header.id, header.ide, buffer, buffer_len).
 *              buffer_len must be <= 8 (classic CAN). If header.dlc is 0 it
 *              is derived from buffer_len.
 * @param timeout_ticks Ticks to block if no driver-owned TX slot is free.
 * @return ESP_OK if queued, ESP_ERR_TIMEOUT if no slot became available,
 *         ESP_ERR_NOT_SUPPORTED for CAN-FD/BRS frames, or a validation error.
 */
esp_err_t can_driver_transmit(const twai_frame_t *frame, TickType_t timeout_ticks);

/**
 * @brief Receive a TWAI frame.
 *
 * Blocks until a matching frame arrives or the timeout expires. Provide
 * `frame->buffer` and its capacity explicitly. On success the driver reports
 * the actual payload length in `frame->buffer_len`; the input value of
 * `frame->buffer_len` is ignored, so a frame object can safely be reused.
 * Remote frames report a zero payload length. A received frame is consumed
 * and `ESP_ERR_INVALID_SIZE` is returned when the supplied capacity is too
 * small; no partial payload is copied in that case.
 *
 * @param frame Frame to populate.
 * @param buffer_capacity Capacity of frame->buffer in bytes.
 * @param timeout_ticks Ticks to block waiting for a frame.
 * @return ESP_OK on success, ESP_ERR_TIMEOUT on timeout, or
 *         ESP_ERR_INVALID_SIZE when the destination is too small.
 */
esp_err_t can_driver_receive(twai_frame_t *frame,
                             size_t buffer_capacity,
                             TickType_t timeout_ticks);

/**
 * @brief TWAI node status/diagnostics snapshot.
 */
typedef struct {
    twai_error_state_t error_state;      /**< Active / Warning / Passive / Bus-off */
    uint16_t tx_error_count;             /**< Transmit error counter */
    uint16_t rx_error_count;             /**< Receive error counter */
    uint32_t tx_slots_remaining;         /**< Free driver-owned TX slots */
    uint32_t twai_tx_queue_remaining;   /**< Free ESP-IDF native TX slots */
    uint32_t rx_queue_remaining;         /**< Free driver RX queue slots */
    uint32_t bus_error_count;            /**< Bus errors since enable */
    uint32_t rx_dropped_count;           /**< Frames lost because the RX queue was full */
    uint32_t software_dropped_count;     /**< Frames discarded by the software filter */
    uint32_t malformed_frame_count;      /**< Rejected FD/invalid-DLC RX frames */
    uint32_t recovery_failure_count;     /**< Deferred bus-off recovery failures */
} CanStatus_t;

/**
 * @brief Get TWAI node status and diagnostics.
 */
esp_err_t can_driver_get_status(CanStatus_t *status);

#ifdef __cplusplus
}
#endif
