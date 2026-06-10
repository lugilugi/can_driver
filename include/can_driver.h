#pragma once

#include "esp_err.h"
#include "driver/twai.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <stdint.h>
#include <stdbool.h>

// =============================================================================
// can_driver.h — Simplified TWAI (CAN) wrapper.
// Designed to pair with auto-generated DBC parsers (e.g., cantools).
// =============================================================================

typedef struct {
    uint32_t loopback    : 1;
    uint32_t listen_only : 1;
} CanInitFlags_t;

/**
 * @brief Initialize the TWAI (CAN) peripheral and start the node.
 * 
 * @param tx_io  The GPIO pin number mapped to the CAN Transceiver TX.
 * @param rx_io  The GPIO pin number mapped to the CAN Transceiver RX.
 * @param baud   The CAN bus baud rate in bps (e.g., 500000 for 500 kbps).
 * @param flags  Init flags (loopback, listen_only).
 * 
 * @return ESP_OK on success.
 */
esp_err_t can_driver_init(gpio_num_t tx_io, gpio_num_t rx_io, uint32_t baud, CanInitFlags_t flags);

/**
 * @brief Stop and uninstall the TWAI driver.
 */
esp_err_t can_driver_deinit(void);

/**
 * @brief Transmit a CAN frame.
 * 
 * This is a thin wrapper over twai_transmit. It natively supports blocking
 * (via FreeRTOS ticks) if the hardware TX queue is full, which solves
 * the ESP_ERR_NO_MEM buffer issue.
 * 
 * @param id CAN arbitration ID.
 * @param data Payload buffer (up to 8 bytes).
 * @param len Payload length (0-8).
 * @param timeout_ticks Ticks to block if the TX queue is full.
 * @return ESP_OK if queued, ESP_ERR_TIMEOUT if queue remained full.
 */
esp_err_t can_driver_transmit(uint32_t id, const uint8_t *data, uint8_t len, TickType_t timeout_ticks);

/**
 * @brief Receive a CAN frame.
 * 
 * This is a thin wrapper over twai_receive.
 * 
 * @param msg Pointer to the TWAI message structure to populate.
 * @param timeout_ticks Ticks to block waiting for a message.
 * @return ESP_OK on success.
 */
esp_err_t can_driver_receive(twai_message_t *msg, TickType_t timeout_ticks);