#pragma once

#include "driver/twai.h"
#include "driver/gpio.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>

// ===================================================================================
// INITIALIZATION API
// ===================================================================================

/**
 * @brief Initializes the ESP32 TWAI controller and starts background routing tasks.
 * @param tx_pin GPIO pin connected to the CAN Transceiver TX
 * @param rx_pin GPIO pin connected to the CAN Transceiver RX
 * @param baud_rate Usually 500000 (500kbps) or 1000000 (1Mbps)
 * @return ESP_OK on success
 */
esp_err_t can_driver_init(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t baud_rate);

// ===================================================================================
// APPLICATION ABSTRACTION MACROS
// ===================================================================================
// Use these macros in your application code. They hide the complexity of FreeRTOS 
// queues and raw memory copies.

/**
 * @brief Thread-safe, non-blocking publish. Instantly pushes data to a background 
 * TX queue so your sensor tasks never freeze waiting for the bus.
 * @param id The CAN Msg ID (e.g., CAN_ID_PEDAL)
 * @param payload_ptr Pointer to the struct containing the payload
 * @usage CAN_PUBLISH(CAN_ID_PEDAL, &my_pedal_data);
 */
#define CAN_PUBLISH(id, payload_ptr) \
    can_send_message((id), (const uint8_t*)(payload_ptr), sizeof(*(payload_ptr)))

/**
 * @brief Creates a receiving queue and registers it with the driver. If multiple 
 * tasks subscribe to the same ID, the driver automatically copies the message
 * to all of them (Fan-Out routing).
 * @param id The CAN Msg ID to listen for
 * @param queue_ptr Pointer to your QueueHandle_t variable
 * @param queue_len Maximum number of unread messages to hold before dropping new ones
 * @usage QueueHandle_t my_q; CAN_SUBSCRIBE_TOPIC(CAN_ID_PEDAL, &my_q, 5);
 */
#define CAN_SUBSCRIBE_TOPIC(id, que ue_ptr, queue_len) \
    do { \
        *(queue_ptr) = xQueueCreate((queue_len), sizeof(twai_message_t)); \
        configASSERT(*(queue_ptr) != NULL); \
        can_subscribe((id), *(queue_ptr)); \
    } while(0)
/**
 * @brief Waits for a message on your queue and safely unpacks it into your struct.
 * If a message doesn't arrive within the timeout, it safely returns false.
 * @param queue_handle The queue you created with CAN_SUBSCRIBE_TOPIC
 * @param payload_ptr Pointer to the struct where data should be saved
 * @param timeout_ms Max time to wait (e.g., 100ms). Prevents stale data usage!
 * @return true if data arrived, false if it timed out.
 * @usage if (CAN_RECEIVE(my_q, &my_pedal_data, 100)) { ... }
 */
#define CAN_RECEIVE(queue_handle, payload_ptr, timeout_ms) \
    can_receive_payload((queue_handle), (void*)(payload_ptr), sizeof(*(payload_ptr)), (timeout_ms))


// ===================================================================================
// LOW-LEVEL INTERNAL FUNCTIONS
// ===================================================================================
// Do not call these directly in application code. They are exposed here so the 
// macros above can use them.
esp_err_t can_send_message(uint32_t id, const uint8_t* data, uint8_t len);
esp_err_t can_subscribe(uint32_t id, QueueHandle_t rx_queue);

static inline bool can_receive_payload(QueueHandle_t q, void* payload_ptr, size_t payload_size, uint32_t timeout_ms) {
    twai_message_t rx_msg;
    if (xQueueReceive(q, &rx_msg, pdMS_TO_TICKS(timeout_ms)) == pdTRUE) {
        if (rx_msg.data_length_code != payload_size) return false;
        memcpy(payload_ptr, rx_msg.data, payload_size);
        return true;
    }
    return false;
}