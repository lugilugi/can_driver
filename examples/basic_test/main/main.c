/**
 * @file can_driver_examples.c
 * @brief Comprehensive usage examples for every feature of can_driver.
 *
 * Each example is a self-contained app_main or task demonstrating one feature.
 * Copy the relevant section into your project's main.c.
 *
 * Index:
 *   1. Basic Initialization
 *   2. Publishing Messages (CAN_PUBLISH / can_publish)
 *   3. Reading State (CAN_GET_STATE)
 *   4. Staleness Detection (can_is_stale)
 *   5. Hardware Filtering (filter_ids in init)
 *   6. RX Hook — SD Logger
 *   7. Graceful Shutdown (can_driver_deinit)
 *   8. Multi-task safe reading
 *   9. Full System Integration Example
 */

#include "can_driver.h"
#include "can_payloads.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/gpio.h"

static const char* TAG = "EXAMPLE";

// GPIO pins — adjust to match your hardware
#define CAN_TX_PIN  GPIO_NUM_4
#define CAN_RX_PIN  GPIO_NUM_5
#define CAN_BAUD    500000


// ===================================================================================
// EXAMPLE 1: Basic Initialization
// ===================================================================================
// The simplest possible init — no hardware filter, accept all CAN IDs on the bus.
// Use this when prototyping or when your node needs to see all traffic.

void example_1_basic_init(void) {
    esp_err_t err = can_driver_init(CAN_TX_PIN, CAN_RX_PIN, CAN_BAUD, NULL, 0);

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN init failed: %s", esp_err_to_name(err));
        return;
    }

    ESP_LOGI(TAG, "CAN driver started successfully.");
}


// ===================================================================================
// EXAMPLE 2: Publishing Messages
// ===================================================================================
// Demonstrates how to build a payload struct and push it onto the bus.
// can_publish is non-blocking — it queues the frame and returns immediately.

void example_2_publish(void) {
    can_driver_init(CAN_TX_PIN, CAN_RX_PIN, CAN_BAUD, NULL, 0);

    // --- Pedal: 55% throttle, brake not pressed ---
    PedalPayload pedal = {0};
    PedalPayload_set(&pedal, 55.0f, false);
    can_publish(CAN_ID_PEDAL, &pedal, sizeof(pedal));

    // --- Aux Control: headlights and left turn signal ---
    AuxControlPayload aux = {0};

    // Directly set the flags in the struct
    aux.headlights = true;
    aux.left_turn  = true;

    // Publish using the hardened driver
    // This will perform a deep copy into the TX queue to prevent dangling pointers
    can_publish(CAN_ID_AUX_CTRL, &aux, sizeof(aux));

    // --- Power: raw ADC values from INA780 ---
    PowerPayload pwr = {0};
    PowerPayload_setRaw(&pwr, 12000, 3500); // ~37.5V, ~8.4A
    can_publish(CAN_ID_PWR_MONITOR_780, &pwr, sizeof(pwr));

    // --- Energy: 40-bit accumulator value ---
    EnergyPayload energy = {0};
    EnergyPayload_setRaw(&energy, 0x786);
    can_publish(CAN_ID_PWR_ENERGY, &energy, sizeof(energy));

    ESP_LOGI(TAG, "All messages published.");
}


// ===================================================================================
// EXAMPLE 3: Reading State from the Whiteboard
// ===================================================================================
// CAN_GET_STATE is the primary way to read received data. It is thread-safe —
// it takes a critical section internally so it can be called from any task.

void example_3_read_state_task(void* arg) {
    while (1) {
        // Pull the latest pedal data
        PedalPayload pedal;
        CAN_GET_STATE(pedal, &pedal);

        float throttle = PedalPayload_getThrottle(&pedal);
        bool  brake    = PedalPayload_isBrakePressed(&pedal);
        ESP_LOGI(TAG, "Throttle: %.1f%%  Brake: %s", throttle, brake ? "ON" : "OFF");

        // Pull the latest power data from traction system
        PowerPayload pwr;
        CAN_GET_STATE(pwr_780, &pwr);

        float voltage = PowerPayload_getVoltage(&pwr);
        float current = PowerPayload_getCurrent_780(&pwr);
        ESP_LOGI(TAG, "Traction — Voltage: %.2fV  Current: %.2fA", voltage, current);

        // Pull auxiliary control state and check specific flags
        AuxControlPayload aux_data;
        CAN_GET_STATE(aux, &aux_data);

        if (aux_data.hazards) {
            ESP_LOGW(TAG, "Hazard lights are active!");
        }

        vTaskDelay(pdMS_TO_TICKS(100));
    }
}


// ===================================================================================
// EXAMPLE 4: Staleness Detection
// ===================================================================================
// can_is_stale checks whether a node has gone silent. In a vehicle this is a
// critical safety check — a missing pedal message is not the same as 0% throttle.

void example_4_staleness_task(void* arg) {
    while (1) {
        // Pedal node must send at least every 100ms
        if (can_is_stale(HB_PEDAL, 100)) {
            ESP_LOGE(TAG, "SAFETY: Pedal node is silent! Cutting power.");
            // Engage safe state — set your motor controller to 0 here
        }

        // Power monitor is less critical, allow 500ms
        if (can_is_stale(HB_PWR_780, 500)) {
            ESP_LOGW(TAG, "WARNING: Traction power monitor offline.");
        }

        if (can_is_stale(HB_PWR_740, 500)) {
            ESP_LOGW(TAG, "WARNING: Aux power monitor offline.");
        }

        vTaskDelay(pdMS_TO_TICKS(50));
    }
}


// ===================================================================================
// EXAMPLE 5: Hardware Filter Configuration
// ===================================================================================
// Passing a list of IDs to can_driver_init programs the TWAI hardware filter.
// The filter uses bitwise common-ones/zeros to compute a mask, so only frames
// whose IDs share the right bit pattern will pass. Frames that don't match are
// rejected in hardware before they even reach the ISR.
//
// IMPORTANT: The mask is a best-fit — it may pass some IDs outside your list
// if their bit patterns overlap. Your routing table acts as the second filter.
//
// Example: To receive only pedal and power messages, pass those IDs:

void example_5_filtered_init(void) {
    static const uint32_t ids_we_care_about[] = {
        CAN_ID_PEDAL,
        CAN_ID_PWR_MONITOR_780,
        CAN_ID_PWR_MONITOR_740,
        CAN_ID_PWR_ENERGY,
    };
    size_t count = sizeof(ids_we_care_about) / sizeof(ids_we_care_about[0]);

    esp_err_t err = can_driver_init(CAN_TX_PIN, CAN_RX_PIN, CAN_BAUD,
                                    ids_we_care_about, count);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Filtered init failed: %s", esp_err_to_name(err));
    }
}


// ===================================================================================
// EXAMPLE 6: RX Hook — SD Logger
// ===================================================================================
// can_set_rx_hook registers a callback that fires for every received frame
// BEFORE it is routed to the whiteboard.
//
// !! WARNING: This callback runs in ISR context !!
// You MUST NOT call blocking functions (printf, vTaskDelay, SD writes, etc.).
// The correct pattern is to copy the frame into a FreeRTOS queue and process
// it in a normal task.

static QueueHandle_t log_queue = NULL;

// ISR-safe hook: just copies the frame into a queue
static void sd_logger_hook(const twai_frame_t* frame) {
    // We need our own buffer since frame->buffer is on the ISR stack
    typedef struct { uint32_t id; uint8_t len; uint8_t data[8]; } LogItem_t;

    LogItem_t item = {
        .id  = frame->header.id,
        .len = frame->buffer_len,
    };
    memcpy(item.data, frame->buffer, frame->buffer_len);

    // Non-blocking send — drop if queue is full rather than blocking the ISR
    BaseType_t woken = pdFALSE;
    xQueueSendFromISR(log_queue, &item, &woken);
    portYIELD_FROM_ISR(woken);
}

// Normal task: drains the queue and writes to SD
static void sd_logger_task(void* arg) {
    typedef struct { uint32_t id; uint8_t len; uint8_t data[8]; } LogItem_t;
    LogItem_t item;

    while (1) {
        if (xQueueReceive(log_queue, &item, portMAX_DELAY)) {
            // Write to SD card here — safe because we're in task context
            ESP_LOGI(TAG, "LOG ID=0x%03lX len=%d", item.id, item.len);
        }
    }
}

void example_6_rx_hook(void) {
    typedef struct { uint32_t id; uint8_t len; uint8_t data[8]; } LogItem_t;
    log_queue = xQueueCreate(64, sizeof(LogItem_t));

    can_driver_init(CAN_TX_PIN, CAN_RX_PIN, CAN_BAUD, NULL, 0);
    can_set_rx_hook(sd_logger_hook);

    xTaskCreate(sd_logger_task, "SD_Log", 4096, NULL, 3, NULL);
}


// ===================================================================================
// EXAMPLE 7: Graceful Shutdown
// ===================================================================================
// can_driver_deinit stops the node, deletes all tasks, and frees all memory.
// After this call it is safe to call can_driver_init again (e.g. to change baud rate).
// is_running is reset so re-initialization works cleanly.

void example_7_deinit(void) {
    can_driver_init(CAN_TX_PIN, CAN_RX_PIN, CAN_BAUD, NULL, 0);

    ESP_LOGI(TAG, "Running normally...");
    vTaskDelay(pdMS_TO_TICKS(5000));

    ESP_LOGI(TAG, "Shutting down CAN driver.");
    esp_err_t err = can_driver_deinit();

    if (err == ESP_OK) {
        ESP_LOGI(TAG, "Driver stopped cleanly.");
    }

    // Safe to reinitialize with different settings
    can_driver_init(CAN_TX_PIN, CAN_RX_PIN, 250000, NULL, 0); // restart at 250kbps
}


// ===================================================================================
// EXAMPLE 8: Multi-Task Safe Reading
// ===================================================================================
// Multiple tasks can safely call CAN_GET_STATE simultaneously.
// The critical section inside can_get_state_internal serializes concurrent reads.
// There is NO need for an application-level mutex.

static void motor_controller_task(void* arg) {
    while (1) {
        PedalPayload pedal;
        CAN_GET_STATE(pedal, &pedal);

        if (can_is_stale(HB_PEDAL, 100)) {
            // Node is offline — safe state
            ESP_LOGE(TAG, "[Motor] Pedal timeout, zeroing output");
        } else {
            float throttle = PedalPayload_getThrottle(&pedal);
            ESP_LOGI(TAG, "[Motor] Setting throttle to %.1f%%", throttle);
        }
        vTaskDelay(pdMS_TO_TICKS(20)); // 50Hz control loop
    }
}

static void dashboard_task(void* arg) {
    while (1) {
        PowerPayload pwr_780, pwr_740;
        CAN_GET_STATE(pwr_780, &pwr_780);
        CAN_GET_STATE(pwr_740, &pwr_740);

        float v    = PowerPayload_getVoltage(&pwr_780);
        float i_hi = PowerPayload_getCurrent_780(&pwr_780);
        float i_lo = PowerPayload_getCurrent_740(&pwr_740);

        ESP_LOGI(TAG, "[Dash] Bus: %.2fV  Traction: %.2fA  Aux: %.2fA", v, i_hi, i_lo);
        vTaskDelay(pdMS_TO_TICKS(500)); // 2Hz display refresh
    }
}

void example_8_multi_task(void) {
    can_driver_init(CAN_TX_PIN, CAN_RX_PIN, CAN_BAUD, NULL, 0);

    xTaskCreate(motor_controller_task, "Motor",  2048, NULL, 8, NULL);
    xTaskCreate(dashboard_task,        "Dash",   2048, NULL, 3, NULL);
}


// ===================================================================================
// EXAMPLE 9: Full System Integration
// ===================================================================================
// Ties everything together. One node that:
//   - Accepts only the IDs it needs (hardware filter)
//   - Publishes its own sensor data
//   - Reads remote node data from the whiteboard at different rates
//   - Monitors node health via staleness checks
//   - Logs all raw frames to an SD queue via the hook

void example_9_full_system(void) {
    // 1. Hardware Filters: Define IDs we need for the Whiteboard
    static const uint32_t filter_ids[] = {
        CAN_ID_PEDAL,
        CAN_ID_PWR_MONITOR_780,
        CAN_ID_PWR_MONITOR_740,
    };

    // 2. Initialize the Driver (v5.5.3 New API)
    // This handles handle creation, task spawning, and 1=Match filter application
    esp_err_t err = can_driver_init(
        CAN_TX_PIN, CAN_RX_PIN, CAN_BAUD,
        filter_ids, sizeof(filter_ids) / sizeof(filter_ids[0])
    );

    if (err != ESP_OK) {
        ESP_LOGE(TAG, "CAN Init failed: %s", esp_err_to_name(err));
        return;
    }

    // 3. SD Logger: Use the "Hook" to capture EVERYTHING (even filtered IDs)
    // Note: The hook runs in ISR context, so we just push to a queue.
    typedef struct { uint32_t id; uint8_t len; uint8_t data[8]; } LogItem_t;
    log_queue = xQueueCreate(64, sizeof(LogItem_t));
    
    can_set_rx_hook(sd_logger_hook); 
    xTaskCreate(sd_logger_task, "SD_Log", 4096, NULL, 3, NULL);

    // 4. Start Consumer Tasks: They independently "Pull" from the Whiteboard
    xTaskCreate(motor_controller_task, "Motor", 2048, NULL, 8, NULL);
    xTaskCreate(dashboard_task,        "Dash",  2048, NULL, 3, NULL);

    // 5. Main Publish Loop: Send Aux Control State (10Hz)
    AuxControlPayload aux_state = {0}; 
    uint32_t cycle = 0;

    while (1) {
        // UPDATED: Logic for new AuxControlPayload
        // Directly toggle the wipers member in the struct
        if (cycle++ % 50 == 0) {
            aux_state.wipers = !aux_state.wipers; // Direct boolean toggle
            ESP_LOGI(TAG, "Wipers toggled: %s", aux_state.wipers ? "ON" : "OFF");
        }

        // Feature: can_publish() enforces Classic CAN (max 8 bytes, no FD)
        esp_err_t pub = can_publish(CAN_ID_AUX_CTRL, &aux_state, sizeof(aux_state));
        
        if (pub != ESP_OK) {
            // Usually indicates the 20-slot TX queue is full (bus congestion)
            ESP_LOGW(TAG, "TX queue full, dropped aux frame");
        }

        vTaskDelay(pdMS_TO_TICKS(100)); // 10Hz
    }
}


// ===================================================================================
// Entry Point
// ===================================================================================
// Uncomment the example you want to run.

void app_main(void) {
    // example_1_basic_init();
    // example_2_publish();
    // example_5_filtered_init();
    // example_6_rx_hook();
    // example_7_deinit();
    // example_8_multi_task();
    example_9_full_system();
}
