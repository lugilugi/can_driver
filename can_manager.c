#include "can_manager.h"
#include "can_driver.h"
#include "can_state.h"
#include "can_payloads.h"
#include "can_config.h"
#include "can_logger.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "can_manager";

// Spinlock for the 64-bit energy struct writes.
// See can_state.h thread-safety note for context.
static portMUX_TYPE s_energy_mux = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// Static task allocation — no heap.
// =============================================================================
static StaticTask_t s_task_buf;
static StackType_t  s_task_stack[CAN_MANAGER_TASK_STACK_SIZE];
static TaskHandle_t s_task_hdl   = NULL;
static volatile bool s_running   = false;

// =============================================================================
// Frame dispatch
//
// In this "Raw-First" model, the manager task performs zero data conversion.
// It validates the length, copies the raw bytes into the global state, 
// and updates the timestamp. All decoding math (scaling/offsets) is deferred 
// to the application tasks that actually need the values.
// =============================================================================
static void dispatch(const CanRxEvent_t *evt)
{
    switch ((CanMsgID_t)evt->id) {

        case CAN_ID_PEDAL:
            if (evt->len >= sizeof(PedalPayload)) {
                // Copy raw 1-byte throttle/brake state
                memcpy(&g_can_pedal.data, evt->data, sizeof(PedalPayload));
                g_can_pedal.last_rx_tick = evt->rx_tick;
            } else {
                ESP_LOGW(TAG, "PEDAL frame too short (%d bytes)", evt->len);
            }
            break;

        case CAN_ID_AUX_CTRL:
            if (evt->len >= sizeof(AuxControlPayload)) {
                // Copy raw 1-byte bitfield union
                memcpy(&g_can_aux.data, evt->data, sizeof(AuxControlPayload));
                g_can_aux.last_rx_tick = evt->rx_tick;
            } else {
                ESP_LOGW(TAG, "AUX_CTRL frame too short (%d bytes)", evt->len);
            }
            break;

        case CAN_ID_PWR_MONITOR_780:
            if (evt->len >= sizeof(PowerPayload)) {
                // Copy raw 4-byte ADC data for Traction battery
                memcpy(&g_can_pwr780.data, evt->data, sizeof(PowerPayload));
                g_can_pwr780.last_rx_tick = evt->rx_tick;
            } else {
                ESP_LOGW(TAG, "PWR_780 frame too short (%d bytes)", evt->len);
            }
            break;

        case CAN_ID_PWR_MONITOR_740:
            if (evt->len >= sizeof(PowerPayload)) {
                // Copy raw 4-byte ADC data for Aux battery
                memcpy(&g_can_pwr740.data, evt->data, sizeof(PowerPayload));
                g_can_pwr740.last_rx_tick = evt->rx_tick;
            } else {
                ESP_LOGW(TAG, "PWR_740 frame too short (%d bytes)", evt->len);
            }
            break;

        case CAN_ID_PWR_ENERGY:
            if (evt->len >= sizeof(EnergyPayload)) {
                // Critical section: protects the 5-byte memcpy from torn 
                // reads. Even though it's not a float write anymore, a 
                // 5-byte copy is not atomic on RISC-V.
                portENTER_CRITICAL(&s_energy_mux);
                memcpy(&g_can_energy.data, evt->data, sizeof(EnergyPayload));
                g_can_energy.last_rx_tick = evt->rx_tick;
                portEXIT_CRITICAL(&s_energy_mux);
            } else {
                ESP_LOGW(TAG, "PWR_ENERGY frame too short (%d bytes)", evt->len);
            }
            break;

        case CAN_ID_DASH_STAT:
            // Store raw bytes if the node echoes back its own dash state.
            if (evt->len <= 8) {
                memcpy(g_can_dash.data, evt->data, evt->len);
                g_can_dash.len = evt->len;
                g_can_dash.last_rx_tick = evt->rx_tick;
            }
            break;

        default:
            ESP_LOGD(TAG, "Unknown ID: 0x%03lX (len=%d)", (unsigned long)evt->id, evt->len);
            break;
    }
}

// =============================================================================
// Manager task
// =============================================================================
static void can_manager_task(void *arg)
{
    ESP_LOGI(TAG, "Manager task started");

    CanRxEvent_t evt;

    while (s_running) {

        // --- Bus-off recovery ---
        // Checked on every iteration. twai_node_recover() must be called from
        // task context (not ISR), so the driver only sets a flag in the callback
        // and defers the actual recovery call here.
        if (can_driver_is_bus_off()) {
            ESP_LOGW(TAG, "Bus-off detected — initiating recovery");
            esp_err_t ret = can_driver_recover();
            if (ret == ESP_OK) {
                can_driver_clear_bus_off();
                ESP_LOGI(TAG, "Recovery initiated (reconnects after 129 recessive bits)");
            } else {
                ESP_LOGE(TAG, "Recovery call failed: %s", esp_err_to_name(ret));
                // Leave the flag set and retry on the next iteration.
            }
        }

        // --- RX dispatch ---
        // Block for up to 10 ms waiting for a frame. The short timeout keeps
        // the bus-off check responsive without burning CPU in a tight loop.
        if (can_driver_receive(&evt, pdMS_TO_TICKS(10)) == ESP_OK) {
            dispatch(&evt);
            can_logger_post(&evt);
        }
    }

    ESP_LOGI(TAG, "Manager task stopping");
    s_task_hdl = NULL;
    vTaskDelete(NULL);
}

// =============================================================================
// Public API
// =============================================================================

esp_err_t can_manager_init(void)
{
    if (s_task_hdl != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_running = true;

    s_task_hdl = xTaskCreateStaticPinnedToCore(
        can_manager_task,
        "can_manager",
        CAN_MANAGER_TASK_STACK_SIZE,
        NULL,
        CAN_MANAGER_TASK_PRIORITY,
        s_task_stack,
        &s_task_buf,
        CAN_MANAGER_TASK_CORE
    );

    if (s_task_hdl == NULL) {
        s_running = false;
        ESP_LOGE(TAG, "Failed to create manager task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Manager initialized (priority=%d, stack=%d bytes)",
             CAN_MANAGER_TASK_PRIORITY, CAN_MANAGER_TASK_STACK_SIZE);
    return ESP_OK;
}

esp_err_t can_manager_deinit(void)
{
    if (s_task_hdl == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_running = false;

    // Give the task one full iteration to exit cleanly (10 ms timeout + margin).
    vTaskDelay(pdMS_TO_TICKS(20));

    // Force-delete if it hasn't exited yet (e.g. blocked on a long receive).
    if (s_task_hdl != NULL) {
        vTaskDelete(s_task_hdl);
        s_task_hdl = NULL;
    }

    ESP_LOGI(TAG, "Manager deinitialized");
    return ESP_OK;
}