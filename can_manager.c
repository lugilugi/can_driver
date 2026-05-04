#include "can_manager.h"
#include "can_driver.h"
#include "can_state.h"
#include "can_message_catalog.h"
#include "can_payloads.h"
#include "can_config.h"
#include "can_logger.h"
#include "can_usb_forward.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "can_manager";

// =============================================================================
// Static task allocation — no heap.
// =============================================================================
static StaticTask_t s_task_buf;
static StackType_t  s_task_stack[CAN_MANAGER_TASK_STACK_SIZE];
static TaskHandle_t s_task_hdl   = NULL;
static volatile bool s_running   = false;
static StaticSemaphore_t s_stop_sem_buf;
static SemaphoreHandle_t s_stop_sem = NULL;
static bool s_logger_owned = false;
static bool s_usb_forward_owned = false;

// Rate-limited diagnostics — avoids log flooding under bus noise.
static uint32_t   s_unknown_id_count  = 0;
static uint32_t   s_short_frame_count = 0;
static TickType_t s_last_diag_tick    = 0;
#define DIAG_SUMMARY_INTERVAL_MS  5000

typedef struct {
    const char *name;
    uint32_t id;
    uint8_t min_len;
    uint8_t copy_len;
    void *state_ptr;
    TickType_t *last_rx_tick_ptr;
} CanCopyRouteDesc_t;

#define MAKE_COPY_ROUTE_DESC(_name, _id, _payload_t, _state_field, _tick_field) \
    {                                                                            \
        .name = (_name),                                                         \
        .id = (_id),                                                             \
        .min_len = (uint8_t)sizeof(_payload_t),                                 \
        .copy_len = (uint8_t)sizeof(_payload_t),                                \
        .state_ptr = &(_state_field),                                            \
        .last_rx_tick_ptr = &(_tick_field),                                      \
    },

static const CanCopyRouteDesc_t s_copy_routes[] = {
    CAN_MESSAGE_COPY_ROUTE_TABLE(MAKE_COPY_ROUTE_DESC)
};

static const CanCopyRouteDesc_t *find_copy_route(uint32_t id)
{
    for (size_t i = 0; i < (sizeof(s_copy_routes) / sizeof(s_copy_routes[0])); i++) {
        if (s_copy_routes[i].id == id) {
            return &s_copy_routes[i];
        }
    }
    return NULL;
}

// Duplicate IDs in the catalog produce a compile-time "duplicate case value"
// error here, which prevents silent message-map drift.
static inline void validate_catalog_unique_ids(void)
{
    switch (0) {
#define UNIQUE_CASE_COPY(_name, _id, _payload_t, _state_field, _tick_field) case _id:
        CAN_MESSAGE_COPY_ROUTE_TABLE(UNIQUE_CASE_COPY)
#undef UNIQUE_CASE_COPY
#define UNIQUE_CASE_SPECIAL(_name, _id) case _id:
        CAN_MESSAGE_SPECIAL_ROUTE_TABLE(UNIQUE_CASE_SPECIAL)
#undef UNIQUE_CASE_SPECIAL
        break;
    default:
        break;
    }
}

static esp_err_t start_optional_services(void)
{
    esp_err_t ret;

    s_logger_owned = false;
    s_usb_forward_owned = false;

    ret = can_logger_init();
    if (ret == ESP_OK) {
        s_logger_owned = true;
    } else if (ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "Logger init failed: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = can_usb_forward_init();
    if (ret == ESP_OK) {
        s_usb_forward_owned = true;
    } else if (ret != ESP_ERR_INVALID_STATE) {
        ESP_LOGE(TAG, "USB forward init failed: %s", esp_err_to_name(ret));
        if (s_logger_owned) {
            can_logger_deinit();
            s_logger_owned = false;
        }
        return ret;
    }

    return ESP_OK;
}

static void stop_optional_services(void)
{
    if (s_usb_forward_owned) {
        esp_err_t ret = can_usb_forward_deinit();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "USB forward deinit failed: %s", esp_err_to_name(ret));
        }
        s_usb_forward_owned = false;
    }

    if (s_logger_owned) {
        esp_err_t ret = can_logger_deinit();
        if (ret != ESP_OK) {
            ESP_LOGW(TAG, "Logger deinit failed: %s", esp_err_to_name(ret));
        }
        s_logger_owned = false;
    }
}

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
    const CanCopyRouteDesc_t *route = find_copy_route(evt->id);
    if (route != NULL) {
        if (evt->len >= route->min_len) {
            memcpy(route->state_ptr, evt->data, route->copy_len);
            *route->last_rx_tick_ptr = evt->rx_tick;
        } else {
            s_short_frame_count++;
        }
        return;
    }

    switch ((CanMsgID_t)evt->id) {
        case CAN_ID_PWR_ENERGY:
            if (evt->len >= sizeof(EnergyPayload)) {
                // Centralized synchronized write in can_state.c.
                can_state_set_energy_raw((const EnergyPayload *)evt->data, evt->rx_tick);
            } else {
                s_short_frame_count++;
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
            s_unknown_id_count++;
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
        // Block for up to 100 ms waiting for a frame. Balances bus-off recovery
        // responsiveness (100 ms latency is negligible vs the hardware recovery
        // sequence) against idle CPU — 10x fewer wakeups than the old 10 ms poll.
        if (can_driver_receive(&evt, pdMS_TO_TICKS(100)) == ESP_OK) {
            dispatch(&evt);
            can_logger_post(&evt);
            can_usb_forward_post(&evt);
        }

        // --- Rate-limited diagnostic summary ---
        TickType_t now = xTaskGetTickCount();
        if ((now - s_last_diag_tick) >= pdMS_TO_TICKS(DIAG_SUMMARY_INTERVAL_MS)) {
            if (s_unknown_id_count > 0 || s_short_frame_count > 0) {
                ESP_LOGW(TAG, "Dispatch: %lu unknown IDs, %lu short frames (last %d ms)",
                         (unsigned long)s_unknown_id_count,
                         (unsigned long)s_short_frame_count,
                         DIAG_SUMMARY_INTERVAL_MS);
                s_unknown_id_count  = 0;
                s_short_frame_count = 0;
            }
            s_last_diag_tick = now;
        }
    }

    ESP_LOGI(TAG, "Manager task stopping");
    if (s_stop_sem != NULL) {
        xSemaphoreGive(s_stop_sem);
    }
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

    validate_catalog_unique_ids();

    if (s_stop_sem == NULL) {
        s_stop_sem = xSemaphoreCreateBinaryStatic(&s_stop_sem_buf);
        if (s_stop_sem == NULL) {
            ESP_LOGE(TAG, "Failed to create stop semaphore");
            return ESP_FAIL;
        }
    }
    xSemaphoreTake(s_stop_sem, 0);

    esp_err_t ret = start_optional_services();
    if (ret != ESP_OK) {
        return ret;
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
        stop_optional_services();
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

    if (s_stop_sem == NULL ||
        xSemaphoreTake(s_stop_sem, pdMS_TO_TICKS(250)) != pdTRUE) {
        ESP_LOGE(TAG, "Timed out waiting for manager task to stop");
        return ESP_ERR_TIMEOUT;
    }

    stop_optional_services();

    ESP_LOGI(TAG, "Manager deinitialized");
    return ESP_OK;
}