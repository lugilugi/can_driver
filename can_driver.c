#include "can_driver.h"
#include "can_config.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include <string.h>

static const char *TAG = "can_driver";

// =============================================================================
// TWAI node handle
// =============================================================================
static twai_node_handle_t s_node_hdl = NULL;

// =============================================================================
// Static TX pool
//
// Each slot bundles a twai_frame_t with the data buffer it points to.
// The TWAI API holds a pointer to twai_frame_t::buffer until on_tx_done fires,
// so the buffer MUST outlive the call to twai_node_transmit(). Keeping frame
// and buffer colocated in one struct makes this trivially safe — the pointer
// never outlives the slot.
//
// Slot lifecycle:  IDLE -> (claimed) -> IN_FLIGHT -> (on_tx_done ISR) -> IDLE
// =============================================================================
typedef struct {
    twai_frame_t  frame;
    uint8_t       data[8];
    volatile bool in_use;
} CanTxSlot_t;

static DRAM_ATTR CanTxSlot_t s_tx_pool[CAN_TX_POOL_SIZE];
static portMUX_TYPE          s_tx_pool_mux = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// Static RX queue — ISR writes, manager task reads.
// DRAM_ATTR ensures the backing storage is accessible when the flash cache is
// disabled (e.g. during NVS writes or OTA).
// =============================================================================
static DRAM_ATTR StaticQueue_t s_rx_queue_struct;
static DRAM_ATTR uint8_t       s_rx_queue_storage[CAN_RX_QUEUE_DEPTH * sizeof(CanRxEvent_t)];
static QueueHandle_t           s_rx_queue = NULL;

// =============================================================================
// Bus-off flag — set by ISR, polled and cleared by the manager task.
// =============================================================================
static volatile bool s_bus_off_pending = false;

// =============================================================================
// ISR diagnostic counters
// Incremented in on_rx_done to distinguish two failure modes:
//   s_isr_rx_calls == 0  →  on_rx_done is never invoked (filter / driver issue)
//   s_isr_rx_fail  > 0  →  callback fires but twai_node_receive_from_isr fails
// Reset and read via can_driver_reset_isr_counters() / can_driver_get_isr_rx_*()
// =============================================================================
static volatile uint32_t s_isr_rx_calls = 0;
static volatile uint32_t s_isr_rx_fail  = 0;

// =============================================================================
// TX pool helpers
// =============================================================================

static CanTxSlot_t *tx_pool_claim(void)
{
    CanTxSlot_t *slot = NULL;
    portENTER_CRITICAL(&s_tx_pool_mux);
    for (int i = 0; i < CAN_TX_POOL_SIZE; i++) {
        if (!s_tx_pool[i].in_use) {
            s_tx_pool[i].in_use = true;
            slot = &s_tx_pool[i];
            break;
        }
    }
    portEXIT_CRITICAL(&s_tx_pool_mux);
    return slot;
}

// Release from task context (error path in can_driver_transmit).
static void tx_pool_release(CanTxSlot_t *slot)
{
    portENTER_CRITICAL(&s_tx_pool_mux);
    slot->in_use = false;
    portEXIT_CRITICAL(&s_tx_pool_mux);
}

// Release from ISR context — matches slot by frame pointer address.
static void IRAM_ATTR tx_pool_release_isr(const twai_frame_t *done_frame)
{
    portENTER_CRITICAL_ISR(&s_tx_pool_mux);
    for (int i = 0; i < CAN_TX_POOL_SIZE; i++) {
        if (&s_tx_pool[i].frame == done_frame) {
            s_tx_pool[i].in_use = false;
            break;
        }
    }
    portEXIT_CRITICAL_ISR(&s_tx_pool_mux);
}

// =============================================================================
// ISR callbacks
// IRAM_ATTR keeps them reachable when the flash cache is disabled.
// Return value: true = yield to higher-priority task on ISR exit.
// =============================================================================

static bool IRAM_ATTR on_tx_done(twai_node_handle_t handle,
                                  const twai_tx_done_event_data_t *edata,
                                  void *user_ctx)
{
    tx_pool_release_isr(edata->done_tx_frame);
    return false;
}

static bool IRAM_ATTR on_rx_done(twai_node_handle_t handle,
                                  const twai_rx_done_event_data_t *edata,
                                  void *user_ctx)
{
    // Count every invocation — if this stays 0 after TX in loopback mode,
    // the filter is rejecting frames before the callback is ever called.
    s_isr_rx_calls++;

    // Capture the tick immediately — most accurate bus arrival timestamp.
    TickType_t rx_tick = xTaskGetTickCountFromISR();

    uint8_t      recv_buf[8] = {0};
    twai_frame_t rx_frame    = {
        .buffer     = recv_buf,
        .buffer_len = sizeof(recv_buf),
    };

    if (twai_node_receive_from_isr(handle, &rx_frame) != ESP_OK) {
        s_isr_rx_fail++;
        return false;
    }

    CanRxEvent_t evt = {
        .id      = rx_frame.header.id,
        .len     = (rx_frame.header.dlc <= 8) ? (uint8_t)rx_frame.header.dlc : 8,
        .rx_tick = rx_tick,
    };
    memcpy(evt.data, recv_buf, evt.len);

    BaseType_t higher_prio_woken = pdFALSE;
    xQueueSendFromISR(s_rx_queue, &evt, &higher_prio_woken);
    return (higher_prio_woken == pdTRUE);
}

static bool IRAM_ATTR on_state_change(twai_node_handle_t handle,
                                       const twai_state_change_event_data_t *edata,
                                       void *user_ctx)
{
    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        // Signal the manager task — twai_node_recover() must not be called
        // from ISR context.
        s_bus_off_pending = true;
    }
    return false;
}

static bool IRAM_ATTR on_error(twai_node_handle_t handle,
                                const twai_error_event_data_t *edata,
                                void *user_ctx)
{
    // Kept minimal — ISR callbacks should do as little as possible.
    // The manager can call twai_node_get_info() if detailed diagnostics are
    // needed from task context.
    return false;
}

// =============================================================================
// Public API
// =============================================================================

esp_err_t can_driver_init(CanInitFlags_t flags)
{
    if (s_node_hdl != NULL) {
        ESP_LOGE(TAG, "Already initialized — call can_driver_deinit() first");
        return ESP_ERR_INVALID_STATE;
    }

    // Validate flag combinations before touching hardware.
    if (flags.loopback && flags.listen_only) {
        ESP_LOGE(TAG, "loopback and listen_only are mutually exclusive");
        return ESP_ERR_INVALID_ARG;
    }

    // Loopback without self_test causes TX failures when no other node is
    // present to supply the ACK. Enforce the pairing silently.
    if (flags.loopback) {
        flags.self_test = 1;
    }

    s_rx_queue = xQueueCreateStatic(
        CAN_RX_QUEUE_DEPTH,
        sizeof(CanRxEvent_t),
        s_rx_queue_storage,
        &s_rx_queue_struct
    );
    if (s_rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create static RX queue");
        return ESP_FAIL;
    }

    // Explicit zero — makes re-init after deinit safe in addition to BSS zero.
    memset(s_tx_pool, 0, sizeof(s_tx_pool));
    s_bus_off_pending = false;
    s_isr_rx_calls    = 0;
    s_isr_rx_fail     = 0;

    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx                = CAN_TX_GPIO,
        .io_cfg.rx                = CAN_RX_GPIO,
        .bit_timing.bitrate       = CAN_BAUD_RATE,
        .tx_queue_depth           = CAN_TX_QUEUE_DEPTH,
        .fail_retry_cnt           = CAN_TX_RETRY_COUNT,
        .flags.enable_self_test   = (uint8_t)flags.self_test,
        .flags.enable_loopback    = (uint8_t)flags.loopback,
        .flags.enable_listen_only = (uint8_t)flags.listen_only,
        .flags.no_receive_rtr     = (uint8_t)flags.no_rtr,
    };

    esp_err_t ret = twai_new_node_onchip(&node_cfg, &s_node_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_new_node_onchip failed: %s", esp_err_to_name(ret));
        return ret;
    }

    twai_event_callbacks_t cbs = {
        .on_rx_done      = on_rx_done,
        .on_tx_done      = on_tx_done,
        .on_state_change = on_state_change,
        .on_error        = on_error,
    };
    ret = twai_node_register_event_callbacks(s_node_hdl, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_register_event_callbacks failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    // Configure accept-all filter BEFORE enable — the hardware requires the
    // node to be stopped when writing filter registers.
    // id=0, mask=0 → every bit is don't-care → every standard frame passes.
    twai_mask_filter_config_t accept_all = {
        .id     = 0,
        .mask   = 0,
        .is_ext = false,
    };
    ret = twai_node_config_mask_filter(s_node_hdl, 0, &accept_all);
    ESP_LOGI(TAG, "Filter config: %s", esp_err_to_name(ret));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_config_mask_filter failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ret = twai_node_enable(s_node_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "twai_node_enable failed: %s", esp_err_to_name(ret));
        goto cleanup;
    }

    ESP_LOGI(TAG, "Initialized at %d bps [loopback=%d self_test=%d listen_only=%d no_rtr=%d] (TX=%d RX=%d)",
             CAN_BAUD_RATE,
             flags.loopback, flags.self_test, flags.listen_only, flags.no_rtr,
             CAN_TX_GPIO, CAN_RX_GPIO);
    return ESP_OK;

cleanup:
    twai_node_delete(s_node_hdl);
    s_node_hdl = NULL;
    return ret;
}

esp_err_t can_driver_deinit(void)
{
    if (s_node_hdl == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    twai_node_disable(s_node_hdl);
    esp_err_t ret = twai_node_delete(s_node_hdl);
    s_node_hdl = NULL;
    ESP_LOGI(TAG, "Deinitialized");
    return ret;
}

esp_err_t can_driver_transmit(uint32_t id, const uint8_t *data, uint8_t len)
{
    if (s_node_hdl == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > 8) {
        return ESP_ERR_INVALID_ARG;
    }

    CanTxSlot_t *slot = tx_pool_claim();
    if (slot == NULL) {
        return ESP_ERR_NO_MEM;
    }

    memcpy(slot->data, data, len);
    slot->frame = (twai_frame_t){
        .header.id  = id,
        .header.ide = false,    // 11-bit standard ID
        .buffer     = slot->data,
        .buffer_len = len,
    };

    // timeout=0: non-blocking. If the hardware TX queue is full the frame is
    // dropped and the slot is released immediately.
    esp_err_t ret = twai_node_transmit(s_node_hdl, &slot->frame, 0);
    if (ret != ESP_OK) {
        tx_pool_release(slot);
        ESP_LOGW(TAG, "twai_node_transmit failed (id=0x%03lX): %s",
                 (unsigned long)id, esp_err_to_name(ret));
    }
    return ret;
}

esp_err_t can_driver_receive(CanRxEvent_t *evt, TickType_t timeout_ticks)
{
    if (xQueueReceive(s_rx_queue, evt, timeout_ticks) == pdTRUE) {
        return ESP_OK;
    }
    return ESP_ERR_TIMEOUT;
}

int can_driver_get_pool_used(void)
{
    int count = 0;
    portENTER_CRITICAL(&s_tx_pool_mux);
    for (int i = 0; i < CAN_TX_POOL_SIZE; i++) {
        if (s_tx_pool[i].in_use) count++;
    }
    portEXIT_CRITICAL(&s_tx_pool_mux);
    return count;
}

bool can_driver_is_bus_off(void)
{
    return s_bus_off_pending;
}

void can_driver_clear_bus_off(void)
{
    s_bus_off_pending = false;
}

esp_err_t can_driver_recover(void)
{
    if (s_node_hdl == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    return twai_node_recover(s_node_hdl);
}

uint32_t can_driver_get_isr_rx_calls(void)
{
    return s_isr_rx_calls;
}

uint32_t can_driver_get_isr_rx_fail(void)
{
    return s_isr_rx_fail;
}

void can_driver_reset_isr_counters(void)
{
    s_isr_rx_calls = 0;
    s_isr_rx_fail  = 0;
}