#include "can_driver.h"
#include "can_config.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/queue.h"
#include "freertos/portmacro.h"
#include <string.h>
#include <stdlib.h>

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
static portMUX_TYPE  s_bus_off_mux     = portMUX_INITIALIZER_UNLOCKED;

// ISR counter spinlock — protects the task-context read/reset side.
// ISR increments are unguarded by design: on single-core targets the ISR is
// naturally atomic; on dual-core the worst case is a missed diagnostic count,
// which is acceptable given the critical-section overhead saved per frame.
static portMUX_TYPE  s_isr_cnt_mux     = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// ISR diagnostic counters
// Incremented in on_rx_done to distinguish two failure modes:
//   s_isr_rx_calls == 0  →  on_rx_done is never invoked (filter / driver issue)
//   s_isr_rx_fail  > 0  →  callback fires but twai_node_receive_from_isr fails
// Reset and read via can_driver_reset_isr_counters() / can_driver_get_isr_rx_*()
// =============================================================================
static volatile uint32_t s_isr_rx_calls = 0;
static volatile uint32_t s_isr_rx_fail  = 0;
static volatile uint32_t s_isr_rx_dropped = 0;

// =============================================================================
// TX pool helpers
// =============================================================================

static inline bool in_isr_context(void)
{
    return xPortInIsrContext();
}

static CanTxSlot_t *tx_pool_claim(bool isr_context)
{
    CanTxSlot_t *slot = NULL;
    if (isr_context) {
        portENTER_CRITICAL_ISR(&s_tx_pool_mux);
    } else {
        portENTER_CRITICAL(&s_tx_pool_mux);
    }

    for (int i = 0; i < CAN_TX_POOL_SIZE; i++) {
        if (!s_tx_pool[i].in_use) {
            s_tx_pool[i].in_use = true;
            slot = &s_tx_pool[i];
            break;
        }
    }

    if (isr_context) {
        portEXIT_CRITICAL_ISR(&s_tx_pool_mux);
    } else {
        portEXIT_CRITICAL(&s_tx_pool_mux);
    }

    return slot;
}

// Release from task context (error path in can_driver_transmit).
static void tx_pool_release(CanTxSlot_t *slot, bool isr_context)
{
    if (isr_context) {
        portENTER_CRITICAL_ISR(&s_tx_pool_mux);
    } else {
        portENTER_CRITICAL(&s_tx_pool_mux);
    }

    slot->in_use = false;

    if (isr_context) {
        portEXIT_CRITICAL_ISR(&s_tx_pool_mux);
    } else {
        portEXIT_CRITICAL(&s_tx_pool_mux);
    }
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
    if (s_rx_queue == NULL ||
        xQueueSendFromISR(s_rx_queue, &evt, &higher_prio_woken) != pdTRUE) {
        s_isr_rx_dropped++;
    }
    return (higher_prio_woken == pdTRUE);
}

static bool IRAM_ATTR on_state_change(twai_node_handle_t handle,
                                       const twai_state_change_event_data_t *edata,
                                       void *user_ctx)
{
    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        // Signal the manager task — twai_node_recover() must not be called
        // from ISR context.
        portENTER_CRITICAL_ISR(&s_bus_off_mux);
        s_bus_off_pending = true;
        portEXIT_CRITICAL_ISR(&s_bus_off_mux);
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

esp_err_t can_driver_init(gpio_num_t tx, gpio_num_t rx, uint32_t baud, CanInitFlags_t flags)
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
    s_isr_rx_dropped  = 0;

    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx                = tx,
        .io_cfg.rx                = rx,
        .bit_timing.bitrate       = baud,
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
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
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
             baud,
             flags.loopback, flags.self_test, flags.listen_only, flags.no_rtr,
             tx, rx);
    return ESP_OK;

cleanup:
    if (s_node_hdl != NULL) {
        twai_node_delete(s_node_hdl);
    }
    s_node_hdl = NULL;
    if (s_rx_queue != NULL) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }
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
    if (s_rx_queue != NULL) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }
    portENTER_CRITICAL(&s_bus_off_mux);
    s_bus_off_pending = false;
    portEXIT_CRITICAL(&s_bus_off_mux);
    ESP_LOGI(TAG, "Deinitialized");
    return ret;
}

esp_err_t can_driver_transmit(uint32_t id, const uint8_t *data, uint8_t len)
{
    bool isr_context = in_isr_context();

    if (s_node_hdl == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (len > 8 || (len > 0 && data == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    CanTxSlot_t *slot = tx_pool_claim(isr_context);
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
        tx_pool_release(slot, isr_context);
        if (!isr_context) {
            ESP_LOGW(TAG, "twai_node_transmit failed (id=0x%03lX): %s",
                     (unsigned long)id, esp_err_to_name(ret));
        }
    }
    return ret;
}

esp_err_t can_driver_receive(CanRxEvent_t *evt, TickType_t timeout_ticks)
{
    if (evt == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_node_hdl == NULL || s_rx_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

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
    bool pending;
    portENTER_CRITICAL(&s_bus_off_mux);
    pending = s_bus_off_pending;
    portEXIT_CRITICAL(&s_bus_off_mux);
    return pending;
}

void can_driver_clear_bus_off(void)
{
    portENTER_CRITICAL(&s_bus_off_mux);
    s_bus_off_pending = false;
    portEXIT_CRITICAL(&s_bus_off_mux);
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
    uint32_t val;
    portENTER_CRITICAL(&s_isr_cnt_mux);
    val = s_isr_rx_calls;
    portEXIT_CRITICAL(&s_isr_cnt_mux);
    return val;
}

uint32_t can_driver_get_isr_rx_fail(void)
{
    uint32_t val;
    portENTER_CRITICAL(&s_isr_cnt_mux);
    val = s_isr_rx_fail;
    portEXIT_CRITICAL(&s_isr_cnt_mux);
    return val;
}

uint32_t can_driver_get_isr_rx_dropped(void)
{
    uint32_t val;
    portENTER_CRITICAL(&s_isr_cnt_mux);
    val = s_isr_rx_dropped;
    portEXIT_CRITICAL(&s_isr_cnt_mux);
    return val;
}

void can_driver_reset_isr_counters(void)
{
    portENTER_CRITICAL(&s_isr_cnt_mux);
    s_isr_rx_calls   = 0;
    s_isr_rx_fail    = 0;
    s_isr_rx_dropped = 0;
    portEXIT_CRITICAL(&s_isr_cnt_mux);
}

// =============================================================================
// Filter helpers (internal)
// =============================================================================

// Bit-width of the ID field for the selected frame format.
// Standard: 11 bits → full_mask = 0x7FF
// Extended: 29 bits → full_mask = 0x1FFFFFFF
static inline uint32_t filter_full_mask(bool is_ext)
{
    return is_ext ? 0x1FFFFFFFU : 0x7FFU;
}

// Compute the tightest single mask filter that accepts every ID in [ids, count).
//
// For each bit position b:
//   • If every ID has bit b == 1  →  agree, filter ID bit = 1, mask bit = 1
//   • If every ID has bit b == 0  →  agree, filter ID bit = 0, mask bit = 1
//   • Mixed                       →  don't-care,              mask bit = 0
//
// Returns the number of spurious IDs accepted beyond the supplied list.
// A spurious count of 0 means the filter is a perfect whitelist.
static uint32_t filter_compute_group(const uint32_t *ids, size_t count,
                                     bool is_ext,
                                     uint32_t *out_id, uint32_t *out_mask)
{
    const uint32_t full = filter_full_mask(is_ext);

    if (count == 0) {
        // Reject-all: id = full, mask = full.
        // No ID can satisfy (incoming & full) == full for a randomly chosen
        // full-bit pattern, so this effectively blocks everything.
        *out_id   = full;
        *out_mask = full;
        return 0;
    }

    // all_ones: bits that are 1 in EVERY ID  →  agreed-1 positions
    // any_ones: bits that are 1 in AT LEAST one ID
    uint32_t all_ones = full;
    uint32_t any_ones = 0;
    for (size_t i = 0; i < count; i++) {
        all_ones &= ids[i];
        any_ones |= ids[i];
    }

    // agreed_mask: bit = 1 where all IDs agree (either all-0 or all-1)
    //   all_ones            → bits that are 1 in every ID (agreed-1)
    //   (~any_ones) & full  → bits that are 0 in every ID (agreed-0)
    uint32_t agreed_mask = all_ones | ((~any_ones) & full);

    *out_id   = all_ones;    // agreed-1 bits carry value 1; agreed-0 stay 0
    *out_mask = agreed_mask; // only agreed bits are enforced

    // Number of don't-care bit positions = number of 0s in agreed_mask
    uint32_t dc_bits   = (uint32_t)__builtin_popcount((~agreed_mask) & full);
    uint32_t n_accepts = (1U << dc_bits);  // IDs that pass = 2^(don't-care bits)

    // Spurious = accepted IDs that are NOT in our list
    return (n_accepts > (uint32_t)count) ? (n_accepts - (uint32_t)count) : 0;
}

// Shared disable→reconfigure→enable sequence.
// The hardware filter registers are write-protected while the node is running.
static esp_err_t filter_apply_config(const twai_mask_filter_config_t *cfg,
                                     const char *label)
{
    if (s_node_hdl == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Disable: immediately stops the node; any in-flight TX is aborted.
    // The caller is responsible for ensuring no critical TX is pending.
    esp_err_t ret = twai_node_disable(s_node_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s: twai_node_disable failed: %s", label, esp_err_to_name(ret));
        return ret;
    }

    ret = twai_node_config_mask_filter(s_node_hdl, 0, cfg);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s: twai_node_config_mask_filter failed: %s",
                 label, esp_err_to_name(ret));
        // Best-effort re-enable even on filter error.
        twai_node_enable(s_node_hdl);
        return ret;
    }

    ret = twai_node_enable(s_node_hdl);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "%s: twai_node_enable failed: %s", label, esp_err_to_name(ret));
    }
    return ret;
}

// =============================================================================
// Filter configuration — explicit (manual)
// =============================================================================

esp_err_t can_driver_apply_single_filter(uint32_t id, uint32_t mask, bool is_ext)
{
    ESP_LOGI(TAG, "Single filter: id=0x%08lX mask=0x%08lX ext=%d",
             (unsigned long)id, (unsigned long)mask, is_ext);

    twai_mask_filter_config_t cfg = {
        .id     = id,
        .mask   = mask,
        .is_ext = is_ext,
    };
    return filter_apply_config(&cfg, "apply_single_filter");
}

esp_err_t can_driver_apply_dual_filter(uint32_t id1, uint32_t mask1,
                                        uint32_t id2, uint32_t mask2,
                                        bool is_ext)
{
    ESP_LOGI(TAG, "Dual filter: f1=0x%08lX/0x%08lX  f2=0x%08lX/0x%08lX  ext=%d",
             (unsigned long)id1, (unsigned long)mask1,
             (unsigned long)id2, (unsigned long)mask2, is_ext);

    // twai_make_dual_filter() packs both id/mask pairs into the single
    // twai_mask_filter_config_t register layout expected by the hardware and
    // sets the internal "dual mode" flag automatically.
    twai_mask_filter_config_t cfg = twai_make_dual_filter(id1, mask1,
                                                           id2, mask2,
                                                           is_ext);
    return filter_apply_config(&cfg, "apply_dual_filter");
}

// =============================================================================
// Filter configuration — automatic (computed from an ID list)
// =============================================================================

esp_err_t can_driver_apply_single_filter_auto(const uint32_t *ids,
                                               size_t count,
                                               bool is_ext)
{
    if (ids == NULL && count > 0) {
        return ESP_ERR_INVALID_ARG;
    }
    if (count == 0) {
        ESP_LOGW(TAG, "apply_single_filter_auto: empty list → accept-all");
        twai_mask_filter_config_t accept_all = { .id = 0, .mask = 0, .is_ext = is_ext };
        return filter_apply_config(&accept_all, "apply_single_filter_auto");
    }

    uint32_t filter_id, filter_mask;
    uint32_t spurious = filter_compute_group(ids, count, is_ext,
                                             &filter_id, &filter_mask);

    ESP_LOGI(TAG, "Single filter auto (%d IDs): id=0x%08lX mask=0x%08lX spurious=%lu",
             (int)count,
             (unsigned long)filter_id, (unsigned long)filter_mask,
             (unsigned long)spurious);

    twai_mask_filter_config_t cfg = {
        .id     = filter_id,
        .mask   = filter_mask,
        .is_ext = is_ext,
    };
    return filter_apply_config(&cfg, "apply_single_filter_auto");
}

// qsort comparator for uint32_t ascending.
static int cmp_u32(const void *a, const void *b)
{
    uint32_t ua = *(const uint32_t *)a;
    uint32_t ub = *(const uint32_t *)b;
    return (ua > ub) - (ua < ub);   // branchless; avoids signed overflow
}

// Stack-allocated scratch cap for dual filter auto.
// 128 IDs × 4 bytes = 512 bytes on stack — safe for typical task stacks and
// avoids non-deterministic heap allocation in a peripheral configuration path.
// Lists exceeding this cap fall back to single-filter mode.
#define DUAL_FILTER_MAX_IDS 128

esp_err_t can_driver_apply_dual_filter_auto(const uint32_t *ids,
                                             size_t count,
                                             bool is_ext)
{
    if (ids == NULL && count > 0) return ESP_ERR_INVALID_ARG;

    if (count == 0) {
        ESP_LOGW(TAG, "apply_dual_filter_auto: empty list → accept-all");
        twai_mask_filter_config_t accept_all = { .id = 0, .mask = 0, .is_ext = is_ext };
        return filter_apply_config(&accept_all, "apply_dual_filter_auto");
    }

    if (count == 1) {
        const uint32_t full = filter_full_mask(is_ext);
        ESP_LOGI(TAG, "Dual filter auto (1 ID): exact match id=0x%08lX",
                 (unsigned long)ids[0]);
        return can_driver_apply_dual_filter(ids[0], full, full, full, is_ext);
    }

    if (count > DUAL_FILTER_MAX_IDS) {
        ESP_LOGW(TAG, "apply_dual_filter_auto: %d IDs > %d cap → single filter fallback",
                 (int)count, DUAL_FILTER_MAX_IDS);
        return can_driver_apply_single_filter_auto(ids, count, is_ext);
    }

    // Stack-allocated scratch — no heap, deterministic lifetime.
    uint32_t sorted[DUAL_FILTER_MAX_IDS];
    memcpy(sorted, ids, count * sizeof(uint32_t));
    qsort(sorted, count, sizeof(uint32_t), cmp_u32);

    size_t   split_idx = 0;
    uint32_t max_gap   = 0;
    for (size_t i = 0; i < count - 1; i++) {
        uint32_t gap = sorted[i + 1] - sorted[i];
        if (gap > max_gap) { max_gap = gap; split_idx = i; }
    }

    const uint32_t *group_a = &sorted[0];
    size_t          count_a = split_idx + 1;
    const uint32_t *group_b = &sorted[split_idx + 1];
    size_t          count_b = count - count_a;

    uint32_t id1, mask1, id2, mask2;
    uint32_t spurious_a = filter_compute_group(group_a, count_a, is_ext, &id1, &mask1);
    uint32_t spurious_b = filter_compute_group(group_b, count_b, is_ext, &id2, &mask2);

    ESP_LOGI(TAG,
             "Dual filter auto (%d IDs, split at gap %lu after 0x%08lX): "
             "f1=0x%08lX/0x%08lX (%d IDs, %lu spurious)  "
             "f2=0x%08lX/0x%08lX (%d IDs, %lu spurious)",
             (int)count, (unsigned long)max_gap, (unsigned long)sorted[split_idx],
             (unsigned long)id1, (unsigned long)mask1, (int)count_a, (unsigned long)spurious_a,
             (unsigned long)id2, (unsigned long)mask2, (int)count_b, (unsigned long)spurious_b);

    return can_driver_apply_dual_filter(id1, mask1, id2, mask2, is_ext);
}