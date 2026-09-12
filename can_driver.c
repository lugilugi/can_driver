#include "can_driver.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include "freertos/queue.h"
#include "freertos/timers.h"
#include <string.h>
#include <stdint.h>

#if defined(CONFIG_TWAI_ISR_CACHE_SAFE) && CONFIG_TWAI_ISR_CACHE_SAFE
#error "can_driver does not support CONFIG_TWAI_ISR_CACHE_SAFE"
#endif

static const char *TAG = "can_driver";

// =============================================================================
// Driver-side resources
// =============================================================================

#define CAN_DRIVER_TX_DEPTH 4   /**< TX slot pool size (must be >= 1) */
#define CAN_DRIVER_RX_DEPTH 8   /**< RX queue depth (must be >= 1) */
#define CAN_MAX_FILTER_IDS 32   /**< Max IDs accepted by the filter config */
#define CAN_DRIVER_CLASSIC_DATA_LEN 8U

#if CAN_DRIVER_TX_DEPTH < 1
#error "CAN_DRIVER_TX_DEPTH must be at least 1"
#endif

#if CAN_DRIVER_RX_DEPTH < 1
#error "CAN_DRIVER_RX_DEPTH must be at least 1"
#endif

typedef enum {
    CAN_LIFECYCLE_UNINITIALIZED = 0,
    CAN_LIFECYCLE_STARTING,
    CAN_LIFECYCLE_RUNNING,
    CAN_LIFECYCLE_STOPPING,
    CAN_LIFECYCLE_STOPPED_ERROR,
} can_lifecycle_state_t;

typedef struct {
    twai_frame_header_t header;
    uint8_t data[CAN_DRIVER_CLASSIC_DATA_LEN];
} can_rx_item_t;

typedef struct {
    twai_frame_t frame;
    uint8_t data[CAN_DRIVER_CLASSIC_DATA_LEN];
} can_tx_slot_t;

typedef struct {
    can_lifecycle_state_t state;
    twai_node_handle_t node;
    bool node_enabled;

    // Public calls and a deferred recovery callback hold an active-call
    // reference while they can access node/queue state. Deinit is refused
    // while this is nonzero, so queues are never deleted underneath waiters.
    uint32_t active_calls;

    QueueHandle_t rx_queue;
    StaticQueue_t rx_queue_storage;
    uint8_t rx_queue_buffer[CAN_DRIVER_RX_DEPTH * sizeof(can_rx_item_t)];

    // The queue contains actual slot identities, not just an availability
    // count. This makes ownership transfer atomic for multiple TX producers
    // and the TX-done ISR.
    QueueHandle_t tx_free_queue;
    StaticQueue_t tx_free_queue_storage;
    uint8_t tx_free_queue_buffer[CAN_DRIVER_TX_DEPTH * sizeof(uint8_t)];
    can_tx_slot_t tx_slots[CAN_DRIVER_TX_DEPTH];

    volatile uint32_t rx_dropped;
    volatile uint32_t software_dropped;
    volatile uint32_t bus_error_count;
    volatile uint32_t malformed_frame_count;
    volatile uint32_t recovery_failure_count;
    volatile uint32_t tx_reclaim_error_count;

    bool recovery_pending;

    // Software acceptance filter snapshot. The copy is intentionally owned
    // by the driver because the ISR must not depend on caller storage.
    uint32_t filter_ids[CAN_MAX_FILTER_IDS];
    uint8_t filter_count;
    bool filter_extd;
} can_driver_context_t;

static can_driver_context_t s_ctx = {
    .state = CAN_LIFECYCLE_UNINITIALIZED,
};

// Protects lifecycle state, active-call references, and deferred recovery
// state. It is not held across a potentially blocking queue operation.
static portMUX_TYPE s_state_mux = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// Lifecycle synchronization helpers
// =============================================================================

static esp_err_t can_begin_init(void)
{
    esp_err_t ret = ESP_OK;

    portENTER_CRITICAL(&s_state_mux);
    if (s_ctx.state != CAN_LIFECYCLE_UNINITIALIZED) {
        ret = ESP_ERR_INVALID_STATE;
    } else {
        s_ctx.state = CAN_LIFECYCLE_STARTING;
    }
    portEXIT_CRITICAL(&s_state_mux);

    return ret;
}

static esp_err_t can_operation_begin(can_driver_context_t **ctx_out)
{
    esp_err_t ret = ESP_OK;

    portENTER_CRITICAL(&s_state_mux);
    if (s_ctx.state != CAN_LIFECYCLE_RUNNING ||
        s_ctx.node == NULL || !s_ctx.node_enabled) {
        ret = ESP_ERR_INVALID_STATE;
    } else {
        s_ctx.active_calls++;
        *ctx_out = &s_ctx;
    }
    portEXIT_CRITICAL(&s_state_mux);

    return ret;
}

static void can_operation_end(can_driver_context_t *ctx)
{
    portENTER_CRITICAL(&s_state_mux);
    if (ctx->active_calls > 0) {
        ctx->active_calls--;
    }
    portEXIT_CRITICAL(&s_state_mux);
}

// =============================================================================
// Event callbacks
// =============================================================================

static bool can_on_tx_done(twai_node_handle_t node,
                           const twai_tx_done_event_data_t *edata,
                           void *user_ctx)
{
    (void)node;

    can_driver_context_t *ctx = user_ctx;
    if (ctx == NULL || edata == NULL || edata->done_tx_frame == NULL ||
        ctx->tx_free_queue == NULL) {
        return false;
    }

    // The callback receives the same frame pointer that was submitted. Avoid
    // pointer subtraction on an unrelated pointer so a bad callback payload
    // cannot invoke undefined pointer arithmetic.
    uintptr_t first = (uintptr_t)&ctx->tx_slots[0].frame;
    uintptr_t last = first + sizeof(ctx->tx_slots);
    uintptr_t done = (uintptr_t)edata->done_tx_frame;
    uintptr_t offset = done - first;

    if (done < first || done >= last ||
        (offset % sizeof(can_tx_slot_t)) != 0) {
        ctx->tx_reclaim_error_count++;
        return false;
    }

    uint8_t idx = (uint8_t)(offset / sizeof(can_tx_slot_t));
    BaseType_t yield = pdFALSE;
    if (xQueueSendFromISR(ctx->tx_free_queue, &idx, &yield) != pdPASS) {
        // This indicates a duplicate completion or a broken ownership path.
        // Never block or overwrite a queue from an ISR.
        ctx->tx_reclaim_error_count++;
    }
    return (yield == pdTRUE);
}

static bool can_on_rx_done(twai_node_handle_t node,
                           const twai_rx_done_event_data_t *edata,
                           void *user_ctx)
{
    (void)edata;

    can_driver_context_t *ctx = user_ctx;
    if (ctx == NULL || ctx->rx_queue == NULL) {
        return false;
    }

    can_rx_item_t item = {0};
    twai_frame_t rx_frame = {
        .buffer = item.data,
        .buffer_len = sizeof(item.data),
    };
    if (twai_node_receive_from_isr(node, &rx_frame) != ESP_OK) {
        return false;
    }

    // This wrapper is deliberately classic-CAN only. The hardware filter
    // also excludes FD frames, but keep the invariant at the software
    // boundary for targets/controllers where the header still reports them.
    if (rx_frame.header.fdf || rx_frame.header.dlc > CAN_DRIVER_CLASSIC_DATA_LEN) {
        ctx->malformed_frame_count++;
        return false;
    }

    // Remote frames carry a requested DLC but no data bytes. The queue and
    // public receive API therefore report zero payload bytes for RTR.
    item.header = rx_frame.header;

    // Optional software whitelist: discard frames the hardware region
    // over-accepts. Only the declared format can match the list; frames of
    // the other format are dropped too.
    if (ctx->filter_count > 0) {
        if (rx_frame.header.ide != ctx->filter_extd) {
            ctx->software_dropped++;
            return false;
        }
        bool match = false;
        for (uint8_t i = 0; i < ctx->filter_count; i++) {
            if (ctx->filter_ids[i] == rx_frame.header.id) {
                match = true;
                break;
            }
        }
        if (!match) {
            ctx->software_dropped++;
            return false;
        }
    }

    BaseType_t yield = pdFALSE;
    if (xQueueSendFromISR(ctx->rx_queue, &item, &yield) != pdPASS) {
        ctx->rx_dropped++;
    }
    return (yield == pdTRUE);
}

static void can_complete_recovery_from_task(can_driver_context_t *ctx)
{
    portENTER_CRITICAL(&s_state_mux);
    if (ctx->recovery_pending) {
        ctx->recovery_pending = false;
        if (ctx->active_calls > 0) {
            ctx->active_calls--;
        }
    }
    portEXIT_CRITICAL(&s_state_mux);
}

// Runs in the FreeRTOS timer-service task, not in the TWAI ISR. This keeps
// twai_node_recover() out of ISR context while preserving automatic recovery.
static void can_recover_from_task(void *arg1, uint32_t arg2)
{
    (void)arg2;

    can_driver_context_t *ctx = arg1;
    if (ctx == NULL) {
        return;
    }

    twai_node_handle_t node;
    portENTER_CRITICAL(&s_state_mux);
    node = ctx->node;
    bool recovery_pending = ctx->recovery_pending;
    bool can_recover = (recovery_pending &&
                        ctx->state == CAN_LIFECYCLE_RUNNING &&
                        node != NULL && ctx->node_enabled);
    portEXIT_CRITICAL(&s_state_mux);

    if (!recovery_pending) {
        return;
    }

    esp_err_t ret = can_recover ? twai_node_recover(node) : ESP_ERR_INVALID_STATE;
    if (ret == ESP_OK) {
        // Recovery is asynchronous. The state-change ISR releases the
        // lifecycle reference once the node leaves TWAI_ERROR_BUS_OFF.
        ESP_LOGI(TAG, "bus-off recovery started");
    } else {
        ctx->recovery_failure_count++;
        ESP_LOGE(TAG, "bus-off recovery failed: %s", esp_err_to_name(ret));
        can_complete_recovery_from_task(ctx);
    }
}

static bool can_on_state_change(twai_node_handle_t node,
                                const twai_state_change_event_data_t *edata,
                                void *user_ctx)
{
    (void)node;

    can_driver_context_t *ctx = user_ctx;
    if (ctx == NULL || edata == NULL) {
        return false;
    }

    if (edata->old_sta == TWAI_ERROR_BUS_OFF &&
        edata->new_sta != TWAI_ERROR_BUS_OFF) {
        // twai_node_recover() returns when recovery starts; this callback is
        // the completion signal for the held lifecycle reference.
        portENTER_CRITICAL_ISR(&s_state_mux);
        if (ctx->recovery_pending) {
            ctx->recovery_pending = false;
            if (ctx->active_calls > 0) {
                ctx->active_calls--;
            }
        }
        portEXIT_CRITICAL_ISR(&s_state_mux);
        return false;
    }

    if (edata->new_sta != TWAI_ERROR_BUS_OFF) {
        return false;
    }

    bool schedule_recovery = false;
    portENTER_CRITICAL_ISR(&s_state_mux);
    if (ctx->state == CAN_LIFECYCLE_RUNNING && !ctx->recovery_pending) {
        ctx->recovery_pending = true;
        ctx->active_calls++;
        schedule_recovery = true;
    }
    portEXIT_CRITICAL_ISR(&s_state_mux);

    if (!schedule_recovery) {
        return false;
    }

    BaseType_t yield = pdFALSE;
    if (xTimerPendFunctionCallFromISR(can_recover_from_task, ctx, 0, &yield) != pdPASS) {
        portENTER_CRITICAL_ISR(&s_state_mux);
        if (ctx->recovery_pending) {
            ctx->recovery_pending = false;
            if (ctx->active_calls > 0) {
                ctx->active_calls--;
            }
        }
        ctx->recovery_failure_count++;
        portEXIT_CRITICAL_ISR(&s_state_mux);
    }
    return (yield == pdTRUE);
}

static bool can_on_error(twai_node_handle_t node,
                         const twai_error_event_data_t *edata,
                         void *user_ctx)
{
    (void)node;
    (void)edata;

    can_driver_context_t *ctx = user_ctx;
    if (ctx != NULL) {
        ctx->bus_error_count++;
    }
    return false;
}

// =============================================================================
// Acceptance filter computation
// =============================================================================

/**
 * @brief Compute the mask of the bits shared by all IDs.
 *
 * Bits that differ between any two IDs become don't-care (0) so that the
 * single hardware filter accepts the smallest maskable region containing the
 * whole list.
 */
static uint32_t can_compute_mask(const uint32_t *ids, uint32_t count, uint32_t full_mask)
{
    uint32_t vary = 0;
    for (uint32_t i = 1; i < count; i++) {
        vary |= ids[0] ^ ids[i];
    }
    return full_mask & ~vary;
}

/** @brief Number of IDs the region described by code/mask accepts. */
static uint32_t can_region_size(uint32_t mask, uint32_t full_mask)
{
    return 1u << (uint32_t)__builtin_popcount(full_mask & ~mask);
}

static esp_err_t can_validate_filter(const CanFilterConfig_t *filter)
{
    if (filter == NULL || filter->id_count == 0) {
        return ESP_OK;
    }

    if (filter->id_count > CAN_MAX_FILTER_IDS) {
        ESP_LOGE(TAG, "filter: too many IDs (%lu, max %d)",
                 (unsigned long)filter->id_count, CAN_MAX_FILTER_IDS);
        return ESP_ERR_INVALID_ARG;
    }
    if (filter->ids == NULL) {
        ESP_LOGE(TAG, "filter: ids must not be NULL when id_count is nonzero");
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t full_mask = filter->extd ? TWAI_EXT_ID_MASK : TWAI_STD_ID_MASK;
    for (uint32_t i = 0; i < filter->id_count; i++) {
        if (filter->ids[i] > full_mask) {
            ESP_LOGE(TAG, "filter: ID 0x%lx out of range for %s format",
                     (unsigned long)filter->ids[i],
                     filter->extd ? "extended" : "standard");
            return ESP_ERR_INVALID_ARG;
        }
        for (uint32_t j = 0; j < i; j++) {
            if (filter->ids[i] == filter->ids[j]) {
                ESP_LOGE(TAG, "filter: duplicate ID 0x%lx", (unsigned long)filter->ids[i]);
                return ESP_ERR_INVALID_ARG;
            }
        }
    }
    return ESP_OK;
}

static esp_err_t can_configure_filter(const CanFilterConfig_t *filter)
{
    esp_err_t ret = can_validate_filter(filter);
    if (ret != ESP_OK) {
        return ret;
    }

    bool use_open_filter = (filter == NULL || filter->accept_all || filter->id_count == 0);
    uint32_t code = 0;
    uint32_t mask = 0;
    uint32_t full_mask = 0;

    if (!use_open_filter) {
        full_mask = filter->extd ? TWAI_EXT_ID_MASK : TWAI_STD_ID_MASK;
        mask = can_compute_mask(filter->ids, filter->id_count, full_mask);
        code = filter->ids[0] & mask;
        uint32_t region = can_region_size(mask, full_mask);

        if (region > filter->id_count) {
            // The list cannot be expressed exactly with a single hardware
            // filter. The software filter can discard the spill IDs.
            ESP_LOGW(TAG,
                     "filter: IDs not exactly maskable; region 0x%lx (mask 0x%lx) also accepts %lu extra IDs%s",
                     (unsigned long)code, (unsigned long)mask,
                     (unsigned long)(region - filter->id_count),
                     filter->software_filter ? "" :
                     " - set .software_filter to discard them");
        } else {
            ESP_LOGI(TAG, "filter: %lu IDs accepted exactly (code 0x%lx, mask 0x%lx)",
                     (unsigned long)filter->id_count,
                     (unsigned long)code, (unsigned long)mask);
        }
    }

    // This component is classic-CAN only, including when the target supports
    // CAN-FD. no_fd prevents FD frames from entering the RX callback.
    twai_mask_filter_config_t mf = {
        .id = code,
        .mask = mask,
        .is_ext = use_open_filter ? false : filter->extd,
        .no_fd = 1,
    };
    ret = twai_node_config_mask_filter(s_ctx.node, 0, &mf);
    if (ret != ESP_OK) {
        return ret;
    }

    s_ctx.filter_count = 0;
    s_ctx.filter_extd = false;
    if (filter != NULL && filter->software_filter && filter->id_count > 0) {
        s_ctx.filter_extd = filter->extd;
        s_ctx.filter_count = (uint8_t)filter->id_count;
        memcpy(s_ctx.filter_ids, filter->ids,
               filter->id_count * sizeof(s_ctx.filter_ids[0]));
    }
    return ESP_OK;
}

// =============================================================================
// Resource and initialization helpers
// =============================================================================

static void can_free_wrapper_resources(void)
{
    if (s_ctx.rx_queue != NULL) {
        vQueueDelete(s_ctx.rx_queue);
        s_ctx.rx_queue = NULL;
    }
    if (s_ctx.tx_free_queue != NULL) {
        vQueueDelete(s_ctx.tx_free_queue);
        s_ctx.tx_free_queue = NULL;
    }

    memset(s_ctx.tx_slots, 0, sizeof(s_ctx.tx_slots));
    memset(s_ctx.filter_ids, 0, sizeof(s_ctx.filter_ids));
    s_ctx.filter_count = 0;
    s_ctx.filter_extd = false;
}

static esp_err_t can_create_wrapper_resources(void)
{
    s_ctx.rx_queue = xQueueCreateStatic(
        CAN_DRIVER_RX_DEPTH,
        sizeof(can_rx_item_t),
        s_ctx.rx_queue_buffer,
        &s_ctx.rx_queue_storage);
    s_ctx.tx_free_queue = xQueueCreateStatic(
        CAN_DRIVER_TX_DEPTH,
        sizeof(uint8_t),
        s_ctx.tx_free_queue_buffer,
        &s_ctx.tx_free_queue_storage);

    if (s_ctx.rx_queue == NULL || s_ctx.tx_free_queue == NULL) {
        ESP_LOGE(TAG, "failed to create driver queues");
        can_free_wrapper_resources();
        return ESP_ERR_NO_MEM;
    }

    for (uint8_t i = 0; i < CAN_DRIVER_TX_DEPTH; i++) {
        if (xQueueSend(s_ctx.tx_free_queue, &i, 0) != pdPASS) {
            ESP_LOGE(TAG, "failed to seed TX free-slot queue");
            can_free_wrapper_resources();
            return ESP_ERR_NO_MEM;
        }
    }
    return ESP_OK;
}

static bool can_cleanup_node(void)
{
    if (s_ctx.node == NULL) {
        return true;
    }

    if (s_ctx.node_enabled) {
        esp_err_t ret = twai_node_disable(s_ctx.node);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to stop TWAI node during cleanup: %s",
                     esp_err_to_name(ret));
            return false;
        }
        s_ctx.node_enabled = false;
    }

    esp_err_t ret = twai_node_delete(s_ctx.node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to delete TWAI node during cleanup: %s",
                 esp_err_to_name(ret));
        return false;
    }
    s_ctx.node = NULL;
    return true;
}

static esp_err_t can_fail_init(esp_err_t original_error)
{
    if (!can_cleanup_node()) {
        // Keep the node and wrapper storage available for diagnosis. The
        // state prevents normal API use and avoids repeating teardown blindly.
        portENTER_CRITICAL(&s_state_mux);
        s_ctx.state = CAN_LIFECYCLE_STOPPED_ERROR;
        portEXIT_CRITICAL(&s_state_mux);
        return original_error;
    }

    can_free_wrapper_resources();
    portENTER_CRITICAL(&s_state_mux);
    s_ctx.state = CAN_LIFECYCLE_UNINITIALIZED;
    portEXIT_CRITICAL(&s_state_mux);
    return original_error;
}

// =============================================================================
// Public lifecycle API
// =============================================================================

esp_err_t can_driver_init(gpio_num_t tx_io,
                          gpio_num_t rx_io,
                          uint32_t baud,
                          CanInitFlags_t flags,
                          const CanFilterConfig_t *filter)
{
    esp_err_t ret = can_begin_init();
    if (ret != ESP_OK) {
        return ret;
    }

    // No callback can be active in STARTING because the old node has already
    // been deleted before a subsequent init is allowed.
    memset(&s_ctx, 0, sizeof(s_ctx));
    s_ctx.state = CAN_LIFECYCLE_STARTING;

    if (baud == 0) {
        return can_fail_init(ESP_ERR_INVALID_ARG);
    }
    ret = can_validate_filter(filter);
    if (ret != ESP_OK) {
        return can_fail_init(ret);
    }

    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx = tx_io,
        .io_cfg.rx = rx_io,
        .io_cfg.quanta_clk_out = GPIO_NUM_NC,
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,
        .bit_timing.bitrate = baud,
        .tx_queue_depth = CAN_DRIVER_TX_DEPTH,
        .flags.enable_loopback = flags.loopback,
        .flags.enable_self_test = flags.loopback,
        .flags.enable_listen_only = flags.listen_only,
    };

    ret = twai_new_node_onchip(&node_cfg, &s_ctx.node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to create TWAI node: %s", esp_err_to_name(ret));
        return can_fail_init(ret);
    }

    ret = can_configure_filter(filter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure acceptance filter: %s", esp_err_to_name(ret));
        return can_fail_init(ret);
    }

    twai_event_callbacks_t cbs = {
        .on_tx_done = can_on_tx_done,
        .on_rx_done = can_on_rx_done,
        .on_state_change = can_on_state_change,
        .on_error = can_on_error,
    };
    ret = twai_node_register_event_callbacks(s_ctx.node, &cbs, &s_ctx);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to register event callbacks: %s", esp_err_to_name(ret));
        return can_fail_init(ret);
    }

    ret = can_create_wrapper_resources();
    if (ret != ESP_OK) {
        return can_fail_init(ret);
    }

    s_ctx.rx_dropped = 0;
    s_ctx.software_dropped = 0;
    s_ctx.bus_error_count = 0;
    s_ctx.malformed_frame_count = 0;
    s_ctx.recovery_failure_count = 0;
    s_ctx.tx_reclaim_error_count = 0;

    ret = twai_node_enable(s_ctx.node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to enable TWAI node: %s", esp_err_to_name(ret));
        return can_fail_init(ret);
    }
    s_ctx.node_enabled = true;

    portENTER_CRITICAL(&s_state_mux);
    s_ctx.state = CAN_LIFECYCLE_RUNNING;
    portEXIT_CRITICAL(&s_state_mux);

    ESP_LOGI(TAG, "classic CAN initialized at %lu bps", (unsigned long)baud);
    return ESP_OK;
}

esp_err_t can_driver_deinit(void)
{
    portENTER_CRITICAL(&s_state_mux);
    bool stopped_error = (s_ctx.state == CAN_LIFECYCLE_STOPPED_ERROR);
    bool node_enabled = s_ctx.node_enabled;
    if (s_ctx.state != CAN_LIFECYCLE_RUNNING && !stopped_error) {
        portEXIT_CRITICAL(&s_state_mux);
        return ESP_ERR_INVALID_STATE;
    }
    if (s_ctx.active_calls != 0) {
        portEXIT_CRITICAL(&s_state_mux);
        return ESP_ERR_INVALID_STATE;
    }
    if (stopped_error && !node_enabled) {
        // The node has already passed through a successful disable and a
        // failed delete. Do not repeat a partially destructive operation
        // without an IDF contract that guarantees retry safety.
        portEXIT_CRITICAL(&s_state_mux);
        return ESP_ERR_INVALID_STATE;
    }
    s_ctx.state = CAN_LIFECYCLE_STOPPING;
    portEXIT_CRITICAL(&s_state_mux);

    if (node_enabled) {
        esp_err_t ret = twai_node_disable(s_ctx.node);
        if (ret != ESP_OK) {
            ESP_LOGE(TAG, "failed to stop TWAI node: %s", esp_err_to_name(ret));
            portENTER_CRITICAL(&s_state_mux);
            s_ctx.state = CAN_LIFECYCLE_RUNNING;
            portEXIT_CRITICAL(&s_state_mux);
            return ret;
        }
        s_ctx.node_enabled = false;
    }

    esp_err_t ret = twai_node_delete(s_ctx.node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to delete TWAI node: %s", esp_err_to_name(ret));
        portENTER_CRITICAL(&s_state_mux);
        s_ctx.state = CAN_LIFECYCLE_STOPPED_ERROR;
        portEXIT_CRITICAL(&s_state_mux);
        return ret;
    }

    s_ctx.node = NULL;
    can_free_wrapper_resources();
    portENTER_CRITICAL(&s_state_mux);
    s_ctx.state = CAN_LIFECYCLE_UNINITIALIZED;
    portEXIT_CRITICAL(&s_state_mux);
    ESP_LOGI(TAG, "TWAI deinitialized");
    return ESP_OK;
}

// =============================================================================
// Transmit / Receive
// =============================================================================

static esp_err_t can_validate_tx_frame(const twai_frame_t *frame, uint16_t *dlc_out)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (frame->header.fdf || frame->header.brs) {
        return ESP_ERR_NOT_SUPPORTED;
    }
    if (frame->buffer_len > CAN_DRIVER_CLASSIC_DATA_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }
    if (frame->buffer_len > 0 && frame->buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t id_mask = frame->header.ide ? TWAI_EXT_ID_MASK : TWAI_STD_ID_MASK;
    if (frame->header.id > id_mask) {
        return ESP_ERR_INVALID_ARG;
    }
    if (frame->header.dlc > CAN_DRIVER_CLASSIC_DATA_LEN) {
        return ESP_ERR_INVALID_ARG;
    }

    if (frame->header.rtr) {
        if (frame->buffer_len != 0) {
            return ESP_ERR_INVALID_ARG;
        }
    } else if (frame->header.dlc != 0 &&
               twaifd_dlc2len(frame->header.dlc) != frame->buffer_len) {
        return ESP_ERR_INVALID_SIZE;
    }

    *dlc_out = (frame->header.dlc != 0) ? frame->header.dlc :
               (uint16_t)twaifd_len2dlc(frame->buffer_len);
    return ESP_OK;
}

esp_err_t can_driver_transmit(const twai_frame_t *frame, TickType_t timeout_ticks)
{
    uint16_t dlc;
    esp_err_t ret = can_validate_tx_frame(frame, &dlc);
    if (ret != ESP_OK) {
        return ret;
    }

    can_driver_context_t *ctx;
    ret = can_operation_begin(&ctx);
    if (ret != ESP_OK) {
        return ret;
    }

    // Claim a concrete slot. The node driver queues frames by pointer, so the
    // slot must stay alive until TX completes; on_tx_done returns its index.
    uint8_t idx;
    if (xQueueReceive(ctx->tx_free_queue, &idx, timeout_ticks) != pdPASS) {
        can_operation_end(ctx);
        return ESP_ERR_TIMEOUT;
    }

    can_tx_slot_t *slot = &ctx->tx_slots[idx];
    slot->frame.header = frame->header;
    slot->frame.header.dlc = dlc;
    slot->frame.buffer = slot->data;
    slot->frame.buffer_len = frame->header.rtr ? 0 : frame->buffer_len;
    if (slot->frame.buffer_len > 0) {
        memcpy(slot->data, frame->buffer, slot->frame.buffer_len);
    }

    // Once a wrapper slot is owned, it is the capacity gate. A native queue
    // wait here would spend the caller's timeout twice.
    ret = twai_node_transmit(ctx->node, &slot->frame, 0);
    if (ret != ESP_OK) {
        if (xQueueSend(ctx->tx_free_queue, &idx, 0) != pdPASS) {
            ctx->tx_reclaim_error_count++;
        }
    }

    can_operation_end(ctx);
    return ret;
}

static esp_err_t can_get_payload_length(const twai_frame_header_t *header,
                                        uint8_t *length_out)
{
    if (header->fdf || header->dlc > CAN_DRIVER_CLASSIC_DATA_LEN) {
        return ESP_ERR_INVALID_SIZE;
    }
    *length_out = header->rtr ? 0 : (uint8_t)twaifd_dlc2len(header->dlc);
    return ESP_OK;
}

esp_err_t can_driver_receive(twai_frame_t *frame,
                             size_t buffer_capacity,
                             TickType_t timeout_ticks)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (buffer_capacity > 0 && frame->buffer == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    can_driver_context_t *ctx;
    esp_err_t ret = can_operation_begin(&ctx);
    if (ret != ESP_OK) {
        return ret;
    }

    can_rx_item_t item;
    if (xQueueReceive(ctx->rx_queue, &item, timeout_ticks) != pdPASS) {
        can_operation_end(ctx);
        return ESP_ERR_TIMEOUT;
    }

    uint8_t payload_length = 0;
    ret = can_get_payload_length(&item.header, &payload_length);
    frame->header = item.header;
    frame->buffer_len = payload_length;
    if (ret != ESP_OK) {
        ctx->malformed_frame_count++;
        can_operation_end(ctx);
        return ret;
    }
    if (payload_length > buffer_capacity) {
        // The frame is consumed, but no partial copy is performed. The
        // caller still receives the header and required length for logging.
        can_operation_end(ctx);
        return ESP_ERR_INVALID_SIZE;
    }
    if (payload_length > 0) {
        memcpy(frame->buffer, item.data, payload_length);
    }

    can_operation_end(ctx);
    return ESP_OK;
}

// =============================================================================
// Status
// =============================================================================

esp_err_t can_driver_get_status(CanStatus_t *status)
{
    if (status == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    can_driver_context_t *ctx;
    esp_err_t ret = can_operation_begin(&ctx);
    if (ret != ESP_OK) {
        return ret;
    }

    twai_node_status_t ns;
    twai_node_record_t nr;
    ret = twai_node_get_info(ctx->node, &ns, &nr);
    if (ret != ESP_OK) {
        can_operation_end(ctx);
        return ret;
    }

    uint32_t bus_error_count;
    uint32_t rx_dropped;
    uint32_t software_dropped;
    uint32_t malformed_frame_count;
    uint32_t recovery_failure_count;
    portENTER_CRITICAL(&s_state_mux);
    bus_error_count = ctx->bus_error_count;
    rx_dropped = ctx->rx_dropped;
    software_dropped = ctx->software_dropped;
    malformed_frame_count = ctx->malformed_frame_count;
    recovery_failure_count = ctx->recovery_failure_count;
    portEXIT_CRITICAL(&s_state_mux);

    status->error_state = ns.state;
    status->tx_error_count = ns.tx_error_count;
    status->rx_error_count = ns.rx_error_count;
    status->tx_slots_remaining = (uint32_t)uxQueueMessagesWaiting(ctx->tx_free_queue);
    status->twai_tx_queue_remaining = ns.tx_queue_remaining;
    status->rx_queue_remaining = (uint32_t)uxQueueSpacesAvailable(ctx->rx_queue);
    status->bus_error_count = bus_error_count;
    status->rx_dropped_count = rx_dropped;
    status->software_dropped_count = software_dropped;
    status->malformed_frame_count = malformed_frame_count;
    status->recovery_failure_count = recovery_failure_count;

    can_operation_end(ctx);
    return ESP_OK;
}
