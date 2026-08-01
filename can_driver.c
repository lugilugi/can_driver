#include "can_driver.h"
#include "esp_twai.h"
#include "esp_twai_onchip.h"
#include "esp_log.h"
#include <string.h>
#include <stdlib.h>

static const char *TAG = "can_driver";

// =============================================================================
// Driver-side resources
// =============================================================================

#define CAN_DRIVER_TX_DEPTH 4   /**< TX slot pool size (must be >= 1) */
#define CAN_DRIVER_RX_DEPTH 8   /**< RX queue depth (must be >= 1) */
#define CAN_MAX_FILTER_IDS 32   /**< Max IDs accepted by the filter config */

typedef struct {
    twai_frame_header_t header;
    uint8_t data[TWAI_FRAME_MAX_LEN];
    uint8_t len;
} can_rx_item_t;

static twai_node_handle_t s_node = NULL;
static QueueHandle_t s_rx_queue = NULL;
static SemaphoreHandle_t s_tx_slot_sem = NULL;
static twai_frame_t *s_tx_frames = NULL;   /**< [s_tx_depth] frame slots */
static uint8_t (*s_tx_data)[TWAI_FRAME_MAX_LEN] = NULL; /**< [s_tx_depth] data buffers */
static uint8_t *s_tx_in_use = NULL;        /**< [s_tx_depth] slot busy flags */
static uint32_t s_tx_depth = 0;
static uint32_t s_tx_next_slot = 0;
static volatile uint32_t s_rx_dropped = 0; /**< RX frames lost to a full queue */
static volatile uint32_t s_bus_error_count = 0;
static volatile uint32_t s_sw_dropped = 0; /**< Frames dropped by the software filter */

// Software acceptance filter state (opt-in via CanFilterConfig_t.software_filter)
static uint32_t s_filter_ids[CAN_MAX_FILTER_IDS];
static uint8_t s_filter_count = 0;   /**< 0 = software filtering disabled */
static bool s_filter_extd = false;   /**< Format of the software filter list */

// =============================================================================
// Event callbacks (all run in ISR context)
// =============================================================================

static bool can_on_tx_done(twai_node_handle_t node, const twai_tx_done_event_data_t *edata, void *user_ctx)
{
    // The driver queues its own frame slots by pointer, so the slot index is
    // recoverable from the frame address. Free the slot for reuse.
    uint32_t idx = (uint32_t)(edata->done_tx_frame - s_tx_frames);
    if (idx < s_tx_depth) {
        s_tx_in_use[idx] = 0;
        BaseType_t yield = pdFALSE;
        xSemaphoreGiveFromISR(s_tx_slot_sem, &yield);
        return (yield == pdTRUE);
    }
    return false;
}

static bool can_on_rx_done(twai_node_handle_t node, const twai_rx_done_event_data_t *edata, void *user_ctx)
{
    can_rx_item_t item;
    twai_frame_t rx_frame = {
        .buffer = item.data,
        .buffer_len = sizeof(item.data),
    };
    if (twai_node_receive_from_isr(node, &rx_frame) != ESP_OK) {
        return false;
    }

    // Optional software whitelist: discard frames the hardware region
    // over-accepts. Only the declared format (s_filter_extd) can match the
    // list; frames of the other format are dropped too, so the delivered
    // stream is exactly the listed IDs.
    if (s_filter_count > 0) {
        if (rx_frame.header.ide != s_filter_extd) {
            s_sw_dropped++;
            return false;
        }
        bool match = false;
        for (uint8_t i = 0; i < s_filter_count; i++) {
            if (s_filter_ids[i] == rx_frame.header.id) {
                match = true;
                break;
            }
        }
        if (!match) {
            s_sw_dropped++;
            return false;
        }
    }

    item.header = rx_frame.header;
    item.len = (uint8_t)twaifd_dlc2len(rx_frame.header.dlc);
    BaseType_t yield = pdFALSE;
    if (xQueueSendFromISR(s_rx_queue, &item, &yield) != pdTRUE) {
        s_rx_dropped++;
    }
    return (yield == pdTRUE);
}

static bool can_on_state_change(twai_node_handle_t node, const twai_state_change_event_data_t *edata, void *user_ctx)
{
    if (edata->new_sta == TWAI_ERROR_BUS_OFF) {
        ESP_EARLY_LOGW(TAG, "bus-off detected, initiating recovery");
        twai_node_recover(node);
    } else if (edata->old_sta == TWAI_ERROR_BUS_OFF) {
        ESP_EARLY_LOGI(TAG, "node recovered");
    }
    return false;
}

static bool can_on_error(twai_node_handle_t node, const twai_error_event_data_t *edata, void *user_ctx)
{
    s_bus_error_count++;
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

static esp_err_t can_configure_filter(const CanFilterConfig_t *filter)
{
    s_filter_count = 0;
    s_filter_extd = false;

    if (filter == NULL || filter->accept_all || filter->id_count == 0) {
        // id = 0 and mask = 0 disables the filter: accept all frames
        twai_mask_filter_config_t open = {
            .id = 0,
            .mask = 0,
            .is_ext = false,
        };
        if (filter != NULL && filter->software_filter) {
            // Accept-all plus a software whitelist (e.g. to receive a
            // specific list from an ID space that cannot be masked).
            s_filter_extd = filter->extd;
            s_filter_count = (uint8_t)filter->id_count;
            if (filter->id_count > 0) {
                memcpy(s_filter_ids, filter->ids, filter->id_count * sizeof(uint32_t));
            }
        }
        return twai_node_config_mask_filter(s_node, 0, &open);
    }
    if (filter->id_count > CAN_MAX_FILTER_IDS) {
        ESP_LOGE(TAG, "filter: too many IDs (%lu, max %d)", (unsigned long)filter->id_count, CAN_MAX_FILTER_IDS);
        return ESP_ERR_INVALID_ARG;
    }

    uint32_t full_mask = filter->extd ? TWAI_EXT_ID_MASK : TWAI_STD_ID_MASK;
    for (uint32_t i = 0; i < filter->id_count; i++) {
        if (filter->ids[i] > full_mask) {
            ESP_LOGE(TAG, "filter: ID 0x%lx out of range for %s format",
                     (unsigned long)filter->ids[i], filter->extd ? "extended" : "standard");
            return ESP_ERR_INVALID_ARG;
        }
    }

    uint32_t mask = can_compute_mask(filter->ids, filter->id_count, full_mask);
    uint32_t code = filter->ids[0] & mask;
    uint32_t region = can_region_size(mask, full_mask);

    if (region < filter->id_count) {
        ESP_LOGE(TAG, "filter: duplicate IDs in list");
        return ESP_ERR_INVALID_ARG;
    }

    if (region > filter->id_count) {
        // The list cannot be expressed exactly with a single hardware filter.
        // The region also accepts (region - id_count) extra IDs; the
        // software filter discards them so only the listed IDs are delivered.
        ESP_LOGW(TAG, "filter: IDs not exactly maskable; region 0x%03lx (mask 0x%03lx) also accepts %lu extra IDs%s",
                 (unsigned long)code, (unsigned long)mask,
                 (unsigned long)(region - filter->id_count),
                 filter->software_filter ? "" : " — set .software_filter to discard them");
    } else {
        ESP_LOGI(TAG, "filter: %lu IDs accepted exactly (code 0x%03lx, mask 0x%03lx)",
                 (unsigned long)filter->id_count, (unsigned long)code, (unsigned long)mask);
    }

    if (filter->software_filter) {
        s_filter_extd = filter->extd;
        s_filter_count = (uint8_t)filter->id_count;
        memcpy(s_filter_ids, filter->ids, filter->id_count * sizeof(uint32_t));
    }

    twai_mask_filter_config_t mf = {
        .id = code,
        .mask = mask,
        .is_ext = filter->extd,
    };
    return twai_node_config_mask_filter(s_node, 0, &mf);
}

// =============================================================================
// Lifecycle
// =============================================================================

static void can_free_resources(void)
{
    if (s_rx_queue != NULL) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }
    if (s_tx_slot_sem != NULL) {
        vSemaphoreDelete(s_tx_slot_sem);
        s_tx_slot_sem = NULL;
    }
    if (s_tx_frames != NULL) {
        free(s_tx_frames);
        s_tx_frames = NULL;
    }
    if (s_tx_data != NULL) {
        free(s_tx_data);
        s_tx_data = NULL;
    }
    if (s_tx_in_use != NULL) {
        free(s_tx_in_use);
        s_tx_in_use = NULL;
    }
    s_tx_depth = 0;
    s_tx_next_slot = 0;
}

esp_err_t can_driver_init(gpio_num_t tx_io, gpio_num_t rx_io, uint32_t baud, CanInitFlags_t flags, const CanFilterConfig_t *filter)
{
    if (s_node != NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    if (baud == 0) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_onchip_node_config_t node_cfg = {
        .io_cfg.tx = tx_io,
        .io_cfg.rx = rx_io,
        .io_cfg.quanta_clk_out = GPIO_NUM_NC,
        .io_cfg.bus_off_indicator = GPIO_NUM_NC,
        .bit_timing.bitrate = baud,          // driver computes the bit timing
        .tx_queue_depth = CAN_DRIVER_TX_DEPTH,
        .flags.enable_loopback = flags.loopback,
        .flags.enable_self_test = flags.loopback,   // classic chips need no-ack for loopback
        .flags.enable_listen_only = flags.listen_only,
    };

    esp_err_t ret = twai_new_node_onchip(&node_cfg, &s_node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to create TWAI node: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = can_configure_filter(filter);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to configure acceptance filter: %s", esp_err_to_name(ret));
        goto err_node;
    }

    twai_event_callbacks_t cbs = {
        .on_tx_done = can_on_tx_done,
        .on_rx_done = can_on_rx_done,
        .on_state_change = can_on_state_change,
        .on_error = can_on_error,
    };
    ret = twai_node_register_event_callbacks(s_node, &cbs, NULL);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to register event callbacks: %s", esp_err_to_name(ret));
        goto err_node;
    }

    // Driver-side plumbing: TX slot pool and RX queue
    s_tx_frames = calloc(CAN_DRIVER_TX_DEPTH, sizeof(twai_frame_t));
    s_tx_data = calloc(CAN_DRIVER_TX_DEPTH, TWAI_FRAME_MAX_LEN);
    s_tx_in_use = calloc(CAN_DRIVER_TX_DEPTH, sizeof(uint8_t));
    s_tx_slot_sem = xSemaphoreCreateCounting(CAN_DRIVER_TX_DEPTH, CAN_DRIVER_TX_DEPTH);
    s_rx_queue = xQueueCreate(CAN_DRIVER_RX_DEPTH, sizeof(can_rx_item_t));
    if (s_tx_frames == NULL || s_tx_data == NULL || s_tx_in_use == NULL ||
        s_tx_slot_sem == NULL || s_rx_queue == NULL) {
        ESP_LOGE(TAG, "failed to allocate driver resources");
        ret = ESP_ERR_NO_MEM;
        goto err_res;
    }
    s_tx_depth = CAN_DRIVER_TX_DEPTH;
    s_rx_dropped = 0;
    s_sw_dropped = 0;
    s_bus_error_count = 0;

    ret = twai_node_enable(s_node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to enable TWAI node: %s", esp_err_to_name(ret));
        goto err_res;
    }

    ESP_LOGI(TAG, "TWAI initialized at %lu bps", (unsigned long)baud);
    return ESP_OK;

err_res:
    can_free_resources();
err_node:
    // A freshly created node is in the stopped state and can be deleted
    // directly (twai_node_disable would fail since it was never enabled).
    if (s_node != NULL) {
        twai_node_delete(s_node);
        s_node = NULL;
    }
    return ret;
}

esp_err_t can_driver_deinit(void)
{
    if (s_node == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    esp_err_t ret = twai_node_disable(s_node);
    if (ret != ESP_OK) {
        // A node in bus-off cannot be disabled until recovery completes
        ESP_LOGE(TAG, "failed to stop TWAI node: %s", esp_err_to_name(ret));
        return ret;
    }
    ret = twai_node_delete(s_node);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "failed to delete TWAI node: %s", esp_err_to_name(ret));
        return ret;
    }
    s_node = NULL;
    can_free_resources();
    ESP_LOGI(TAG, "TWAI deinitialized");
    return ESP_OK;
}

// =============================================================================
// Transmit / Receive
// =============================================================================

esp_err_t can_driver_transmit(const twai_frame_t *frame, TickType_t timeout_ticks)
{
    if (frame == NULL || frame->buffer_len > TWAI_FRAME_MAX_LEN) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_node == NULL) {
        return ESP_ERR_INVALID_STATE;
    }
    uint32_t id_mask = frame->header.ide ? TWAI_EXT_ID_MASK : TWAI_STD_ID_MASK;
    if (frame->header.id > id_mask) {
        return ESP_ERR_INVALID_ARG;
    }

    // Claim a TX slot. The node driver queues frames by pointer, so the slot
    // must stay alive until TX completes; on_tx_done returns it to the pool.
    if (xSemaphoreTake(s_tx_slot_sem, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint32_t idx = s_tx_next_slot;
    for (uint32_t i = 0; i < s_tx_depth; i++) {
        uint32_t candidate = (s_tx_next_slot + i) % s_tx_depth;
        if (!s_tx_in_use[candidate]) {
            idx = candidate;
            break;
        }
    }

    twai_frame_t *tx = &s_tx_frames[idx];
    tx->header = frame->header;
    tx->header.dlc = (frame->header.dlc != 0) ? frame->header.dlc
                                              : (uint16_t)twaifd_len2dlc(frame->buffer_len);
    tx->buffer = s_tx_data[idx];
    tx->buffer_len = frame->buffer_len;
    if (frame->buffer_len > 0) {
        memcpy(tx->buffer, frame->buffer, frame->buffer_len);
    }

    s_tx_in_use[idx] = 1;
    s_tx_next_slot = (idx + 1) % s_tx_depth;

    int timeout_ms = (timeout_ticks == portMAX_DELAY) ? -1 : (int)pdTICKS_TO_MS(timeout_ticks);
    esp_err_t ret = twai_node_transmit(s_node, tx, timeout_ms);
    if (ret != ESP_OK) {
        s_tx_in_use[idx] = 0;
        xSemaphoreGive(s_tx_slot_sem);
    }
    return ret;
}

esp_err_t can_driver_receive(twai_frame_t *frame, TickType_t timeout_ticks)
{
    if (frame == NULL) {
        return ESP_ERR_INVALID_ARG;
    }
    if (s_node == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    can_rx_item_t item;
    if (xQueueReceive(s_rx_queue, &item, timeout_ticks) != pdTRUE) {
        return ESP_ERR_TIMEOUT;
    }

    uint8_t *user_buf = frame->buffer;
    size_t user_len = frame->buffer_len;

    frame->header = item.header;
    frame->buffer_len = item.len;
    if (user_buf != NULL && user_len > 0) {
        uint32_t copy_len = (item.len < user_len) ? item.len : (uint32_t)user_len;
        memcpy(user_buf, item.data, copy_len);
    }
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
    if (s_node == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    twai_node_status_t ns;
    twai_node_record_t nr;
    esp_err_t ret = twai_node_get_info(s_node, &ns, &nr);
    if (ret != ESP_OK) {
        return ret;
    }

    status->error_state = ns.state;
    status->tx_error_count = ns.tx_error_count;
    status->rx_error_count = ns.rx_error_count;
    status->tx_queue_remaining = ns.tx_queue_remaining;
    status->rx_queue_remaining = (uint32_t)uxQueueSpacesAvailable(s_rx_queue);
    status->bus_error_count = nr.bus_err_num;
    status->rx_dropped_count = s_rx_dropped;
    status->software_dropped_count = s_sw_dropped;
    return ESP_OK;
}
