#include "can_usb_forward.h"

// Nothing to compile when disabled — the header provides inline stubs.
#if CAN_USB_FORWARD_ENABLED

#include "can_payloads.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <ctype.h>

static const char *TAG = "can_usb";

// =============================================================================
// Time anchor
// =============================================================================
static portMUX_TYPE s_time_mux    = portMUX_INITIALIZER_UNLOCKED;
static int64_t      s_epoch_ms    = 0;
static TickType_t   s_anchor_tick = 0;
static bool         s_anchored    = false;
static portMUX_TYPE s_drop_mux    = portMUX_INITIALIZER_UNLOCKED;
static uint32_t     s_drops       = 0;

static void drops_inc(void)
{
    portENTER_CRITICAL(&s_drop_mux);
    s_drops++;
    portEXIT_CRITICAL(&s_drop_mux);
}

static uint32_t drops_take_clear(void)
{
    uint32_t d;
    portENTER_CRITICAL(&s_drop_mux);
    d = s_drops;
    s_drops = 0;
    portEXIT_CRITICAL(&s_drop_mux);
    return d;
}

void can_usb_forward_anchor_time(int64_t rtc_epoch_ms)
{
    portENTER_CRITICAL(&s_time_mux);
    s_epoch_ms    = rtc_epoch_ms;
    s_anchor_tick = xTaskGetTickCount();
    s_anchored    = true;
    portEXIT_CRITICAL(&s_time_mux);
}

static int64_t get_ts_ms(TickType_t rx_tick)
{
    int64_t epoch_ms;
    TickType_t anchor_tick;
    bool anchored;

    portENTER_CRITICAL(&s_time_mux);
    epoch_ms = s_epoch_ms;
    anchor_tick = s_anchor_tick;
    anchored = s_anchored;
    portEXIT_CRITICAL(&s_time_mux);

    if (anchored) return epoch_ms + (int64_t)(rx_tick - anchor_tick);
    return (int64_t)rx_tick;
}

// =============================================================================
// RX queue (CAN -> phone)
// =============================================================================
static StaticQueue_t s_rx_queue_struct;
static uint8_t       s_rx_queue_storage[CAN_USB_FWD_QUEUE_DEPTH * sizeof(CanRxEvent_t)];
static QueueHandle_t s_rx_queue  = NULL;
static volatile bool s_running   = false;

// Cooperative shutdown semaphores — one per task.
static StaticSemaphore_t s_rx_stop_sem_buf;
static SemaphoreHandle_t s_rx_stop_sem = NULL;
static StaticSemaphore_t s_tx_stop_sem_buf;
static SemaphoreHandle_t s_tx_stop_sem = NULL;

// =============================================================================
// RX task — reads CAN frames from queue, prints candump lines to stdout
// =============================================================================
static StaticTask_t s_rx_task_buf;
static StackType_t  s_rx_task_stack[CAN_USB_FWD_TASK_STACK_SIZE];
static TaskHandle_t s_rx_task_hdl = NULL;

static void rx_task(void *arg)
{
    ESP_LOGI(TAG, "RX task started (CAN -> USB)");

    CanRxEvent_t evt;

    while (s_running) {
        if (xQueueReceive(s_rx_queue, &evt, pdMS_TO_TICKS(100)) != pdTRUE) {
            continue;
        }

        int64_t ts_ms  = get_ts_ms(evt.rx_tick);
        int64_t ts_sec = ts_ms / 1000;
        int     ts_rem = (int)(ts_ms % 1000);

        // Inline hex — avoids heap allocation per frame
        char hex[17] = {0};
        for (uint8_t i = 0; i < evt.len && i < 8; i++) {
            hex[i * 2]     = "0123456789ABCDEF"[evt.data[i] >> 4];
            hex[i * 2 + 1] = "0123456789ABCDEF"[evt.data[i] & 0x0F];
        }

        // candump format: (sec.ms) can0 ID#DATA
        printf("(%lld.%03d) can0 %03lX#%s\n",
               (long long)ts_sec, ts_rem,
               (unsigned long)evt.id, hex);
    }

    ESP_LOGI(TAG, "RX task stopped");
    if (s_rx_stop_sem != NULL) {
        xSemaphoreGive(s_rx_stop_sem);
    }
    s_rx_task_hdl = NULL;
    vTaskDelete(NULL);
}

// =============================================================================
// TX task — reads lines from stdin, parses and transmits CAN frames
//
// Accepted formats (both produce the same result):
//   (1234.567) can0 110#DEADBEEF\n   <- full candump, timestamp ignored
//   110#DEADBEEF\n                   <- shorthand, no timestamp needed
//
// Parser strategy:
//   Scan the line for the last token containing '#'. Split on '#' to get
//   the ID (hex) and data (hex pairs). This makes it robust to both formats
//   without caring about what comes before the ID#DATA token.
//
// Error responses written back to stdout so the phone can detect them:
//   ERR bad_id\n
//   ERR bad_data\n
//   ERR tx_failed <esp_err code>\n
//   OK\n
// =============================================================================
static StaticTask_t s_tx_task_buf;
static StackType_t  s_tx_task_stack[CAN_USB_FWD_TASK_STACK_SIZE];
static TaskHandle_t s_tx_task_hdl = NULL;

// Convert a hex character to its nibble value.
// Returns -1 if the character is not valid hex.
static int hex_nibble(char c)
{
    if (c >= '0' && c <= '9') return c - '0';
    if (c >= 'A' && c <= 'F') return c - 'A' + 10;
    if (c >= 'a' && c <= 'f') return c - 'a' + 10;
    return -1;
}

// Parse "ID#HEXDATA" token into id and data buffer.
// Returns the number of data bytes parsed, or -1 on error.
static int parse_frame(const char *token, uint32_t *out_id, uint8_t *out_data)
{
    // Find the '#' separator
    const char *hash = strchr(token, '#');
    if (hash == NULL) return -1;

    // Parse ID (hex, up to 8 chars before '#')
    size_t id_len = (size_t)(hash - token);
    if (id_len == 0 || id_len > 8) return -1;

    uint32_t id = 0;
    for (size_t i = 0; i < id_len; i++) {
        int n = hex_nibble(token[i]);
        if (n < 0) return -1;
        id = (id << 4) | (uint32_t)n;
    }
    *out_id = id;

    // Parse data bytes (pairs of hex chars after '#')
    const char *p = hash + 1;
    int nbytes = 0;
    while (*p && nbytes < 8) {
        // Skip any spaces between byte pairs (some tools insert them)
        while (*p == ' ') p++;
        if (*p == '\0' || *p == '\r' || *p == '\n') break;

        int hi = hex_nibble(p[0]);
        int lo = hex_nibble(p[1]);
        if (hi < 0 || lo < 0) return -1;

        out_data[nbytes++] = (uint8_t)((hi << 4) | lo);
        p += 2;
    }

    return nbytes;
}

static void tx_task(void *arg)
{
    ESP_LOGI(TAG, "TX task started (USB -> CAN)");

    char line[64];

    while (s_running) {
        // fgets blocks until a newline or EOF. On USB Serial/JTAG this yields
        // the task when no data is available so it doesn't spin the CPU.
        if (fgets(line, sizeof(line), stdin) == NULL) {
            vTaskDelay(pdMS_TO_TICKS(10));
            continue;
        }

        // Strip trailing whitespace
        char *end = line + strlen(line) - 1;
        while (end >= line && (*end == '\n' || *end == '\r' || *end == ' ')) {
            *end-- = '\0';
        }
        if (line[0] == '\0') continue;

        // Find the token containing '#' — scan backwards through space-delimited
        // tokens so the timestamp and "can0" prefix are naturally skipped.
        char *token = NULL;
        char *p = line;
        char *last_hash_token = NULL;
        while (*p) {
            // Skip spaces
            while (*p == ' ') p++;
            if (*p == '\0') break;
            char *tok_start = p;
            // Find end of token
            while (*p && *p != ' ') p++;
            // Null-terminate this token temporarily
            char saved = *p;
            *p = '\0';
            if (strchr(tok_start, '#') != NULL) {
                last_hash_token = tok_start;
            }
            *p = saved;
            if (saved) p++;
        }
        token = last_hash_token;

        if (token == NULL) {
            printf("ERR no_frame_token\n");
            continue;
        }

        uint32_t id = 0;
        uint8_t  data[8] = {0};
        int nbytes = parse_frame(token, &id, data);

        if (nbytes < 0) {
            printf("ERR bad_frame\n");
            continue;
        }

        // Validate ID range — standard 11-bit only for now
        if (id > 0x7FF) {
            printf("ERR bad_id\n");
            continue;
        }

        esp_err_t ret = can_driver_transmit(id, data, (uint8_t)nbytes);
        if (ret == ESP_OK) {
            printf("OK\n");
        } else {
            printf("ERR tx_failed_%d\n", (int)ret);
        }
    }

    ESP_LOGI(TAG, "TX task stopped");
    if (s_tx_stop_sem != NULL) {
        xSemaphoreGive(s_tx_stop_sem);
    }
    s_tx_task_hdl = NULL;
    vTaskDelete(NULL);
}

// =============================================================================
// Public API
// =============================================================================

esp_err_t can_usb_forward_init(void)
{
    if (s_rx_task_hdl != NULL || s_tx_task_hdl != NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    // Create stop semaphores for cooperative shutdown.
    if (s_rx_stop_sem == NULL) {
        s_rx_stop_sem = xSemaphoreCreateBinaryStatic(&s_rx_stop_sem_buf);
        if (s_rx_stop_sem == NULL) return ESP_FAIL;
    }
    xSemaphoreTake(s_rx_stop_sem, 0);

    if (s_tx_stop_sem == NULL) {
        s_tx_stop_sem = xSemaphoreCreateBinaryStatic(&s_tx_stop_sem_buf);
        if (s_tx_stop_sem == NULL) return ESP_FAIL;
    }
    xSemaphoreTake(s_tx_stop_sem, 0);

    s_rx_queue = xQueueCreateStatic(
        CAN_USB_FWD_QUEUE_DEPTH, sizeof(CanRxEvent_t),
        s_rx_queue_storage, &s_rx_queue_struct);
    if (s_rx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create RX queue");
        return ESP_FAIL;
    }

    s_running = true;
    s_drops = 0;

    s_rx_task_hdl = xTaskCreateStaticPinnedToCore(
        rx_task, "can_usb_rx",
        CAN_USB_FWD_TASK_STACK_SIZE, NULL,
        CAN_USB_FWD_RX_TASK_PRIORITY, s_rx_task_stack, &s_rx_task_buf,
        CAN_USB_FWD_TASK_CORE);

    s_tx_task_hdl = xTaskCreateStaticPinnedToCore(
        tx_task, "can_usb_tx",
        CAN_USB_FWD_TASK_STACK_SIZE, NULL,
        CAN_USB_FWD_TX_TASK_PRIORITY, s_tx_task_stack, &s_tx_task_buf,
        CAN_USB_FWD_TASK_CORE);

    if (s_rx_task_hdl == NULL || s_tx_task_hdl == NULL) {
        s_running = false;
        if (s_rx_task_hdl != NULL) {
            vTaskDelete(s_rx_task_hdl);
            s_rx_task_hdl = NULL;
        }
        if (s_tx_task_hdl != NULL) {
            vTaskDelete(s_tx_task_hdl);
            s_tx_task_hdl = NULL;
        }
        if (s_rx_queue != NULL) {
            vQueueDelete(s_rx_queue);
            s_rx_queue = NULL;
        }
        ESP_LOGE(TAG, "Failed to create tasks");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "USB bridge ready — RX: candump out, TX: ID#HEX in");
    return ESP_OK;
}

esp_err_t can_usb_forward_deinit(void)
{
    if (s_rx_task_hdl == NULL && s_tx_task_hdl == NULL && s_rx_queue == NULL) {
        return ESP_ERR_INVALID_STATE;
    }

    s_running = false;

    // Cooperative RX shutdown — task checks s_running every 100 ms.
    if (s_rx_stop_sem != NULL) {
        if (xSemaphoreTake(s_rx_stop_sem, pdMS_TO_TICKS(300)) != pdTRUE) {
            ESP_LOGW(TAG, "RX task did not stop in time — force deleting");
            if (s_rx_task_hdl != NULL) { vTaskDelete(s_rx_task_hdl); s_rx_task_hdl = NULL; }
        }
    }

    // TX task may block on fgets(stdin) indefinitely — use generous timeout
    // with force-delete fallback. This is inherent to blocking I/O; no clean
    // workaround exists without non-blocking stdin.
    if (s_tx_stop_sem != NULL) {
        if (xSemaphoreTake(s_tx_stop_sem, pdMS_TO_TICKS(500)) != pdTRUE) {
            ESP_LOGW(TAG, "TX task did not stop in time (blocked on stdin?) — force deleting");
            if (s_tx_task_hdl != NULL) { vTaskDelete(s_tx_task_hdl); s_tx_task_hdl = NULL; }
        }
    }

    if (s_rx_queue != NULL) {
        vQueueDelete(s_rx_queue);
        s_rx_queue = NULL;
    }

    return ESP_OK;
}

void can_usb_forward_post(const CanRxEvent_t *evt)
{
    if (s_rx_queue == NULL || evt == NULL) return;
    if (xQueueSend(s_rx_queue, evt, 0) != pdTRUE) {
        drops_inc();
    }
}

uint32_t can_usb_forward_get_drop_count(void)
{
    return drops_take_clear();
}

#endif // CAN_USB_FORWARD_ENABLED