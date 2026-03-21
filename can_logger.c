#include "can_logger.h"
#include "can_config.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdspi_host.h"
#include "driver/spi_common.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include <string.h>
#include <stdio.h>
#include <sys/time.h>
#include <time.h>

// Only include the SDMMC host header on targets that actually have the
// peripheral. Selecting SDIO on a C3 is a hard compile error.
#if (CAN_LOGGER_BUS == CAN_LOGGER_BUS_SDIO)
#  ifdef CONFIG_SOC_SDMMC_HOST_SUPPORTED
#    include "driver/sdmmc_host.h"
#  else
#    error "CAN_LOGGER_BUS_SDIO selected but this target has no SDMMC host. Use CAN_LOGGER_BUS_SPI instead."
#  endif
#endif

static const char *TAG = "can_logger";

// =============================================================================
// Log record — 32 bytes, 16 records per 512-byte SD sector.
// =============================================================================

#define CAN_LOG_FLAG_NORMAL  0x00
#define CAN_LOG_FLAG_SYNC    0x01

#pragma pack(push, 1)
typedef struct {
    int64_t  wall_time_ms;   //  8 bytes
    uint32_t id;             //  4 bytes
    uint8_t  data[8];        //  8 bytes
    uint8_t  len;            //  1 byte
    uint8_t  flags;          //  1 byte
    uint8_t  _pad[10];       // 10 bytes -> 32 bytes total
} CanLogRecord_t;
#pragma pack(pop)

_Static_assert(sizeof(CanLogRecord_t) == 32,
               "CanLogRecord_t must be exactly 32 bytes");
_Static_assert((CAN_LOGGER_FLUSH_RECORDS % 16) == 0,
               "CAN_LOGGER_FLUSH_RECORDS must be a multiple of 16");

// =============================================================================
// Time anchor
// =============================================================================
static portMUX_TYPE  s_time_mux      = portMUX_INITIALIZER_UNLOCKED;
static int64_t       s_rtc_epoch_ms  = 0;
static TickType_t    s_anchor_tick   = 0;
static volatile bool s_time_anchored = false;
static volatile bool s_sync_pending  = false;

static inline int64_t to_wall_ms(TickType_t rx_tick)
{
    return s_rtc_epoch_ms + (int64_t)(rx_tick - s_anchor_tick);
}

// =============================================================================
// Logger queue
// =============================================================================
static StaticQueue_t     s_queue_struct;
static uint8_t           s_queue_storage[CAN_LOGGER_QUEUE_DEPTH * sizeof(CanRxEvent_t)];
static QueueHandle_t     s_log_queue = NULL;
static volatile uint32_t s_drops     = 0;

// =============================================================================
// Flush buffer
// =============================================================================
static CanLogRecord_t s_flush_buf[CAN_LOGGER_FLUSH_RECORDS];
static size_t         s_flush_pending = 0;

// =============================================================================
// SD card and file state
// =============================================================================
static sdmmc_card_t *s_card       = NULL;
static FILE         *s_file       = NULL;
static uint64_t      s_file_bytes = 0;
static uint32_t      s_file_index = 0;

// =============================================================================
// Task state
// =============================================================================
static StaticTask_t  s_task_buf;
static StackType_t   s_task_stack[CAN_LOGGER_TASK_STACK_SIZE];
static TaskHandle_t  s_task_hdl = NULL;
static volatile bool s_running  = false;

// =============================================================================
// SD mount / unmount
// =============================================================================

static esp_err_t sd_mount(void)
{
    esp_vfs_fat_sdmmc_mount_config_t mount_cfg = {
        .format_if_mount_failed = false,
        .max_files              = 4,
        .allocation_unit_size   = 16 * 1024,
    };

    esp_err_t ret;

#if (CAN_LOGGER_BUS == CAN_LOGGER_BUS_SDIO)

    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    host.max_freq_khz = SDMMC_FREQ_HIGHSPEED;

    sdmmc_slot_config_t slot = SDMMC_SLOT_CONFIG_DEFAULT();
    slot.width = 4;

#  if defined(CONFIG_SOC_SDMMC_USE_GPIO_MATRIX)
    slot.clk = CAN_LOGGER_SDIO_CLK;
    slot.cmd = CAN_LOGGER_SDIO_CMD;
    slot.d0  = CAN_LOGGER_SDIO_D0;
    slot.d1  = CAN_LOGGER_SDIO_D1;
    slot.d2  = CAN_LOGGER_SDIO_D2;
    slot.d3  = CAN_LOGGER_SDIO_D3;
#  endif

    ret = esp_vfs_fat_sdmmc_mount(
        CAN_LOGGER_MOUNT_POINT, &host, &slot, &mount_cfg, &s_card);

#elif (CAN_LOGGER_BUS == CAN_LOGGER_BUS_SPI)

    spi_bus_config_t bus_cfg = {
        .mosi_io_num     = CAN_LOGGER_SPI_MOSI,
        .miso_io_num     = CAN_LOGGER_SPI_MISO,
        .sclk_io_num     = CAN_LOGGER_SPI_CLK,
        .quadwp_io_num   = -1,
        .quadhd_io_num   = -1,
        .max_transfer_sz = 4096,
    };
    ret = spi_bus_initialize(CAN_LOGGER_SPI_HOST, &bus_cfg, SPI_DMA_CH_AUTO);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "spi_bus_initialize failed: %s", esp_err_to_name(ret));
        return ret;
    }

    sdmmc_host_t host = SDSPI_HOST_DEFAULT();
    host.slot = CAN_LOGGER_SPI_HOST;

    sdspi_device_config_t dev_cfg = SDSPI_DEVICE_CONFIG_DEFAULT();
    dev_cfg.gpio_cs  = CAN_LOGGER_SPI_CS;
    dev_cfg.host_id  = CAN_LOGGER_SPI_HOST;

    ret = esp_vfs_fat_sdspi_mount(
        CAN_LOGGER_MOUNT_POINT, &host, &dev_cfg, &mount_cfg, &s_card);

#endif

    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "SD mount failed: %s", esp_err_to_name(ret));
#if (CAN_LOGGER_BUS == CAN_LOGGER_BUS_SPI)
        spi_bus_free(CAN_LOGGER_SPI_HOST);
#endif
        return ret;
    }

    sdmmc_card_print_info(stdout, s_card);
    ESP_LOGI(TAG, "SD mounted at %s via %s", CAN_LOGGER_MOUNT_POINT,
             (CAN_LOGGER_BUS == CAN_LOGGER_BUS_SDIO) ? "SDIO 4-bit" : "SPI+DMA");
    return ESP_OK;
}

static void sd_unmount(void)
{
    // esp_vfs_fat_sdmmc_unmount() was deprecated in v5.x.
    // The replacement takes the mount point and card handle explicitly.
    esp_vfs_fat_sdcard_unmount(CAN_LOGGER_MOUNT_POINT, s_card);
    s_card = NULL;
#if (CAN_LOGGER_BUS == CAN_LOGGER_BUS_SPI)
    spi_bus_free(CAN_LOGGER_SPI_HOST);
#endif
}

// =============================================================================
// File helpers
// =============================================================================

static esp_err_t open_file(void)
{
    char path[64];

    if (s_time_anchored) {
        time_t t = (time_t)(s_rtc_epoch_ms / 1000);
        struct tm tm_info;
        gmtime_r(&t, &tm_info);
        snprintf(path, sizeof(path),
                 CAN_LOGGER_FILE_PREFIX "%04d%02d%02d_%02d%02d%02d_%05lu.bin",
                 tm_info.tm_year + 1900, tm_info.tm_mon + 1, tm_info.tm_mday,
                 tm_info.tm_hour, tm_info.tm_min, tm_info.tm_sec,
                 (unsigned long)s_file_index);
    } else {
        snprintf(path, sizeof(path),
                 CAN_LOGGER_FILE_PREFIX "unanchored_%05lu.bin",
                 (unsigned long)s_file_index);
    }

    s_file = fopen(path, "wb");
    if (s_file == NULL) {
        ESP_LOGE(TAG, "fopen failed: %s", path);
        return ESP_FAIL;
    }
    s_file_bytes = 0;
    ESP_LOGI(TAG, "Opened: %s", path);
    return ESP_OK;
}

static void close_file(void)
{
    if (s_file == NULL) return;
    fflush(s_file);
    fclose(s_file);
    s_file = NULL;
    ESP_LOGI(TAG, "Closed log file (%llu bytes)", (unsigned long long)s_file_bytes);
}

// =============================================================================
// Flush
// =============================================================================

static void flush(void)
{
    if (s_flush_pending == 0 || s_file == NULL) return;

    size_t written = fwrite(s_flush_buf, sizeof(CanLogRecord_t),
                            s_flush_pending, s_file);
    if (written != s_flush_pending) {
        ESP_LOGE(TAG, "Short write: %d of %d records",
                 (int)written, (int)s_flush_pending);
    }

    s_file_bytes   += written * sizeof(CanLogRecord_t);
    s_flush_pending = 0;

    if (s_file_bytes >= CAN_LOGGER_MAX_FILE_BYTES) {
        ESP_LOGI(TAG, "Rotating log file");
        close_file();
        s_file_index++;
        open_file();
    }
}

static void write_marker(uint8_t flags)
{
    if (s_flush_pending >= CAN_LOGGER_FLUSH_RECORDS) flush();
    CanLogRecord_t *rec = &s_flush_buf[s_flush_pending++];
    memset(rec, 0, sizeof(*rec));
    rec->wall_time_ms = s_time_anchored ? to_wall_ms(xTaskGetTickCount()) : 0;
    rec->id    = 0xFFFFFFFF;
    rec->flags = flags;
    flush();
}

// =============================================================================
// Logger task
// =============================================================================

static void logger_task(void *arg)
{
    ESP_LOGI(TAG, "Logger task started");

    const TickType_t flush_ticks = pdMS_TO_TICKS(CAN_LOGGER_FLUSH_INTERVAL_MS);
    const TickType_t fsync_ticks = pdMS_TO_TICKS(CAN_LOGGER_FSYNC_INTERVAL_MS);
    TickType_t last_flush = xTaskGetTickCount();
    TickType_t last_fsync = xTaskGetTickCount();

    while (s_running) {

        if (s_sync_pending) {
            s_sync_pending = false;
            write_marker(CAN_LOG_FLAG_SYNC);
            last_flush = xTaskGetTickCount();
        }

        TickType_t now     = xTaskGetTickCount();
        TickType_t elapsed = now - last_flush;
        TickType_t wait    = (s_flush_pending >= CAN_LOGGER_FLUSH_RECORDS) ? 0
                           : (elapsed < flush_ticks) ? (flush_ticks - elapsed)
                           : 0;

        CanRxEvent_t evt;
        while (s_flush_pending < CAN_LOGGER_FLUSH_RECORDS) {
            TickType_t timeout = (s_flush_pending == 0) ? wait : 0;
            if (xQueueReceive(s_log_queue, &evt, timeout) != pdTRUE) break;

            CanLogRecord_t *rec = &s_flush_buf[s_flush_pending++];
            memset(rec->_pad, 0, sizeof(rec->_pad));
            rec->wall_time_ms = s_time_anchored
                                ? to_wall_ms(evt.rx_tick)
                                : (int64_t)evt.rx_tick;
            rec->id    = evt.id;
            rec->len   = evt.len;
            rec->flags = CAN_LOG_FLAG_NORMAL;
            memcpy(rec->data, evt.data, evt.len);
            if (evt.len < 8) memset(rec->data + evt.len, 0, 8 - evt.len);
        }

        now = xTaskGetTickCount();
        if (s_flush_pending > 0 &&
            (s_flush_pending >= CAN_LOGGER_FLUSH_RECORDS ||
             (now - last_flush) >= flush_ticks)) {
            flush();
            last_flush = xTaskGetTickCount();
        }

        now = xTaskGetTickCount();
        if ((now - last_fsync) >= fsync_ticks && s_file != NULL) {
            fflush(s_file);
            last_fsync = xTaskGetTickCount();
            ESP_LOGD(TAG, "FAT synced (%llu bytes)", (unsigned long long)s_file_bytes);
        }
    }

    if (s_flush_pending > 0) flush();
    if (s_file != NULL) fflush(s_file);

    ESP_LOGI(TAG, "Logger task stopping");
    s_task_hdl = NULL;
    vTaskDelete(NULL);
}

// =============================================================================
// Public API
// =============================================================================

esp_err_t can_logger_init(void)
{
    if (s_task_hdl != NULL) return ESP_ERR_INVALID_STATE;

    s_log_queue = xQueueCreateStatic(
        CAN_LOGGER_QUEUE_DEPTH, sizeof(CanRxEvent_t),
        s_queue_storage, &s_queue_struct);
    if (s_log_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create logger queue");
        return ESP_FAIL;
    }

    esp_err_t ret = sd_mount();
    if (ret != ESP_OK) return ret;

    ret = open_file();
    if (ret != ESP_OK) { sd_unmount(); return ret; }

    s_flush_pending = 0;
    s_drops         = 0;
    s_running       = true;

    s_task_hdl = xTaskCreateStaticPinnedToCore(
        logger_task, "can_logger",
        CAN_LOGGER_TASK_STACK_SIZE, NULL,
        CAN_LOGGER_TASK_PRIORITY, s_task_stack, &s_task_buf,
        CAN_LOGGER_TASK_CORE);

    if (s_task_hdl == NULL) {
        s_running = false;
        close_file();
        sd_unmount();
        ESP_LOGE(TAG, "Failed to create logger task");
        return ESP_FAIL;
    }

    ESP_LOGI(TAG, "Logger init — queue=%d flush=%d records/%d ms fsync=%d ms",
             CAN_LOGGER_QUEUE_DEPTH, CAN_LOGGER_FLUSH_RECORDS,
             CAN_LOGGER_FLUSH_INTERVAL_MS, CAN_LOGGER_FSYNC_INTERVAL_MS);
    return ESP_OK;
}

esp_err_t can_logger_deinit(void)
{
    if (s_task_hdl == NULL) return ESP_ERR_INVALID_STATE;

    s_running = false;
    vTaskDelay(pdMS_TO_TICKS(CAN_LOGGER_FLUSH_INTERVAL_MS + 100));

    if (s_task_hdl != NULL) {
        vTaskDelete(s_task_hdl);
        s_task_hdl = NULL;
    }

    close_file();
    sd_unmount();
    return ESP_OK;
}

void can_logger_anchor_time(void)
{
    struct timeval tv;
    TickType_t tick;

    portENTER_CRITICAL(&s_time_mux);
    gettimeofday(&tv, NULL);
    tick = xTaskGetTickCount();
    portEXIT_CRITICAL(&s_time_mux);

    s_rtc_epoch_ms  = (int64_t)tv.tv_sec * 1000LL + (int64_t)(tv.tv_usec / 1000);
    s_anchor_tick   = tick;
    s_time_anchored = true;
    s_sync_pending  = true;

    ESP_LOGI(TAG, "Time anchored: epoch=%lld ms tick=%lu",
             (long long)s_rtc_epoch_ms, (unsigned long)tick);
}

void can_logger_post(const CanRxEvent_t *evt)
{
    if (xQueueSend(s_log_queue, evt, 0) != pdTRUE) {
        s_drops++;
    }
}

bool can_logger_is_running(void)
{
    return s_task_hdl != NULL && s_running;
}

uint32_t can_logger_get_drop_count(void)
{
    uint32_t d = s_drops;
    s_drops = 0;
    return d;
}