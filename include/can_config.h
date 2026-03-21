#pragma once

// =============================================================================
// can_config.h — All compile-time constants for the CAN component.
// Tune these to match your hardware and application requirements.
// =============================================================================

// -----------------------------------------------------------------------------
// TX Pool
// -----------------------------------------------------------------------------
#define CAN_TX_POOL_SIZE        8
#define CAN_TX_QUEUE_DEPTH      8
#define CAN_TX_RETRY_COUNT      3

// -----------------------------------------------------------------------------
// RX Queue
// Depth of the static ISR -> manager task queue.
// -----------------------------------------------------------------------------
#define CAN_RX_QUEUE_DEPTH      16

// -----------------------------------------------------------------------------
// Manager Task
// -----------------------------------------------------------------------------
#define CAN_MANAGER_TASK_PRIORITY   5
#define CAN_MANAGER_TASK_STACK_SIZE 2048
#define CAN_MANAGER_TASK_CORE       tskNO_AFFINITY

// -----------------------------------------------------------------------------
// Logger Queue
//
// Separate from CAN_RX_QUEUE_DEPTH — this is the manager -> logger task queue.
// Sized to absorb SD write latency spikes without dropping frames.
// Consumer SD cards can stall 50-200 ms; at 1 Mbit/s a saturated CAN bus
// produces ~80 frames/s, so 256 slots covers ~3 seconds of lag.
// RAM cost: 256 x 18 bytes ~= 4.5 KB.
// -----------------------------------------------------------------------------
#define CAN_LOGGER_QUEUE_DEPTH      256

// -----------------------------------------------------------------------------
// Logger Task
// -----------------------------------------------------------------------------
#define CAN_LOGGER_TASK_PRIORITY    4
#define CAN_LOGGER_TASK_STACK_SIZE  4096
#ifdef CONFIG_FREERTOS_UNICORE
#  define CAN_LOGGER_TASK_CORE      0   // C3 is single-core, core 1 doesn't exist
#else
#  define CAN_LOGGER_TASK_CORE      1   // opposite core to manager on dual-core
#endif

// -----------------------------------------------------------------------------
// Logger flush policy
//
// Each CanLogRecord_t is 32 bytes -> 16 records per 512-byte SD sector.
// CAN_LOGGER_FLUSH_RECORDS must be a multiple of 16 for sector-aligned writes.
// -----------------------------------------------------------------------------
#define CAN_LOGGER_FLUSH_RECORDS        64      // must be multiple of 16
#define CAN_LOGGER_FLUSH_INTERVAL_MS    2000
#define CAN_LOGGER_FSYNC_INTERVAL_MS    10000

// -----------------------------------------------------------------------------
// Log file management
// -----------------------------------------------------------------------------
#define CAN_LOGGER_MOUNT_POINT      "/sdcard"
#define CAN_LOGGER_FILE_PREFIX      "/sdcard/can_"
#define CAN_LOGGER_MAX_FILE_BYTES   (100ULL * 1024 * 1024)

// -----------------------------------------------------------------------------
// SD bus interface selection
//
//   CAN_LOGGER_BUS_SDIO — 4-bit SDIO via SDMMC host peripheral.
//                         NOT available on ESP32-C3 (no SDMMC host).
//                         Available on: ESP32, ESP32-S3, ESP32-P4.
//
//   CAN_LOGGER_BUS_SPI  — 1-bit SPI. Works on all targets including C3.
//                         DMA via SPI_DMA_CH_AUTO. Fine for CAN logging rates.
//
// A compile-time error is raised if SDIO is selected on a target without
// the SDMMC peripheral.
// -----------------------------------------------------------------------------
#define CAN_LOGGER_BUS_SDIO   0
#define CAN_LOGGER_BUS_SPI    1
#define CAN_LOGGER_BUS        CAN_LOGGER_BUS_SPI   // C3 only supports SPI

// SDIO pins — only relevant on targets with SDMMC host (ESP32 slot 2,
// or flexible routing on S3/P4). Ignored when CAN_LOGGER_BUS_SPI is selected.
#define CAN_LOGGER_SDIO_CLK     14
#define CAN_LOGGER_SDIO_CMD     15
#define CAN_LOGGER_SDIO_D0      2
#define CAN_LOGGER_SDIO_D1      4
#define CAN_LOGGER_SDIO_D2      12
#define CAN_LOGGER_SDIO_D3      13

// SPI pins — adjust to match your board wiring.
#define CAN_LOGGER_SPI_HOST     SPI2_HOST
#define CAN_LOGGER_SPI_CLK      15
#define CAN_LOGGER_SPI_MOSI     2
#define CAN_LOGGER_SPI_MISO     12
#define CAN_LOGGER_SPI_CS       4