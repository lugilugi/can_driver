#pragma once

#include "esp_err.h"
#include "can_driver.h"
#include <stdbool.h>
#include <stdint.h>

// =============================================================================
// can_logger.h — SD card frame logger. Self-contained, no changes to the
// driver or the CanRxEvent_t wire format.
//
// The existing CanRxEvent_t::rx_tick (TickType_t, ms resolution) is converted
// to absolute wall-clock time inside this module using a one-time RTC anchor
// stored at boot. Nothing upstream changes.
//
// Pipeline:
//   can_manager  --(can_logger_post)--> logger queue
//                                           |
//                                      logger task
//                                           |
//                                    sector-aligned buffer
//                                           |
//                                  fwrite() -> FatFs -> SDMMC DMA -> SD card
//
// Binary record format (32 bytes, little-endian):
//   Offset  Size  Field
//   0       8     wall_time_ms  ms since Unix epoch (UTC)
//   8       4     id            CAN arbitration ID
//   12      8     data          frame payload
//   20      1     len           payload length (0-8)
//   21      1     flags         CAN_LOG_FLAG_*
//   22      10    reserved      always zero
//
// Usage:
//   1. can_logger_init()         -- mount SD, open file, start task
//   2. can_logger_anchor_time()  -- call once after your RTC is ready
//   3. can_logger_post(&evt)     -- called by can_manager for every frame
//   4. can_logger_deinit()       -- flush, close, unmount
// =============================================================================

#include "can_config.h"

#if CAN_LOGGER_ENABLED

// Initialise SDMMC, mount FAT filesystem, open first log file, start task.
esp_err_t can_logger_init(void);

// Flush, close log file, stop task, unmount SD card.
esp_err_t can_logger_deinit(void);

// Anchor logger time to the RTC.
void can_logger_anchor_time(void);

// Post a received frame to the logger queue. Non-blocking.
void can_logger_post(const CanRxEvent_t *evt);

// Returns true if the logger task is running and the SD card is mounted.
bool can_logger_is_running(void);

// Returns frames dropped since last call (read-and-clear).
uint32_t can_logger_get_drop_count(void);

#else

static inline esp_err_t can_logger_init(void)             { return ESP_OK; }
static inline esp_err_t can_logger_deinit(void)           { return ESP_OK; }
static inline void      can_logger_anchor_time(void)      { }
static inline void      can_logger_post(const CanRxEvent_t *evt) { (void)evt; }
static inline bool      can_logger_is_running(void)       { return false; }
static inline uint32_t  can_logger_get_drop_count(void)   { return 0; }

#endif // CAN_LOGGER_ENABLED