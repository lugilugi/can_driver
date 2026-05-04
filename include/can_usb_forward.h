#pragma once

#include "can_driver.h"
#include "esp_err.h"

// =============================================================================
// can_usb_forward.h — Bidirectional CAN <-> USB serial bridge.
//
// RX (CAN -> phone): candump format, one line per frame.
//   (seconds.ms) can0 ID#HEXDATA\n
//   e.g. (1234.567) can0 110#DEADBEEF0102
//
// TX (phone -> ESP32): same candump format, or shorthand without timestamp.
//   Full:      (1234.567) can0 110#DEADBEEF\n   (timestamp is ignored)
//   Shorthand: 110#DEADBEEF\n
//
// The TX parser is intentionally lenient — it only needs the ID#DATA portion.
// Anything before the last space-delimited token containing '#' is ignored,
// so both formats work without the phone needing to generate a timestamp.
//
// Two tasks run internally:
//   rx_task  — drains the CAN RX queue, writes candump lines to stdout
//   tx_task  — reads lines from stdin, parses and calls can_driver_transmit()
//
// menuconfig requirement on ESP32-C3:
//   Component config > ESP System Settings > Channel for console output
//     > USB Serial/JTAG Controller
//
// Enable / disable:
//   Set CAN_USB_FORWARD_ENABLED in can_config.h.
//   When 0, every call site in can_manager.c compiles away to nothing —
//   no #ifdefs needed anywhere else.
// =============================================================================

// -----------------------------------------------------------------------------
// Task configuration — only meaningful when enabled, but always defined so
// the .c file compiles without errors regardless of the enabled state.
// -----------------------------------------------------------------------------
#define CAN_USB_FWD_QUEUE_DEPTH      64
#define CAN_USB_FWD_RX_TASK_PRIORITY 6
#define CAN_USB_FWD_TX_TASK_PRIORITY 6
#define CAN_USB_FWD_TASK_STACK_SIZE  2048
#define CAN_USB_FWD_TASK_CORE        tskNO_AFFINITY

#if CAN_USB_FORWARD_ENABLED

// Start both the RX and TX tasks.
esp_err_t can_usb_forward_init(void);

// Stop both tasks.
esp_err_t can_usb_forward_deinit(void);

// Anchor wall-clock time for RX timestamps.
// Pass RTC epoch in ms since Unix epoch. If not called, raw ticks are used.
void can_usb_forward_anchor_time(int64_t rtc_epoch_ms);

// Post a received CAN frame to the RX output queue.
// Called by can_manager after dispatch(). Non-blocking, drops if full.
void can_usb_forward_post(const CanRxEvent_t *evt);

// Returns frames dropped since last call (read-and-clear).
uint32_t can_usb_forward_get_drop_count(void);

#else

// Stubbed to nothing when disabled — all call sites compile away cleanly.
static inline esp_err_t can_usb_forward_init(void)                        { return ESP_OK; }
static inline esp_err_t can_usb_forward_deinit(void)                      { return ESP_OK; }
static inline void      can_usb_forward_anchor_time(int64_t rtc_epoch_ms) { (void)rtc_epoch_ms; }
static inline void      can_usb_forward_post(const CanRxEvent_t *evt)     { (void)evt; }
static inline uint32_t  can_usb_forward_get_drop_count(void)              { return 0; }

#endif // CAN_USB_FORWARD_ENABLED