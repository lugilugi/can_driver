# CAN Driver (ESP-IDF TWAI)

This component provides a static-allocation CAN (TWAI) stack for ESP-IDF with a
clear separation between hardware access, message dispatch, and application
state. It is designed for deterministic behavior on embedded targets, especially
single-core ESP32-C3 systems.

## Requirements

- ESP-IDF 5.5.3 or newer.
- ESP32 target with TWAI controller (ESP32, ESP32-S3, ESP32-C3, etc.).
- External CAN transceiver and correct wiring.
- Classic CAN frames (DLC 0-8). Transmit path uses standard (11-bit) IDs.

## Architecture

The component is split into three layers plus optional services:

- Driver layer: [include/can_driver.h](include/can_driver.h)
   - Owns TWAI node lifecycle, ISR callbacks, static TX pool, and RX queue.
   - Payload-agnostic. Frames are copied into a queue as `CanRxEvent_t`.
- Manager layer: [include/can_manager.h](include/can_manager.h)
   - Dedicated FreeRTOS task drains the RX queue.
   - Dispatches frames to state structs and handles bus-off recovery.
   - Owns optional logger and USB forward services when enabled.
- State layer: [include/can_state.h](include/can_state.h)
   - Global, latest-value structs that the application reads.
   - Each struct includes `last_rx_tick` for staleness checks.

Optional services:

- SD logger: [include/can_logger.h](include/can_logger.h)
- USB forwarder: [include/can_usb_forward.h](include/can_usb_forward.h)

## Message flow

Transmit:

1. Application calls `can_driver_transmit(id, data, len)`.
2. Driver claims a TX pool slot and queues a frame to TWAI.
3. ISR releases the slot when TX completes.

Receive:

1. TWAI ISR captures the frame and timestamp, enqueues `CanRxEvent_t`.
2. Manager task drains the queue and updates state.
3. Application reads `g_can_*` state and checks `last_rx_tick` for staleness.

## Quick start (driver-only)

Use this when you want to handle frames directly without the manager task.

```c
#include "can_driver.h"
#include "can_payloads.h"
#include "esp_err.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

void app_main(void)
{
      CanInitFlags_t flags = {0};
      ESP_ERROR_CHECK(can_driver_init(GPIO_NUM_4, GPIO_NUM_5, 500000, flags));

      uint8_t payload[2] = {0x12, 0x34};
      ESP_ERROR_CHECK(can_driver_transmit(0x123, payload, sizeof(payload)));

      CanRxEvent_t evt = {0};
      esp_err_t ret = can_driver_receive(&evt, pdMS_TO_TICKS(100));
      if (ret == ESP_OK) {
            // evt.id, evt.data[0..evt.len-1], evt.rx_tick
      }

      ESP_ERROR_CHECK(can_driver_deinit());
}
```

For single-board bring-up, use loopback + self-test flags:

```c
CanInitFlags_t flags = { .loopback = 1, .self_test = 1 };
```

## Manager and state usage

Use this for typical application flow where the manager updates global state.

```c
#include "can_driver.h"
#include "can_manager.h"
#include "can_payloads.h"
#include "can_state.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static bool is_stale(TickType_t last_rx_tick, uint32_t max_age_ms)
{
      if (last_rx_tick == 0) return true;
      return (xTaskGetTickCount() - last_rx_tick) > pdMS_TO_TICKS(max_age_ms);
}

void app_main(void)
{
      CanInitFlags_t flags = { .loopback = 1, .self_test = 1 };
      ESP_ERROR_CHECK(can_driver_init(GPIO_NUM_4, GPIO_NUM_5, 500000, flags));
      ESP_ERROR_CHECK(can_manager_init());

      if (!is_stale(g_can_pedal.last_rx_tick, 250)) {
            float pct = PedalPayload_getThrottle(&g_can_pedal.data);
            (void)pct;
      }

      EnergyPayload energy = {0};
      can_state_get_energy_raw(&energy);
      double joules = EnergyPayload_getJoules_780(&energy);
      (void)joules;
}
```

## API details

### Driver lifecycle

- `can_driver_init(tx_io, rx_io, baud, flags)`
   - Sets up TWAI, RX queue, TX pool, and ISR callbacks.
   - Rejects incompatible flags (loopback + listen_only).
- `can_driver_deinit()`
   - Disables TWAI and deletes the RX queue.

Call order:

1. `can_driver_init(...)`
2. `can_manager_init()` (optional)
3. Application tasks
4. `can_manager_deinit()` (if used)
5. `can_driver_deinit()`

### Transmit

- `can_driver_transmit(id, data, len)`
   - Non-blocking, uses a static TX pool.
   - Returns `ESP_ERR_NO_MEM` if the pool is exhausted.
   - Returns `ESP_ERR_INVALID_ARG` if `len > 8` or `data == NULL` when `len > 0`.
   - Uses standard 11-bit IDs for transmit (`ide = false`).

### Receive

- `can_driver_receive(evt, timeout_ticks)`
   - Blocks until a frame arrives or timeout expires.
   - Returns `ESP_ERR_TIMEOUT` when the queue is empty.
   - Intended for the manager task or low-level examples.

### Init flags (`CanInitFlags_t`)

- `loopback`: route TX frames back into RX. Forces `self_test = 1`.
- `self_test`: disable ACK checking for single-node testing.
- `listen_only`: passive sniffing, no dominant bits or ACKs.
- `no_rtr`: hardware drops RTR frames.

### Filters

Default filter at init is accept-all standard ID only (extended IDs are
ignored unless you configure an extended filter).

Manual configuration:

- `can_driver_apply_single_filter(id, mask, is_ext)`
- `can_driver_apply_dual_filter(id1, mask1, id2, mask2, is_ext)`

Automatic configuration from an allowlist:

- `can_driver_apply_single_filter_auto(ids, count, is_ext)`
- `can_driver_apply_dual_filter_auto(ids, count, is_ext)`

Notes:

- Filters are applied via a disable-config-enable cycle. Any in-flight TX is
   aborted, so configure filters when the bus is idle.
- `*_auto` with `count == 0` accepts all IDs.
- `can_driver_apply_dual_filter_auto` uses a stack scratch buffer and falls
   back to single-filter mode when `count > 128`.

### Bus-off recovery

- The driver sets a bus-off flag from ISR context.
- The manager polls the flag and calls `can_driver_recover()`.
- If you do not run the manager, poll `can_driver_is_bus_off()` in a task and
   call `can_driver_recover()` yourself (never from ISR).

### Diagnostics

- `can_driver_get_pool_used()`
- `can_driver_get_isr_rx_calls()`
- `can_driver_get_isr_rx_fail()`
- `can_driver_get_isr_rx_dropped()`
- `can_driver_reset_isr_counters()`

## State layer and staleness

Each `g_can_*` state struct includes `last_rx_tick`. The application owns the
definition of staleness. Example:

```c
TickType_t age = xTaskGetTickCount() - g_can_pedal.last_rx_tick;
if (age > pdMS_TO_TICKS(50)) {
      emergency_stop_motor();
}
```

Energy payloads are 5 bytes and must be accessed through synchronized helpers:

```c
EnergyPayload energy = {0};
can_state_get_energy_raw(&energy);
```

## Payloads and catalog

- [include/can_payloads.h](include/can_payloads.h) defines the wire format for
   every frame (IDs, payload structs, and encode/decode helpers).
- [include/can_message_catalog.h](include/can_message_catalog.h) is the
   routing catalog. Copy-routed frames are listed in
   `CAN_MESSAGE_COPY_ROUTE_TABLE`, and special routes are listed in
   `CAN_MESSAGE_SPECIAL_ROUTE_TABLE`.

## Optional services

### SD logger

Enable in [include/can_config.h](include/can_config.h):

```c
#define CAN_LOGGER_ENABLED 1
```

Pull SD dependencies in your project CMake configuration:

```
idf.py -DCAN_LOGGER_PULL_DEPS=ON build
```

Usage:

1. `can_logger_init()` to mount SD and start the logger task.
2. `can_logger_anchor_time()` after RTC is ready (once).
3. Logger runs automatically when the manager posts frames.

The log record format is documented in [include/can_logger.h](include/can_logger.h).

### USB forwarder

Enable in [include/can_config.h](include/can_config.h):

```c
#define CAN_USB_FORWARD_ENABLED 1
```

The bridge reads and writes `candump`-style lines. For ESP32-C3, set the
console channel to USB Serial/JTAG in menuconfig as described in
[include/can_usb_forward.h](include/can_usb_forward.h).

## Self-test

The self-test uses loopback + self-test mode and runs without external
hardware. It must be called before the driver or manager is initialized.

```c
if (!can_selftest_run()) {
      // handle failure
}
```

See [include/can_selftest.h](include/can_selftest.h) and
[examples/selftest](examples/selftest).

## Examples

- [examples/basic_test](examples/basic_test) covers driver init, filters, TX/RX,
   diagnostics, and deinit.
- [examples/manager_state](examples/manager_state) shows manager and state usage.
- [examples/selftest](examples/selftest) runs the built-in self-test.
- [examples/unit_test](examples/unit_test) runs Unity tests on target.

## Tests

The Unity tests live under [test](test). See [test/README.md](test/README.md)
for recommended flows.

## Configuration reference

All component compile-time constants are in
[include/can_config.h](include/can_config.h). Common knobs:

- `CAN_TX_POOL_SIZE`, `CAN_TX_QUEUE_DEPTH`, `CAN_TX_RETRY_COUNT`
- `CAN_RX_QUEUE_DEPTH`
- `CAN_MANAGER_TASK_PRIORITY`, `CAN_MANAGER_TASK_STACK_SIZE`
- `CAN_LOGGER_ENABLED`, `CAN_USB_FORWARD_ENABLED`
- Logger SD settings and SPI/SDIO pin selection

## Adding a new CAN ID

1. Add the ID and payload type in [include/can_payloads.h](include/can_payloads.h).
2. Add state struct fields in [include/can_state.h](include/can_state.h) and
    define them in `can_state.c`.
3. Add routing in [include/can_message_catalog.h](include/can_message_catalog.h).
4. Update dispatch logic in `can_manager.c` if it is a special route.
5. Add or update tests under [test](test).
6. Update example allowlists if needed.

The catalog enforces compile-time duplicate ID checks via the manager build.