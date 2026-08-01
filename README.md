# CAN Driver (ESP-IDF TWAI)

This component provides a lightweight CAN (TWAI) wrapper for ESP-IDF, specifically tailored to work seamlessly with `cantools` auto-generated DBC parsers. It is built on the modern ESP-IDF 5.5 `esp_driver_twai` node API (callback-driven, no polling).

## Architecture

This component consists of a thin driver layer that abstracts away the complex TWAI setup, and generated message parsers based on your DBC specification:
- **`can_driver`**: A wrapper to safely initialize the TWAI peripheral, queue transmissions, and receive messages cleanly. Features:
  - **Hardware ID-list filtering**: accept a list of IDs; non-matching frames never wake the CPU. The driver computes the smallest maskable region containing the list.
  - **Software filtering (opt-in)**: `.software_filter` discards frames the hardware region over-accepts, so only the listed IDs are delivered.
  - **Callback-driven RX**: matching frames wake the CPU and are queued by the driver — no polling, no busy loops.
  - **Automatic bus-off recovery** via the driver's state-change callback.
  - **Diagnostics**: error counters, queue depth, bus error count (`can_driver_get_status`).
  - **Safe TX slots**: the driver copies frames into its own TX pool, so callers can use stack-allocated frames and buffers.
- **`network.c` / `network.h`**: Auto-generated from `network.dbc` using `cantools`. This provides typed structures and unpack/pack functions for handling specific CAN network messages.

## Requirements

- **ESP-IDF**: >= 5.5.3 (uses the `esp_driver_twai` node API)
- **Hardware**: Any ESP32 series chip that features the TWAI (Two-Wire Automotive Interface) controller (e.g. ESP32, ESP32-S2, ESP32-S3, ESP32-C3).
- **External Transceiver**: A 3.3V compatible CAN transceiver is required to connect to a physical CAN bus.

## Example Usage

See `examples/dbc_usage` for a complete working example (multi-ID filtering, DBC decode, TX, diagnostics).

### 1. Initializing the Driver

```c
#include "can_driver.h"

// Set up the ESP32 TWAI driver using your board's TX and RX pins
// e.g., GPIO 4 for TX, GPIO 5 for RX at 500 kbps (ESP32-C3)
CanInitFlags_t flags = { .loopback = 0, .listen_only = 0 };

// Accept only the frames this node cares about (here: PEDAL and AUX_CTRL).
// Other frames are dropped in hardware and never wake the CPU.
static const uint32_t rx_ids[] = { NETWORK_PEDAL_FRAME_ID, NETWORK_AUX_CTRL_FRAME_ID };
CanFilterConfig_t filter = { .ids = rx_ids, .id_count = 2 };

// NULL or CAN_FILTER_ACCEPT_ALL() accepts everything
esp_err_t err = can_driver_init(GPIO_NUM_4, GPIO_NUM_5, 500000, flags, &filter);
if (err != ESP_OK) {
    // Handle error
}
```

#### How the filter behaves

At boot the driver logs what it configured:

| Log line | Meaning |
|----------|---------|
| `filter: N IDs accepted exactly (code 0x..., mask 0x...)` | The list forms one exact maskable region — optimal |
| `filter: ... also accepts N extra IDs — set .software_filter to discard them` | One region covering the list plus N unwanted IDs. The CPU will wake for those; set `.software_filter` to prevent delivery |

```c
// Exact delivery: the driver discards over-accepted frames in the ISR.
// Only the listed IDs are ever delivered. (Power note: the CPU still wakes
// for every frame the hardware region accepts — this is an exactness fix.)
CanFilterConfig_t filter = {
    .ids = rx_ids,
    .id_count = 2,
    .software_filter = 1,
};
```

> [!NOTE]
> The acceptance filter can only target one ID format at a time: standard *or* extended (`filter.extd`). With accept-all + `.software_filter`, frames of the other format are also discarded.

> [!WARNING]
> Be sure to specify the correct `GPIO_NUM_xx` pins corresponding to your specific ESP32 target. For example, ESP32-C3 has no `GPIO_NUM_22` (GPIO range is 0..21, with 18/19 used for USB) — the examples use GPIO 4/5 for CAN TX/RX. Make sure to update the pin constants in your code.

### 2. Receiving and Decoding Frames

Using the included auto-generated `cantools` code (`network.h`), parsing frames becomes straightforward:

```c
#include "network.h"

uint8_t rx_buf[8];
twai_frame_t rx_msg = { .buffer = rx_buf, .buffer_len = sizeof(rx_buf) };

// Wait for a message (blocks until one is received)
if (can_driver_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
    switch (rx_msg.header.id) {
        case NETWORK_PEDAL_FRAME_ID: {
            struct network_pedal_t decoded_pedal;

            // Unpack the raw data payload into the strongly typed struct
            network_pedal_unpack(&decoded_pedal, rx_msg.buffer, rx_msg.buffer_len);

            printf("Received Pedal Throttle: %d\n", decoded_pedal.throttle_raw);
            break;
        }
        default:
            // If the driver logged an over-acceptance warning at boot,
            // handle (or discard) the extra IDs here.
            break;
    }
}
```

### 3. Transmitting Frames

```c
// Build a frame with a cantools-generated pack() function
struct network_dash_stat_t dash_stat = { 0 };
network_dash_stat_init(&dash_stat);

uint8_t payload[8];
int payload_len = network_dash_stat_pack(payload, &dash_stat, sizeof(payload));

// The driver copies the frame into its own TX slots, so a stack buffer is safe.
// If you leave .header.dlc at 0 it is derived from buffer_len.
twai_frame_t tx_msg = {
    .header.id = NETWORK_DASH_STAT_FRAME_ID,
    .buffer = payload,
    .buffer_len = payload_len,
};

// Transmit onto the CAN bus, timeout of 50ms if TX queue is full
if (can_driver_transmit(&tx_msg, pdMS_TO_TICKS(50)) == ESP_OK) {
    printf("Successfully sent frame\n");
}
```

### 4. Diagnostics

```c
CanStatus_t status;
if (can_driver_get_status(&status) == ESP_OK) {
    // status.error_state (active/warning/passive/bus_off)
    // status.tx_error_count, status.rx_error_count
    // status.tx_queue_remaining, status.rx_queue_remaining
    // status.bus_error_count, status.rx_dropped_count
    // status.software_dropped_count (frames discarded by .software_filter)
}
```

`bus_error_count` counts `on_error` events (bit errors, form errors, stuff errors, etc.) since the node was enabled; `rx_dropped_count` counts frames lost because the RX queue (depth 8) was full; `software_dropped_count` counts frames discarded by the software filter. All are cheap counters you can poll from a low-rate monitoring task.

### 5. Testing in Loopback (no transceiver needed)

```c
CanInitFlags_t flags = { .loopback = 1, .listen_only = 0 };
// On classic chips (ESP32-C3) loopback requires self-test mode, which the
// driver enables automatically when .loopback is set.
can_driver_init(GPIO_NUM_4, GPIO_NUM_5, 500000, flags, NULL);
```

In loopback mode, every frame you transmit is also received by the same node — a quick way to verify the driver and the DBC pack/unpack round-trip on the bench. A receiving task (see section 2) will see the frame you sent in section 3.

## Power Notes

- The acceptance filter is the main power lever: only matching frames generate an interrupt, so the CPU stays asleep between relevant frames even on a busy bus.
- RX is interrupt-driven (no polling task in the driver), and the driver spawns no background tasks.
- While the node is enabled it holds a PM lock (APB frequency / light-sleep constraint on some targets). Call `can_driver_deinit()` before entering light sleep to release it, then `can_driver_init()` again to resume; the transceiver standby pin (e.g. STB) can be driven alongside this for further savings.

```c
// Before entering light sleep
can_driver_deinit();
gpio_set_level(STB_GPIO, 1);  // put the transceiver in standby

// ... sleep ...

// On wake
gpio_set_level(STB_GPIO, 0);
can_driver_init(GPIO_NUM_4, GPIO_NUM_5, 500000, flags, &filter);
```

## CAN ID Convention (mask-friendly)

The driver is ID-agnostic, but a hardware mask can only express "these bit positions are fixed, the rest are free". To keep filtering exact, allocate IDs as fixed bit fields with power-of-two-aligned widths. The `network.dbc` in this repo follows this convention:

| Bits | Field | Meaning |
|------|-------|---------|
| 10..8 | Class | 0x0 reserved · 0x1 pedal/safety · 0x2 aux · 0x3 power · 0x4 dashboard · 0x5..0x7 future |
| 7..0  | Message index | 256 per class, allocate contiguously |

Rules for adding IDs:
- New messages stay in their sender's class with the next free index (e.g. a new power frame → `0x313`).
- A new domain gets the next unused class (never reuse one).
- Class 0x0 stays reserved so no live traffic collides with filter-region spill.

Why this works:
- **Receive a whole class** → exact mask `code = C<<8`, `mask = 0x700`, regardless of how many messages are added later.
- **Receive one message** → exact mask with that single ID.
- **Receive across classes** (e.g. pedal + power) → not expressible with one hardware filter; use `.software_filter` (or accept-all + your own dispatch).

## Modifying the DBC

If you modify `network.dbc` to define new network messages, you must regenerate the C parser code. You can do this by using the Python `cantools` package:

```bash
cantools generate_c_source network.dbc
```

This updates `network.c` and `network.h` with your new frames and signals. Since this repo keeps `network.h` in `include/`, generate into a temp directory and copy the files over (as `include/network.h` and `network.c`), or adapt the command. The checked-in files are generated with cantools 41.4.3.

## GitHub Actions / CI

Two GitHub Actions are provided:
- **`ci.yml`**: builds both examples (`dbc_usage`, `light_board`) for the `esp32c3` target on every push/PR — a compile gate for the driver and example code.
- **`publish.yml`**: builds the examples for `esp32c3` and publishes the component to the ESP Component Registry when a `v*.*.*` tag is pushed.

The examples target ESP32-C3 (CAN TX = GPIO 4, RX = GPIO 5, plus the light_board wiring). If you intend to use this on other chips (like ESP32-S3 or ESP32), modify the pins in your application to match your hardware layout.
