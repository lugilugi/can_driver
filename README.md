# CAN Driver (ESP-IDF TWAI) — v5.0.0

This component provides a bounded, callback-driven wrapper around the ESP-IDF
5.5.3 `esp_driver_twai` node API and the ECT2026 CAN V5 protocol bindings.

The V5 protocol is a breaking release. The DBC source of truth is
`network.dbc`, and the checked-in `network.c` / `include/network.h` files
are generated with cantools 41.4.3.

## Protocol contract

ECT2026_CAN_V5 is intentionally:

- Classical CAN only; CAN-FD frames are rejected by the driver.
- Standard 11-bit identifiers.
- Intended for a 500 kbit/s bus.
- Little-endian signal encoding.
- Bounded to eight-byte payloads.

| ID | Message | Bytes | Sender | Cycle metadata |
| ---: | --- | ---: | --- | ---: |
| 0x110 | PEDAL_STATUS | 6 | PDLB | 20 ms |
| 0x210 | AUX_COMMAND | 1 | STER | 100 ms |
| 0x310 | PACK_POWER | 4 | JBOX | 100 ms |
| 0x311 | AUX_POWER | 4 | JBOX | 100 ms |
| 0x312 | PACK_ENERGY | 5 | JBOX | 5000 ms |
| 0x400 | VEHICLE_MOTION | 8 | TELE | 100 ms |
| 0x401 | VEHICLE_TIME | 7 | TELE | 1000 ms |
| 0x410 | GPS_STATUS | 4 | TELE | 1000 ms |
| 0x411 | GPS_POSITION | 8 | TELE | 200 ms |
| 0x412 | GPS_MOTION | 4 | TELE | 200 ms |
| 0x600 | MOTOR_STATE | 8 | IMC | 50 ms |
| 0x601 | MOTOR_CURRENT | 6 | IMC | 20 ms |
| 0x602 | MOTOR_VOLTAGE | 6 | IMC | 50 ms |
| 0x603 | MOTOR_FAULTS | 8 | IMC | 100 ms |
| 0x604 | MOTOR_ESTIMATOR | 8 | IMC | 20 ms |
| 0x605 | MOTOR_PHASE_CURRENT | 6 | IMC | 0 |

A cycle value of zero means that no periodic transmission is prescribed. It
does not request continuous transmission or transmission on every scheduler
iteration. The driver never schedules protocol messages.

Application firmware owns policy such as:

- Forcing `throttle_command` to zero while `throttle_inhibit` is asserted.
- Treating `inhibit_reason` values 6–15 as reserved/invalid.
- Enforcing message freshness and sequence-counter policy.
- Interpreting GPS validity flags.
- Selecting safe outputs when safety inputs become stale.

The auxiliary command message reserves bits 2 and 7. Brake-lamp control uses
`PEDAL_STATUS.brake_active`, not an auxiliary command signal.

## Architecture

The component contains two layers:

- **`can_driver`**: TWAI setup, hardware/software filtering, bounded RX
  delivery, driver-owned TX slots, deferred bus-off recovery, and diagnostics.
- **Generated protocol bindings**: typed raw-value structures plus pack/unpack
  and physical conversion functions derived from `network.dbc`.

The driver has one singleton instance, no driver-owned task, no per-frame heap
allocation, and fixed-depth queues. Callers must stop and join all tasks using
the driver before deinitializing it.

## Requirements

- ESP-IDF >= 5.5.3.
- An ESP32-family target with the classic TWAI controller.
- A compatible external CAN transceiver for physical-bus operation.

CAN-FD-capable targets may be used, but this component remains classic-CAN-only.

## Example usage

### Initialize and filter

```c
#include "can_driver.h"
#include "network.h"

CanInitFlags_t flags = {
    .loopback = 0,
    .listen_only = 0,
};

static const uint32_t rx_ids[] = {
    NETWORK_PEDAL_STATUS_FRAME_ID,
    NETWORK_AUX_COMMAND_FRAME_ID,
};

CanFilterConfig_t filter = {
    .ids = rx_ids,
    .id_count = sizeof(rx_ids) / sizeof(rx_ids[0]),
    .software_filter = 1,
};

esp_err_t err = can_driver_init(GPIO_NUM_4,
                                GPIO_NUM_5,
                                500000,
                                flags,
                                &filter);
```

A single classic-CAN hardware mask can cover a region, not an arbitrary list.
When the list spans multiple regions, `.software_filter = 1` ensures that
only the requested IDs are delivered after hardware acceptance.

### Receive and decode

The receive buffer capacity is explicit and remains valid across calls:

```c
uint8_t rx_buf[8];
twai_frame_t rx_msg = {
    .buffer = rx_buf,
    .buffer_len = sizeof(rx_buf),
};

if (can_driver_receive(&rx_msg, sizeof(rx_buf), portMAX_DELAY) == ESP_OK) {
    if (rx_msg.header.id == NETWORK_PEDAL_STATUS_FRAME_ID &&
        rx_msg.buffer_len == NETWORK_PEDAL_STATUS_LENGTH) {
        struct network_pedal_status_t decoded = {0};

        if (network_pedal_status_unpack(&decoded,
                                        rx_msg.buffer,
                                        rx_msg.buffer_len) == 0) {
            // Apply application-level inhibit, freshness, and safety policy.
        }
    }
}
```

A received frame with a payload larger than the supplied capacity is consumed
and returns `ESP_ERR_INVALID_SIZE`; no partial payload is copied.

### Pack and transmit

Generated structures contain raw integer signal values. Use generated
`*_encode()` helpers when converting physical values:

```c
struct network_vehicle_motion_t motion = {
    .speed_kmh = 1234,       // 12.34 km/h
    .trip_distance_m = 2500, // 250.0 m
    .motion_valid =
        NETWORK_VEHICLE_MOTION_MOTION_VALID_VALID_CHOICE,
    .seq_counter = 1,
};

uint8_t payload[8];
int payload_len = network_vehicle_motion_pack(payload,
                                               &motion,
                                               sizeof(payload));

twai_frame_t tx_msg = {
    .header.id = NETWORK_VEHICLE_MOTION_FRAME_ID,
    .buffer = payload,
    .buffer_len = (size_t)payload_len,
};

if (payload_len == NETWORK_VEHICLE_MOTION_LENGTH &&
    can_driver_transmit(&tx_msg, pdMS_TO_TICKS(50)) == ESP_OK) {
    // Queued successfully.
}
```

The driver copies the descriptor and payload into a stable TX slot, so stack
storage is safe after the transmit call returns.

### Diagnostics

```c
CanStatus_t status;

if (can_driver_get_status(&status) == ESP_OK) {
    // status.error_state
    // status.tx_slots_remaining
    // status.twai_tx_queue_remaining
    // status.rx_queue_remaining
    // status.bus_error_count
    // status.rx_dropped_count
    // status.software_dropped_count
    // status.malformed_frame_count
    // status.recovery_failure_count
}
```

## Light-board freshness behavior

The rear light-board example maintains independent freshness state:

```text
AUX_COMMAND freshness
    └── turn signals, headlights, hazards, horn, wipers

PEDAL_STATUS freshness
    └── brake_active
```

Loss of the auxiliary command produces the safe auxiliary state. Loss of the
pedal-status message makes brake state invalid and selects the local
brake-inactive fail-safe. One message never refreshes the other.

## Regenerating the protocol bindings

Install the pinned generator and write output to a temporary directory:

```bash
python -m pip install cantools==41.4.3
python -m cantools generate_c_source network.dbc -o /tmp/generated
```

Copy the generated `network.c` to the repository root and
`network.h` to `include/network.h`. Do not hand-edit generated files.

Run the repository checks:

```bash
python tools/check_network_artifacts.py
python tools/check_active_references.py
```

The artifact check validates the DBC contract and detects drift in both
generated files. Cantools embeds a generation timestamp, so only that
timestamp is normalized during comparison.

## CI and release

CI validates the DBC, generated artifacts, codec tests, and both ESP-IDF
examples for ESP32-C3 under ESP-IDF 5.5.3.

The v5.0.0 release is represented by:

- Git tag: `v5.0.0`
- ESP Component Registry version: `5.0.0`
- GitHub release: `v5.0.0`
- Release notes describing the CAN V5 breaking migration

The component manifest intentionally does not maintain a separate checked-in
component version. The publishing workflow derives the Registry version from
the Git tag.
