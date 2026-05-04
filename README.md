## CAN Message Flow

### Sending a Message
1. **Application** calls `can_driver_transmit(id, data, len)`.
2. **Driver Layer**:
   - Claims a slot from the static TX pool.
   - Prepares the CAN frame and queues it for transmission.
   - TWAI hardware transmits the frame.
   - On TX completion, ISR marks the slot as free.

### Receiving a Message
1. **TWAI hardware** receives a CAN frame.
2. **Driver Layer**:
   - ISR timestamps the frame and writes it to the static RX queue.
   - Sets bus-off flag if an error occurs.
3. **Manager Layer**:
   - FreeRTOS manager task drains the RX queue.
   - Decodes the payload and updates the relevant global state struct (e.g., `g_can_pedal`).
   - Updates `last_rx_tick` for staleness checks.

---

## CAN Communication Subsystem

This component provides a robust, statically-allocated CAN (TWAI) communication stack for the ESP32-C3. It follows a Producer-Consumer model where hardware events are decoupled from application logic to ensure timing stability in a vehicle environment.

---

### 🏗️ Architecture Overview

The system is divided into three distinct layers to ensure that a failure in one does not compromise the entire stack:

- **Driver Layer (`can_driver`)**: Hardware Abstraction Layer (HAL). Manages the TWAI peripheral, static TX pool, and high-priority ISRs.
- **Manager Layer (`can_manager`)**: The "Glue" layer. Dedicated FreeRTOS task drains the RX queue, handles bus-off recovery, and dispatches data to the state layer.
- **State Layer (`can_state`)**: The "Global Mailbox." Stores the latest decoded telemetry. This is the only layer the main application should interact with.

---

### 🛠️ API Reference

#### 1. Driver Layer (`can_driver.h`)
Payload-agnostic hardware control.

| Function                | Description |
|------------------------|-------------|
| `can_driver_init(tx_io, rx_io, baud, flags)` | Configures GPIOs, bitrate, and driver flags. Initializes static TX pool and RX queue. |
| `can_driver_deinit()`         | Disables TWAI node and frees hardware resources. |
| `can_driver_transmit(id, data, len)` | Thread-safe. Claims a slot from the static pool and queues a frame for transmission. Non-blocking. |
| `can_driver_receive(evt, timeout)`    | Usually used by the manager task (also used in low-level examples). Blocks until a frame arrives in the RX queue. |
| `can_driver_recover()`         | Triggers TWAI bus-off recovery sequence. Must be called from task context. |
| `can_driver_is_bus_off()`      | Returns true if hardware has entered bus-off state due to electrical errors. |
| `can_driver_apply_single_filter*` / `can_driver_apply_dual_filter*` | Configures TWAI hardware mask filters (manual and auto modes). |

#### 2. Manager Layer (`can_manager.h`)
Lifecycle management for the background dispatcher.

| Function                | Description |
|------------------------|-------------|
| `can_manager_init()`         | Spawns the manager task with static stack allocation. Must be called after the driver is up. |
| `can_manager_deinit()`       | Signals the manager task to stop and deletes the task handle. |

#### 3. State Layer (`can_state.h`)
Data access for the vehicle application.

This layer exposes Global State Structs (e.g., `g_can_pedal`, `g_can_energy`).

| Global Instance   | Source ID | Contained Data |
|-------------------|-----------|---------------|
| `g_can_pedal`     | 0x110     | Raw `PedalPayload` plus `last_rx_tick` |
| `g_can_aux`       | 0x210     | Raw `AuxControlPayload` plus `last_rx_tick` |
| `g_can_pwr780`    | 0x310     | Raw `PowerPayload` plus `last_rx_tick` |
| `g_can_pwr740`    | 0x311     | Raw `PowerPayload` plus `last_rx_tick` |
| `g_can_energy`    | 0x312     | Raw 5-byte `EnergyPayload` plus `last_rx_tick` |
| `g_can_dash`      | 0x400     | Raw dash bytes plus `last_rx_tick` |

---

### ⚠️ Vital Safety Protocols

#### Staleness Checks
Every state struct includes a `last_rx_tick`. The application must verify data freshness before taking action.

```c
// Example: Safety interlock in the Motor Task
TickType_t age = xTaskGetTickCount() - g_can_pedal.last_rx_tick;
if (age > pdMS_TO_TICKS(50)) {
    emergency_stop_motor(); // Pedal node has gone silent
}
```

#### Thread Safety (Energy Payload Access)
The energy state stores a 5-byte payload and should be accessed through the synchronized helper APIs:

```c
// Thread-safe read of energy payload
EnergyPayload energy_raw = {0};
can_state_get_energy_raw(&energy_raw);
double current_joules = EnergyPayload_getJoules_780(&energy_raw);
```

#### Bus-Off Recovery
The manager task handles recovery automatically. If `can_driver_is_bus_off()` is true, the manager will initiate recovery, which requires 129 occurrences of 11 consecutive recessive bits on the bus to complete.

---

### 🧪 Self-Test

A built-in self-test (`can_selftest`) uses TWAI loopback and self-test mode to verify every layer without external hardware. See [include/can_selftest.h](include/can_selftest.h) and [examples/selftest/](examples/selftest/) for details.

---

### ✅ ESP-IDF Unity Unit Tests

This component includes ESP-IDF target-based Unity tests in [test/](test/) and a ready-to-run test app in [examples/unit_test/](examples/unit_test/).

- Recommended flow (in-repo): open [examples/unit_test/](examples/unit_test/), build/flash/monitor, press Enter, then run `[can]` tests.
- Advanced flow (ESP-IDF unit-test-app): use [test/README.md](test/README.md) if you want to aggregate this component into Espressif's shared unit-test-app.
- Current tests validate payload helpers/wire layout, catalog integrity, state accessor synchronization, and driver pre-init API contracts.

---

### ➕ How To Add A New CAN ID

Use this sequence to avoid drift:

1. Add the new ID and payload type in [include/can_payloads.h](include/can_payloads.h).
2. Add runtime state in [include/can_state.h](include/can_state.h) and define it in `can_state.c`.
3. Add catalog routing metadata in [include/can_message_catalog.h](include/can_message_catalog.h):
   - Copy-route messages go in `CAN_MESSAGE_COPY_ROUTE_TABLE`.
   - Custom-handled messages go in `CAN_MESSAGE_SPECIAL_ROUTE_TABLE` and handler logic in `can_manager.c`.
4. Add/extend tests under [test/](test/):
   - payload behavior (`test_can_payloads.c`)
   - catalog integrity (`test_can_catalog.c`)
   - state accessors (`test_can_state.c`) as needed
5. If this ID is used in examples, update the filter allowlists and producer logic in example apps.

The catalog already enforces compile-time duplicate-ID checks through manager build logic.

---

### 📁 Folder Structure

- `can_driver.c/h` — Hardware abstraction
- `can_manager.c/h` — RX dispatch and state update
- `can_state.c/h` — Global state structs
- `can_selftest.c/h` — Self-test routines
- `include/can_message_catalog.h` — Routing/source-of-truth table for IDs
- `test/` — ESP-IDF Unity target-based unit tests
- `include/` — Public headers
- `examples/` — Example applications


---

### 📚 Further Reading
- See header files in [include/](include/) for detailed API documentation.
- Example usage in [examples/basic_test/main/](examples/basic_test/main/).
- Manager/state flow example in [examples/manager_state/](examples/manager_state/).
- Self-test implementation in [examples/selftest/](examples/selftest/).
- Unity test guidance in [test/README.md](test/README.md).