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
| `can_driver_init(flags)`      | Configures GPIOs, baud rate, hardware filters. Initializes static TX pool and RX queue. |
| `can_driver_deinit()`         | Disables TWAI node and frees hardware resources. |
| `can_driver_transmit(id, data, len)` | Thread-safe. Claims a slot from the static pool and queues a frame for transmission. Non-blocking. |
| `can_driver_receive(evt, timeout)`    | Internal-use only (Manager). Blocks until a frame arrives in the RX queue. |
| `can_driver_recover()`         | Triggers TWAI bus-off recovery sequence. Must be called from task context. |
| `can_driver_is_bus_off()`      | Returns true if hardware has entered bus-off state due to electrical errors. |

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
| `g_can_pedal`     | 0x110     | Throttle (%), Brake (bool), last_rx_tick |
| `g_can_aux`       | 0x210     | Lights, wipers, horn bitfields |
| `g_can_pwr780`    | 0x310     | Traction battery Voltage and Current |
| `g_can_energy`    | 0x312     | 64-bit Accumulated Joules (critical section required for reads) |

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

#### Thread Safety (64-bit Data)
The ESP32-C3 is a 32-bit architecture. Accessing `g_can_energy.joules_780` is not atomic. Always wrap reads in a critical section to prevent "torn reads":

```c
// Thread-safe read of energy
portENTER_CRITICAL(&s_energy_mux); // Note: Requires exposing the mux or using a getter
current_joules = g_can_energy.joules_780;
portEXIT_CRITICAL(&s_energy_mux);
```

#### Bus-Off Recovery
The manager task handles recovery automatically. If `can_driver_is_bus_off()` is true, the manager will initiate recovery, which requires 129 occurrences of 11 consecutive recessive bits on the bus to complete.

---

### 🧪 Self-Test

A built-in self-test (`can_selftest`) uses TWAI loopback and self-test mode to verify every layer without external hardware. See [include/can_selftest.h](include/can_selftest.h) and [examples/selftest/](examples/selftest/) for details.

---

### 📁 Folder Structure

- `can_driver.c/h` — Hardware abstraction
- `can_manager.c/h` — RX dispatch and state update
- `can_state.c/h` — Global state structs
- `can_selftest.c/h` — Self-test routines
- `include/` — Public headers
- `examples/` — Example applications

---

### 📚 Further Reading
- See header files in [include/](include/) for detailed API documentation.
- Example usage in [examples/basic_test/main/](examples/basic_test/main/).
- Self-test implementation in [examples/selftest/](examples/selftest/).