# can_drive

## ECT CAN Driver for ESP32

This component abstracts the ESP32's TWAI peripheral into a robust Publisher/Subscriber model, ensuring telemetry tasks never interfere with critical motor control.

---

### 🌟 Key Features

- **Non-Blocking Architecture:** Uses FreeRTOS queues for background TX/RX. Your main code never hangs waiting for the CAN bus.
- **Topic-Based Routing:** Multiple tasks can subscribe to the same CAN ID (fan-out).
- **Automatic Fault Recovery:** Handles "Bus-Off" states and EMI-induced hardware hangs automatically.
- **Efficient Bit-Packing:** Includes a dictionary for 40-bit energy accumulators (INA780/740) to save bus bandwidth.

---

## 🛠 Installation

Add this component to your ESP-IDF project via the registry:

```bash
idf.py component add lugilugi/can_driver
```

Or add it to your `idf_component.yml`:

```yaml
dependencies:
  lugilugi/can_driver: "^1.0.0"
```

---

## 📖 Usage Overview

### 1. Header Files

- **CanDriver.h:** Contains the API and macros you'll use in your tasks.
- **CanMessages.h:** The "Source of Truth" for CAN IDs and data packing. All nodes must use the same version.

### 2. Initializing the Bus

Call this once in your `app_main`. It handles hardware setup and spins up background monitoring tasks.

```c
#include "CanDriver.h"

void app_main() {
    // Parameters: TX GPIO, RX GPIO, Baud Rate
    can_driver_init(GPIO_NUM_5, GPIO_NUM_4, 500000);
}
```

### 3. Publishing Data (Sender)

To send data, use the `CAN_PUBLISH` macro. It pushes your data into a software queue immediately, letting your sensor task move on while hardware handles wire timing.

```c
PedalPayload my_data;
PedalPayload_set(&my_data, 75.5, false); // 75.5% throttle, no brake

CAN_PUBLISH(CAN_ID_PEDAL, &my_data);
```

### 4. Subscribing to Data (Receiver)

The driver uses a Topic Router. You create a local queue and subscribe it to a specific CAN ID.

```c
QueueHandle_t pedal_q;
PedalPayload rx_pedal;

// Create a queue and subscribe it to CAN_ID_PEDAL
CAN_SUBSCRIBE_TOPIC(CAN_ID_PEDAL, &pedal_q, 5);

while(1) {
    // Wait for data with a timeout (safety first!)
    if (CAN_RECEIVE(pedal_q, &rx_pedal, 100)) {
        float throttle = PedalPayload_getThrottle(&rx_pedal);
        printf("Driving at %.1f%%\n", throttle);
    }
}
```

---

## 💡 Example Application

See `examples/basic_test/main/main.c` for a complete working demo with simulated pedal and motor controller tasks.

---

## 📚 License & Contributions

Open to contributions! Please submit issues or pull requests for improvements.