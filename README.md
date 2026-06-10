# CAN Driver (ESP-IDF TWAI)

This component provides a lightweight CAN (TWAI) wrapper for ESP-IDF, specifically tailored to work seamlessly with `cantools` auto-generated DBC parsers. 

## Architecture

This component consists of a thin driver layer that abstracts away the complex TWAI setup, and generated message parsers based on your DBC specification:
- **`can_driver`**: A wrapper to safely initialize the TWAI peripheral, queue transmissions, and receive messages cleanly. 
- **`network.c` / `network.h`**: Auto-generated from `network.dbc` using `cantools`. This provides typed structures and unpack/pack functions for handling specific CAN network messages.

## Requirements

- **ESP-IDF**: >= 5.5.3
- **Hardware**: Any ESP32 series chip that features the TWAI (Two-Wire Automotive Interface) controller (e.g. ESP32, ESP32-S2, ESP32-S3, ESP32-C3).
- **External Transceiver**: A 3.3V compatible CAN transceiver is required to connect to a physical CAN bus.

## Example Usage

### 1. Initializing the Driver

```c
#include "can_driver.h"

// Set up the ESP32 TWAI driver using your board's TX and RX pins
// e.g., GPIO 21 for TX, GPIO 22 for RX at 500 kbps
CanInitFlags_t flags = { .loopback = 0, .listen_only = 0 };
esp_err_t err = can_driver_init(GPIO_NUM_21, GPIO_NUM_22, 500000, flags);
if (err != ESP_OK) {
    // Handle error
}
```

> [!WARNING]  
> Be sure to specify the correct `GPIO_NUM_xx` pins corresponding to your specific ESP32 target. For example, the ESP32-S3 uses different default pins, and `GPIO_NUM_22` is reserved internally! Make sure to update the pin constants in your code.

### 2. Receiving and Decoding Frames

Using the included auto-generated `cantools` code (`network.h`), parsing frames becomes straightforward:

```c
#include "network.h"

twai_message_t rx_msg;

// Wait for a message (blocks until one is received)
if (can_driver_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
    switch (rx_msg.identifier) {
        case NETWORK_PEDAL_FRAME_ID: {
            struct network_pedal_t decoded_pedal;
            
            // Unpack the raw data payload into the strongly typed struct
            network_pedal_unpack(&decoded_pedal, rx_msg.data, rx_msg.data_length_code);
            
            printf("Received Pedal Throttle: %d\n", decoded_pedal.throttle_raw);
            break;
        }
        default:
            // Unhandled CAN ID
            break;
    }
}
```

### 3. Transmitting Frames

```c
uint8_t payload[8] = {0};
// Populate payload using network_*_pack() ...

// Transmit onto the CAN bus, timeout of 50ms if TX queue is full
if (can_driver_transmit(0x400, payload, 8, pdMS_TO_TICKS(50)) == ESP_OK) {
    printf("Successfully sent frame\n");
}
```

## Modifying the DBC

If you modify `network.dbc` to define new network messages, you must regenerate the C parser code. You can do this by using the Python `cantools` package:

```bash
cantools generate_c_source network.dbc
```

This updates `network.c` and `network.h` with your new frames and signals.

## GitHub Actions / CI

A GitHub action is provided to automatically build the examples. Currently, it is configured to build exclusively for the `esp32` target to ensure the example code (which uses ESP32-specific GPIO 21 and 22 pins) successfully compiles. If you intend to use this on other chips (like ESP32-S3 or ESP32-C3), modify the pins in your application to match your hardware layout.