# Manager and State Example

This example demonstrates the manager-driven path:

1. Driver receives frames into the RX queue.
2. Manager task dispatches frames into global state.
3. Application tasks read from state structs and helper accessors.

## What it covers

- can_driver_init
- can_driver_apply_single_filter_auto
- can_manager_init
- can_driver_transmit (producer simulation)
- state reads via g_can_* and can_state_get_energy_raw
- staleness checks using last_rx_tick

## How to run

1. Open this folder as the ESP-IDF project root.
2. Build, flash, and monitor.
3. Observe producer/consumer logs in monitor.

## Notes

- Uses loopback and self-test flags so one ESP32-C3 board is enough.
- For real bus usage, switch flags and wiring in main/main.c.
