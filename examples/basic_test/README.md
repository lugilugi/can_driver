# Basic CAN Driver Example

This example is an up-to-date driver-primitives demo focused on non-optional core APIs.

## What it demonstrates

- `can_driver_init(...)` in loopback/self-test mode.
- Filter configuration with:
	- `can_driver_apply_single_filter_auto(...)`
	- `can_driver_apply_dual_filter(...)`
	- `can_driver_apply_dual_filter_auto(...)`
- Frame TX/RX with:
	- `can_driver_transmit(...)`
	- `can_driver_receive(...)`
- Basic diagnostics:
	- `can_driver_get_pool_used()`
	- ISR RX counters (`calls/fail/dropped`)
- Cleanup with `can_driver_deinit()`.

## Structure

- `main/main.c`: Single entry point for the example.

## Usage

1. Open this folder as the ESP-IDF project root.
2. Build and flash on ESP32-C3.
3. Open monitor to see pass/fail logs for each filter/TX/RX step.

## Notes

- Default mode is loopback + self-test, so it can run on one board.
- For external bus testing, switch flags/pins in `main/main.c`.

See the main [README](../../README.md) for subsystem details.