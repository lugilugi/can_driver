# CAN Self-Test Example

This example runs the built-in self-test for the CAN subsystem using TWAI loopback and self-test mode.

## Structure
- `main/selftestmain.c`: Entry point for self-test.
- Uses the `can_selftest` routines to verify hardware and software layers.

## Usage
- Build and flash to ESP32-C3.
- The self-test will run automatically if enabled in configuration.

## What It Tests
- Driver initialization and error handling.
- Manager task operation.
- RX/TX roundtrip for all payload types.
- Staleness detection and TX pool exhaustion.

See [include/can_selftest.h](../../include/can_selftest.h) and the main [README](../../README.md) for details.