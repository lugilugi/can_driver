# Basic CAN Example

This example demonstrates how to use the CAN communication subsystem in a simple application.

## Structure
- `main.c`: Entry point for the example.
- `can_example.c`: Shows how to interact with the CAN stack.

## Usage
- Build and flash to ESP32-C3.
- The example initializes the CAN driver and manager, then interacts with the state layer.

## Key Concepts
- Producer-Consumer model for CAN events.
- Staleness checks for safety.
- Thread-safe access to telemetry data.

See the main [README](../../README.md) for subsystem details.