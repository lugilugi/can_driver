# CAN Unity Test Example

This app runs the component's Unity `TEST_CASE` tests directly from this repository.

## What it runs

- `test/test_can_payloads.c` (payload helpers and wire format)
- `test/test_can_catalog.c` (catalog completeness and ID integrity)
- `test/test_can_state.c` (energy state synchronization accessor behavior)
- `test/test_can_driver_contract.c` (driver pre-init API contracts)

To add more suites, create `test/test_*.c` and add it to
`examples/unit_test/main/CMakeLists.txt`.

## Build/flash/monitor

1. Open this folder as the ESP-IDF project root:
   - `examples/unit_test/`
2. Set target to `esp32c3`.
3. Build, flash, then start monitor.
4. Press Enter to show the test menu.
5. Run all CAN tests with `[can]`, or run by test index.

## Notes

- If your serial port is busy, close any other monitor tools first.
- On Windows, very long paths can break object generation. Keeping the repo in a short path (for example `C:\src\can_driver`) helps avoid this.
