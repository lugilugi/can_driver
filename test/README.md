# ESP-IDF Unity Tests (Target-Based)

This component now includes target-based Unity tests under this directory.

## Quick start (recommended)

Use the in-repo test app at [examples/unit_test/](../examples/unit_test/):

1. Open `examples/unit_test/` as the ESP-IDF project.
2. Set target to `esp32c3`.
3. Build, flash, and monitor.
4. Press Enter to show the Unity menu.
5. Run `[can]` to execute the CAN test group.

## Files

- test_can_payloads.c: payload helper/wire-format tests.
- test_can_catalog.c: catalog metadata and ID integrity checks.
- test_can_state.c: synchronized energy state accessor tests.
- test_can_driver_contract.c: pre-init and argument validation contract tests.
- CMakeLists.txt: registers this test component with Unity.

## Optional: run with ESP-IDF unit-test-app

1. Open the ESP-IDF unit-test-app workspace (from your IDF checkout).
2. Add this repository as an extra component directory.
3. Build/flash the unit-test-app for your target (esp32c3).
4. Open monitor and press Enter to print the Unity menu.
5. Run test group [can] or specific test index.

Example invocation pattern (adjust paths for your machine):

idf.py -DTEST_COMPONENTS=can_driver -DEXTRA_COMPONENT_DIRS="<path-to-can_driver>" set-target esp32c3 build flash monitor

## Notes

- These tests are normal single-DUT target tests (no external hardware required).
- The existing can_selftest module remains the runtime loopback/system self-check path.
