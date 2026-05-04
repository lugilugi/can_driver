#pragma once

#include <stdbool.h>

// =============================================================================
// can_selftest.h — Self-test for the entire CAN component.
//
// Uses the TWAI controller's built-in loopback + self-test mode so no external
// transceiver or second node is required. The controller ACKs its own frames
// and routes them back through the normal RX path, exercising every layer:
//
//   can_driver_transmit()  →  TWAI hardware (loopback)
//                          →  on_rx_done ISR
//                          →  RX queue
//                          →  can_manager task
//                          →  global state structs (can_state.h)
//
// IMPORTANT: can_selftest_run() must only be called when the driver and manager
// are NOT already running. It manages its own init/deinit cycle internally.
// If you call it after can_driver_init(), it will return false immediately.
//
// Typical usage in app_main():
//
//   #ifdef CONFIG_CAN_SELFTEST_ENABLE
//   if (!can_selftest_run()) {
//       ESP_LOGE("app", "CAN self-test FAILED — halting");
//       while (1) { vTaskDelay(portMAX_DELAY); }
//   }
//   #endif
//   ESP_ERROR_CHECK(can_driver_init((CanInitFlags_t){0}));
//   ESP_ERROR_CHECK(can_manager_init());
//
// =============================================================================

// Individual test IDs — reported in the result struct.
typedef enum {
    CAN_TEST_PREINIT_GUARD = 0, // driver rejects calls before init
    CAN_TEST_INVALID_ARGS,      // rejects len > 8
    CAN_TEST_INVALID_FLAGS,     // rejects loopback + listen_only combination
    CAN_TEST_INIT_LOOPBACK,     // loopback flag init succeeds, self_test implied
    CAN_TEST_MANAGER_INIT,      // manager starts successfully
    CAN_TEST_RX_TIMEOUT,        // receive times out on an empty queue
    CAN_TEST_CATALOG_COPY_ROUTES, // every catalog copy-route updates expected state
    CAN_TEST_PEDAL_ROUNDTRIP,   // PedalPayload encode → TX → RX → state update
    CAN_TEST_AUX_ROUNDTRIP,     // AuxControlPayload roundtrip
    CAN_TEST_PWR780_ROUNDTRIP,  // PowerPayload (INA780 scale) roundtrip
    CAN_TEST_PWR740_ROUNDTRIP,  // PowerPayload (INA740 scale) roundtrip
    CAN_TEST_ENERGY_ROUNDTRIP,  // EnergyPayload roundtrip
    CAN_TEST_STALENESS,         // last_rx_tick == 0 before RX, non-zero after;
                                //   also verifies tick was set by ISR (not manager)
    CAN_TEST_TX_POOL_EXHAUSTION,// pool returns NO_MEM when full, recovers after
    CAN_TEST_DEINIT,            // clean deinit + post-deinit guard restored
    CAN_TEST_COUNT              // must be last
} CanTestID_t;

// Result of a single test.
typedef struct {
    bool        passed;
    const char *name;       // human-readable test name
    const char *detail;     // failure detail, or NULL if passed
} CanTestResult_t;

// Aggregated result of the full test suite.
typedef struct {
    CanTestResult_t tests[CAN_TEST_COUNT];
    int             passed_count;
    int             failed_count;
    bool            all_passed;
} CanSelftestResult_t;

// Run the full self-test suite. Blocks until complete (~500 ms).
// Prints a formatted report via ESP_LOG on completion.
// Returns true if every test passed, false if any failed.
bool can_selftest_run(void);

// Run the test suite and return the full result struct for programmatic
// inspection. The caller can examine individual tests by index.
CanSelftestResult_t can_selftest_run_detailed(void);