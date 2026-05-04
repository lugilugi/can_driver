# CAN Driver Improvement Audit

Date: 2026-04-17

## Scope Reviewed
- Core: `can_driver.c`, `can_manager.c`, `can_state.c`, `can_selftest.c`, `can_logger.c`, `can_usb_forward.c`
- Public headers: `include/can_driver.h`, `include/can_manager.h`, `include/can_state.h`, `include/can_payloads.h`, `include/can_logger.h`, `include/can_usb_forward.h`, `include/can_config.h`
- Build/package docs: `CMakeLists.txt`, `idf_component.yml`, `README.md`
- Examples: `examples/basic_test/main/main.c`, `examples/selftest/main/selftestmain.c`
- CI workflow: `.github/workflows/publish.yml`

---

## Priority Summary (Do These First)
1. Fix concurrency correctness first (shared lock ownership, ISR/task-safe flags/counters).
2. Eliminate undefined behavior and API-contract mismatches (`can_selftest`, ISR-safe transmit contract).
3. Define and enforce overload behavior (RX/TX queue full policies, drop counters, observability).
4. Unify message onboarding (single message catalog; no multi-file manual drift).
5. Gate releases on reliability tests (build + selftest + fault-injection checks in CI).

---

## RTOS + CAN Reliability Target

This component should meet these invariants in normal and faulted operation:

1. No undefined behavior at runtime (no OOB, no null memcpy, no stale handle use).
2. Deterministic ISR behavior (bounded ISR work, no blocking operations).
3. Explicit backpressure behavior (every dropped frame path counted and exposed).
4. Safe cross-context synchronization (ISR <-> task and task <-> task correctness).
5. Deterministic lifecycle (init/start/stop/deinit are idempotent and race-safe).
6. Message-contract consistency (ID, payload, state, dispatch, test, docs derived from one source).

ESP-IDF alignment:
- TWAI ISR/transmit/receive behavior:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#transmit-from-isr>
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#receiving-messages>
- TWAI bus-off recovery and thread-safety guidance:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#bus-errors-and-recovery>
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#thread-safety>

---

## New ID And Data Struct Onboarding (Current Fragmentation)

### What adding one new message currently requires

To add one new CAN message type today, you typically touch all of these places:

1. Message dictionary and wire type
  - Add ID in `include/can_payloads.h` (enum at `include/can_payloads.h:26-32`).
  - Add packed payload struct/union and helpers in `include/can_payloads.h`.

2. Runtime state model
  - Add state struct or field in `include/can_state.h`.
  - Add global instance in `can_state.c` (globals at `can_state.c:13-18`).
  - Optionally add thread-safe getter/setter when multi-byte atomics matter (existing pattern at `can_state.c:20-25`).

3. Manager routing and state update
  - Add a new `case` in `can_manager.c` switch (`can_manager.c:36-101`).
  - Add length check + memcpy + `last_rx_tick` update for the new state.

4. Self-test coverage
  - Add test enum entry in `include/can_selftest.h` (`include/can_selftest.h:36-52`).
  - Add test name entry and roundtrip test function in `can_selftest.c`.
  - Add invocation ordering in the runner.

5. Docs/examples/filter lists
  - Update state/API tables in `README.md` (`README.md:66-71`).
  - Update any filter-ID allowlists in app examples (e.g., `examples/basic_test/main/main.c:171-176`).

### Why this feels fragmented

The same message metadata is repeated in multiple places, but in different forms:

- ID and wire shape: `include/can_payloads.h`.
- State ownership and staleness destination: `include/can_state.h` + `can_state.c`.
- Routing logic: `can_manager.c` switch.
- Validation coverage: `include/can_selftest.h` + `can_selftest.c`.
- Human-facing contract: `README.md` and examples.

There is no single source of truth that says: "ID X maps to payload Y, state Z, min length N, and test T".

### Precision improvements for onboarding (short term)

Add a mandatory checklist section in this repo (for PR template or contributing doc):

1. `include/can_payloads.h` updated.
2. `include/can_state.h` and `can_state.c` updated.
3. `can_manager.c` dispatch updated.
4. `include/can_selftest.h` and `can_selftest.c` updated.
5. `README.md` table updated.
6. Example filters updated if needed.
7. Build + selftest pass in CI.

This does not change architecture yet, but it removes "forgot one file" failures.

### Recommended unification design (medium term)

Create one catalog header, for example `include/can_message_catalog.h`, with one macro row per message:

```c
// name, id, payload_type, state_symbol, min_len
#define CAN_MESSAGE_TABLE(X) \
  X(PEDAL,       0x110, PedalPayload,      g_can_pedal,  sizeof(PedalPayload)) \
  X(AUX_CTRL,    0x210, AuxControlPayload, g_can_aux,    sizeof(AuxControlPayload)) \
  X(PWR_780,     0x310, PowerPayload,      g_can_pwr780, sizeof(PowerPayload))
```

Then generate from that table:

1. `CanMsgID_t` enum.
2. Central dispatch descriptors for manager (ID -> copy target, min len).
3. Static assertions for payload sizes.
4. Optional selftest registration skeleton.

This converts message onboarding from multi-file manual wiring into a mostly declarative one-row change plus any custom decode helper.

### Manager refactor target

Replace hardcoded switch copy blocks with descriptor-driven routing:

1. Lookup descriptor by ID.
2. Check `evt->len >= min_len`.
3. Copy payload to target state.
4. Stamp `last_rx_tick`.

Custom/exception IDs (if any) can still use explicit handlers, but common cases become uniform.

### Definition of done for adding a message (after unification)

1. Add one row to message catalog.
2. Add payload type only if new wire layout is introduced.
3. Add optional custom handler only if non-standard copy behavior is needed.
4. Selftest auto-registers baseline roundtrip from catalog.
5. CI ensures:
  - no duplicate IDs,
  - every catalog message has state backing,
  - manager route coverage equals catalog count.

### ESP-IDF alignment for ID onboarding

- Use low numeric IDs for higher-priority traffic and arbitration planning:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#transmitting-messages>
- Keep filter plans synchronized with the message catalog:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#filter-configuration>

---

## RTOS Reliability Hardening Workstreams (Actionable)

### W1. Concurrency correctness and memory safety
1. Single-owner lock model for shared state (especially energy path).
2. Replace cross-context `volatile bool` flags with atomic/critical-section access where required.
3. Add null/len argument guards to all public APIs before memcpy or queue access.
4. Ensure deinit clears or invalidates all handles that can be used by other tasks.

### W2. ISR/task boundary hardening
1. Make ISR-safe API claims match implementation exactly.
2. Ensure ISR paths use only ISR-safe FreeRTOS primitives and bounded operations.
3. Add explicit counters for ISR receive failures and queue overflows.

### W3. Queueing, backpressure, and overload behavior
1. Define behavior for each queue when full: drop-oldest, drop-newest, or fail-fast.
2. Export per-queue telemetry: posted, dropped, high-water mark, max stall duration.
3. Add runtime warnings only as rate-limited summaries, not per-frame floods.

### W4. Bus-off and recovery state machine
1. Represent bus-off lifecycle with explicit states (RUNNING, BUS_OFF, RECOVERING, READY).
2. Ensure one recovery sequence in flight at a time.
3. Record recovery attempts, successes, failures, and time-to-recover.

### W5. Deterministic startup and shutdown
1. Manager owns lifecycle of optional services (logger, USB forward).
2. Cooperative shutdown (notification + ack) replaces force delete fallback.
3. Init/deinit must be reentrant-safe and return clear status on partial startup.

### W6. Message model unification (new-ID reliability)
1. Introduce `can_message_catalog.h` as source-of-truth table.
2. Derive IDs, dispatch metadata, and baseline test registration from catalog.
3. Add compile-time checks for duplicate IDs and payload-size assumptions.

### W7. RTOS observability and diagnosability
1. Provide diagnostic API snapshot structs (driver, manager, logger, usb bridge).
2. Track CPU-impacting events: queue starvation, missed deadlines, long flush times.
3. Expose diagnostics in selftest and optional debug command path.

### W8. Reliability validation and fault injection
1. Add stress test for RX saturation and verify deterministic drop accounting.
2. Add bus-off injection test and verify recovery state transitions.
3. Add init/deinit race tests under repeated cycles.
4. Add malformed frame length and unknown-ID fuzz tests.

### W9. Release gating in CI
1. Build component and both examples.
2. Run selftest target.
3. Enforce no API-doc drift checks for critical public signatures.
4. Fail release when reliability assertions or counters indicate regressions.

### W10. Standalone modular selftest profile (ESP32-only default)
Goal:
1. Provide one selftest app that runs on a bare ESP32 board with no external peripherals by default.
2. Let developers enable module tests individually from a single "options at top" section.

Proposed test selection model:
1. Top-level profile switch:
  - `SELFTEST_PROFILE_MINIMAL` (core only, no external dependencies)
  - `SELFTEST_PROFILE_EXTENDED` (includes optional modules with internal mocks)
2. Per-module toggles:
  - `SELFTEST_ENABLE_DRIVER_CORE`
  - `SELFTEST_ENABLE_MANAGER_STATE`
  - `SELFTEST_ENABLE_FILTERS`
  - `SELFTEST_ENABLE_USB_FORWARD_INTERNAL`
  - `SELFTEST_ENABLE_LOGGER_INTERNAL`
  - `SELFTEST_ENABLE_FULL_INTEGRATION`

Module test intent (no external hardware):
1. Driver core:
  - TWAI loopback + self-test mode roundtrip and pool/queue stress.
2. Manager/state:
  - Dispatch correctness per ID, staleness stamps, short-frame handling.
3. Filters:
  - Auto/single/dual filter calculation checks and acceptance expectations.
4. USB forward internal:
  - Parser/formatter and queue-drop tests using injected input strings, no host required.
5. Logger internal:
  - Record packing, flush policy, drop accounting with RAM sink/mock writer when SD is absent.

Notes:
1. Keep external-dependent tests optional and explicitly labeled (host USB interaction, SD card media).
2. Default run should remain fully useful with only ESP32 + serial monitor.

---

## Detailed Findings And Improvements

## Reliability

### R1. Energy read/write lock mismatch can still produce torn reads
- Evidence:
  - `can_manager.c:16` defines `s_energy_mux`.
  - `can_manager.c:83` writes energy under that lock.
  - `can_state.c:3` defines a different `s_energy_mux`.
  - `can_state.c:24` reads energy under a different lock.
- Risk:
  - Writer and reader are not synchronized by the same mutex, so the intended protection is ineffective.
- Improvement:
  - Centralize ownership of the energy lock in one module only.
  - Recommended pattern: move both protected write and protected read behind `can_state_set_energy_raw()` and `can_state_get_energy_raw()` in `can_state.c`, and remove direct `g_can_energy` writes from `can_manager.c`.

### R2. Self-test has out-of-bounds reads in guard checks
- Evidence:
  - `can_selftest.c:84-85`: one-byte `dummy` is transmitted with `sizeof(PedalPayload)`.
  - `can_selftest.c:394-395`: same pattern in deinit test.
- Risk:
  - Undefined behavior during test execution, potentially masking real issues.
- Improvement:
  - Use a buffer sized to the requested length, or call `can_driver_transmit(..., &dummy, 1)` for guard-only checks.

### R3. `can_driver_transmit()` ISR safety contract is not matched by implementation
- Evidence:
  - Public contract says ISR-safe: `include/can_driver.h:113`.
  - Implementation uses task-context critical section in claim path: `can_driver.c:69` (`portENTER_CRITICAL`).
  - Function can be called from many contexts: `can_driver.c:287`.
- Risk:
  - API contract can mislead users into invoking from ISR when internals are not ISR-hardened.
- Improvement:
  - Either:
    1. Fully support ISR use by branching on ISR context and using ISR-safe primitives for pool claim, or
    2. Remove ISR-safe claim from API and expose a dedicated ISR transmit path.
- ESP-IDF reference:
  - TWAI transmit from ISR behavior and timeout semantics: <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#transmit-from-isr>

### R4. `can_driver_transmit()` does not validate `data` when `len > 0`
- Evidence:
  - `can_driver.c:301` does `memcpy(slot->data, data, len)` without null check.
- Risk:
  - Null pointer crash from caller misuse.
- Improvement:
  - Add argument validation:
    - `if (len > 0 && data == NULL) return ESP_ERR_INVALID_ARG;`

### R5. `can_driver_receive()` lacks initialized-state guard
- Evidence:
  - `can_driver.c:320` uses queue receive directly and does not check whether driver/queue is initialized.
- Risk:
  - Undefined behavior if called before `can_driver_init()` or after deinit race.
- Improvement:
  - Add checks for `s_node_hdl` and `s_rx_queue` and return `ESP_ERR_INVALID_STATE` when not ready.

### R6. RX queue overflow in ISR is not tracked
- Evidence:
  - `can_driver.c:146` calls `xQueueSendFromISR(...)` and ignores return value.
- Risk:
  - Silent frame drops under burst load, with no diagnostics.
- Improvement:
  - Check return value and increment an overflow counter exposed via diagnostics API.

### R7. Logger post/drop counters are race-prone and assume init order
- Evidence:
  - `can_logger.c:414-417` posts and increments `s_drops` without lock/atomic.
  - `can_logger.c:426` read-clear of `s_drops` is not synchronized.
  - `can_logger.c:340` queue is only created in `can_logger_init()`, but no global owner ensures init before first post.
  - `can_manager.c:139` always calls `can_logger_post(&evt)`.
- Risk:
  - Data races and possible null queue usage (when logger enabled but not initialized).
- Improvement:
  - Add `if (s_log_queue == NULL) return;` guard in post path.
  - Protect drop counter with atomic or critical section.
  - Define logger lifecycle ownership (see M1).

### R8. USB forward init can leave partial resources on task-creation failure
- Evidence:
  - `can_usb_forward.c:257`, `can_usb_forward.c:263` create two tasks.
  - `can_usb_forward.c:269-272` returns failure if either is null, but does not clean up the one that succeeded.
- Risk:
  - Partial-start subsystem with leaked task handles.
- Improvement:
  - On partial failure, delete whichever task exists and clear handles before return.

### R9. Manager and logger deinit use force-delete fallback instead of cooperative shutdown
- Evidence:
  - Manager: `can_manager.c:191`, `can_manager.c:195`.
  - Logger: `can_logger.c:383`, `can_logger.c:386`.
- Risk:
  - Force deletion can interrupt cleanup and increase shutdown nondeterminism.
- Improvement:
  - Replace with task notification/ack pattern:
    - signal stop,
    - task drains and exits,
    - waits for explicit completion event.

### R10. Self-test pin assumptions may not represent production constraints
- Evidence:
  - `can_selftest.c:96`, `can_selftest.c:111` initialize with TX and RX both as `GPIO_NUM_1`.
- Risk:
  - Potentially misleading for users who copy self-test assumptions into non-loopback mode.
- Improvement:
  - Keep loopback behavior explicit in comments and use dedicated constants indicating test-only pins.
- ESP-IDF reference:
  - TWAI hardware connection and single-node loopback testing guidance: <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#hardware-connection>

### R11. Bus-off flag signaling is cross-context but not explicitly synchronized
- Evidence:
  - ISR sets flag: `can_driver.c:157`.
  - Task reads/clears flag: `can_driver.c:339`, `can_driver.c:344`.
  - Backing storage is plain volatile bool: `can_driver.c:50`.
- Risk:
  - Volatile alone does not fully define cross-context synchronization semantics.
- Improvement:
  - Use atomic or critical-section protected flag accessors for ISR/task handoff.

### R12. Time-anchor fields are updated and read concurrently without a unified synchronization model
- Evidence:
  - Logger reads anchor-derived values in hot path: `can_logger.c:63`, `can_logger.c:298-299`.
  - Logger anchor update path sets anchor flags/values: `can_logger.c:395`, `can_logger.c:407-408`.
  - USB forward reads/writes anchor fields similarly: `can_usb_forward.c:26`, `can_usb_forward.c:35-37`.
- Risk:
  - Timestamp conversion can observe partially updated anchor state during resync windows.
- Improvement:
  - Use one synchronization strategy consistently (atomic snapshot or lock-guarded copy) for anchor state read/write.

### R13. Driver queue-handle lifecycle is not fully explicit on deinit
- Evidence:
  - RX queue created once at init: `can_driver.c:195`.
  - Deinit removes node handle but does not reset queue handle: `can_driver.c:275`.
  - Receive path uses queue directly: `can_driver.c:320`.
- Risk:
  - API behavior around post-deinit receive calls is underspecified and can drift.
- Improvement:
  - Define and enforce deinit contract explicitly (invalidate queue handle and return `ESP_ERR_INVALID_STATE` from receive when deinitialized).

### R14. USB-forward post path does not account for queue full drops
- Evidence:
  - `can_usb_forward.c:293` posts with `xQueueSend(...)` and ignores result.
- Risk:
  - Silent data loss under high USB/consumer latency, no telemetry for debugging.
- Improvement:
  - Check send result, increment drop counter, and expose read-clear diagnostic API similar to logger.

## Modularity

### M1. Optional subsystems are not lifecycle-owned by any module
- Evidence:
  - Manager posts logger events: `can_manager.c:139`.
  - No call sites for logger init/deinit outside logger module itself.
  - USB forward functions exist in `can_usb_forward.c` but are not integrated into manager lifecycle.
- Risk:
  - Feature enable flags can produce partially wired behavior.
- Improvement:
  - Introduce explicit service lifecycle in manager:
    - `can_manager_init()` calls optional `can_logger_init()` / `can_usb_forward_init()`.
    - `can_manager_deinit()` calls corresponding deinit functions.
  - Add initialization-state checks in post functions.

### M2. Filter APIs implemented in source are not exposed in public header
- Evidence:
  - Implemented: `can_driver.c:471`, `can_driver.c:484`, `can_driver.c:505`, `can_driver.c:543`.
  - Public header ends without these declarations: last public declarations at `include/can_driver.h:159-161`.
- Risk:
  - Hidden capabilities and API fragmentation.
- Improvement:
  - Add declarations and doc comments for all filter APIs in `include/can_driver.h`.
  - Provide one high-level wrapper for common filter use-cases.
- ESP-IDF reference:
  - Mask filter and dual filter behavior: <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#filter-configuration>

### M3. USB forward feature flag exists but module is not part of build graph
- Evidence:
  - Feature flag exists: `include/can_config.h:100`.
  - `can_usb_forward.c` is not listed in root `CMakeLists.txt` sources (`CMakeLists.txt:3-7`).
- Risk:
  - Feature appears available by config but is non-functional in component build.
- Improvement:
  - Add conditional source inclusion for USB forward, or remove/disable user-facing flag until integrated.

### M4. Build dependencies are always pulled even when logger is disabled
- Evidence:
  - Root `CMakeLists.txt:15-17` unconditionally requires `esp_driver_sdmmc`, `esp_driver_sdspi`, `fatfs`.
- Risk:
  - Increased dependency surface and portability friction.
- Improvement:
  - Gate optional dependencies with Kconfig / CMake options tied to logger enablement.

## Readability And Maintainability

### D1. API docs and examples are out-of-sync with actual interfaces
- Evidence:
  - README shows old signature `can_driver_init(flags)`: `README.md:46`.
  - Actual signature requires TX/RX/baud/flags: `include/can_driver.h:94`.
  - Header examples still show old single-argument style: `include/can_driver.h:29-31`.
  - README references removed energy fields (`joules_780`) and direct lock sample: `README.md:89-95`.
- Risk:
  - High onboarding confusion and incorrect usage.
- Improvement:
  - Regenerate README and header examples from a single authoritative API spec.
  - Keep one canonical basic example that compiles in CI.

### D2. Basic example modernization completed
- Status:
  - `examples/basic_test` now uses a single up-to-date entry point in `examples/basic_test/main/main.c`.
- Covered APIs:
  - `can_driver_init`, filter configuration APIs, `can_driver_transmit`, `can_driver_receive`, diagnostics counters, and `can_driver_deinit`.
- Remaining improvement:
  - Add a second advanced example for manager/state flow once additional integration scenarios are needed.

### D3. State documentation no longer matches actual state structure
- Evidence:
  - `include/can_state.h:25` still says energy fields are 64-bit doubles (`joules_780/joules_740`) though struct now stores raw `EnergyPayload`.
  - Getter comment references `g_can_energy.raw`: `include/can_state.h:104` while actual field is `g_can_energy.data.rawEnergy`.
- Risk:
  - Misuse by app developers and stale concurrency assumptions.
- Improvement:
  - Update comments to reflect current raw-payload model and centralized getter usage.

## Performance

### P1. Default receive filter is accept-all standard ID only
- Evidence:
  - `can_driver.c:248` sets `.is_ext = false` with ID/mask 0 in init path.
- Impact:
  - All standard frames pass into ISR path; extended IDs are implicitly excluded.
- Improvement:
  - Make default filter policy explicit in API:
    - mode `ACCEPT_ALL_STD`, `ACCEPT_ALL_EXT`, `ACCEPT_ALL_BOTH`, or explicit list-based auto filter.
  - Encourage explicit filter configuration for production buses.
- ESP-IDF reference:
  - Mask filter semantics and standard/extended mode behavior: <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#filter-configuration>

### P2. Manager loop polls every 10 ms regardless of load
- Evidence:
  - `can_manager.c:137` waits with `pdMS_TO_TICKS(10)` in hot loop.
- Impact:
  - Predictable but unnecessary wakeups in idle periods.
- Improvement:
  - Use longer receive block plus direct-to-task notify for bus-off events, or event-group signaling from callbacks to cut idle wakeups.

### P3. Unknown/short-frame logging can become noisy under bus noise
- Evidence:
  - Multiple warning/debug logs in dispatch path (`can_manager.c` switch block).
- Impact:
  - Log overhead under malformed traffic.
- Improvement:
  - Add rate-limited logging or counters with periodic summary output.

### P4. Filter auto configuration uses heap allocation (`malloc`) in embedded path
- Evidence:
  - `can_driver.c:569` allocates, `can_driver.c:602` frees.
- Impact:
  - Non-deterministic allocation behavior if invoked at runtime.
- Improvement:
  - Prefer caller-provided scratch buffer or fixed-size static scratch region.

## CI And Verification Gaps

### C1. CI publishes component but does not build/test examples
- Evidence:
  - `.github/workflows/publish.yml` only performs upload step (`line 22`) and contains no compile/test step.
- Risk:
  - API drift in examples and docs is not caught before publishing.
- Improvement:
  - Add pre-publish gates:
    - build component for target matrix,
    - build `examples/basic_test` and `examples/selftest`,
    - optionally run host-side static analysis.

---

## ESP-IDF Documentation References (From Provided Links)

### TWAI (Primary)
- Overview and classic/extended frame support:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#overview>
- Creating node, config flags, enable/disable behavior:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#creating-and-starting-a-twai-node>
- Transmit semantics, pointer lifetime, ISR behavior:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#transmitting-messages>
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#transmit-from-isr>
- Receive callback and ISR receive API usage:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#receiving-messages>
- Mask/dual filter semantics:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#filter-configuration>
- Bus-off and recovery requirements:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#bus-errors-and-recovery>
- Thread safety, IRAM/cache-safe performance notes:
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#thread-safety>
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#performance>
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/twai.html#cache-safety>

### SDIO Card Slave (Terminology Clarification)
- SDIO slave overview (CMD52/CMD53 host-slave protocol; not SD card filesystem host):
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/sdio_slave.html#overview>
  - <https://docs.espressif.com/projects/esp-idf/en/v5.5.3/esp32/api-reference/peripherals/sdio_slave.html#communication-with-esp-sdio-slave>
- Why this matters here:
  - In `include/can_config.h`, logger bus naming uses "SDIO" for SDMMC-style 4-bit card logging. Renaming to something like `SDMMC_4BIT` would reduce confusion with ESP-IDF `sdio_slave` APIs.

---

## Reliability Implementation Plan (RTOS-first)

### Phase 0. Baseline and guardrails
1. Add diagnostics baseline capture for RX/TX drops, bus-off events, and task lifecycle status.
2. Freeze public API surface for this hardening cycle (except explicit reliability changes).
3. Add CI build gates for component and examples before further refactors.

Exit criteria:
1. Every build runs with diagnostics enabled in debug profile.
2. CI fails on build breakage in examples.

### Phase 1. Safety and correctness blockers
1. Implement R1, R2, R3, R4, R5, R6, R11, R12, R13, R14.
2. Fix obvious undefined behavior and synchronization gaps before feature work.
3. Add targeted selftests for these failure modes.

Exit criteria:
1. No known UB findings remain in public code paths.
2. ISR contract and implementation are consistent and documented.
3. Queue overflow behavior is measurable via counters.

### Phase 2. Lifecycle and subsystem reliability
1. Implement M1, R7, R8, R9 and deterministic cooperative shutdown.
2. Define manager-owned startup/shutdown for optional services.
3. Ensure partial-start failures are rollback-safe.

Exit criteria:
1. 100-cycle init/deinit stress loop completes without deadlock or leaked task handles.
2. Optional services can be enabled/disabled without touching unrelated modules.

### Phase 3. Message model unification and onboarding reliability
1. Implement W6 and M2 with `can_message_catalog.h` source-of-truth.
2. Move manager routing to descriptor/table-driven path where possible.
3. Add compile-time checks for duplicate IDs and mapping completeness.

Exit criteria:
1. New-ID onboarding requires one catalog entry plus optional custom handler.
2. Catalog coverage checks pass in CI.

### Phase 4. Performance and noise hardening
1. Implement P1, P2, P3, P4.
2. Add rate-limited logging and event-summary diagnostics.
3. Tune filter defaults and wakeup behavior for lower idle CPU load.

Exit criteria:
1. Idle wakeups reduced versus baseline.
2. Log volume under malformed traffic remains bounded.

### Phase 5. Release-grade verification
1. Implement W8 and W9 fault-injection and stress suites.
2. Add release checklist linking docs, tests, and API consistency.
3. Ensure onboarding documentation exactly matches code.

Exit criteria:
1. Stress/fault suite pass rate is 100% in release CI.
2. No doc/API drift on critical public functions and message model.

### Phase 6. Final deliverable: standalone modular selftest creation
1. Implement W10 in `examples/selftest` as the final planned creation item.
2. Add one top-level options block for profile + per-module toggles.
3. Ensure default profile runs with no external hardware except ESP32 itself.
4. Document module matrix and expected pass/fail output in the selftest README.

Exit criteria:
1. Fresh board run passes core tests with no external items connected.
2. Optional module toggles run independently without requiring unrelated modules.
3. Test report clearly shows skipped external-dependent tests vs executed tests.

## Reliability Acceptance Matrix

1. Concurrency:
  - No shared state path uses different locks for read/write.
  - No unsynchronized read-clear counters across tasks.
  - Cross-context flags and time-anchor snapshots are synchronized with a single defined model.
2. ISR correctness:
  - ISR-only code paths are bounded and non-blocking.
  - ISR-to-task communication is fully observable (posted/dropped counters).
3. Overload handling:
  - Every queue has explicit full-policy and exported drop telemetry.
  - USB-forward and logger paths both expose queue drop counters.
4. Recovery behavior:
  - Bus-off transitions are observable and recovery is single-flight.
5. Lifecycle:
  - Init/deinit is deterministic under repeated cycles.
6. Onboarding:
  - Each CAN ID maps to payload, state, route, and test from one source table.
7. Selftest quality:
  - Core selftest runs on ESP32-only setup.
  - Module toggles allow isolated verification of features such as USB forward and logger paths.
