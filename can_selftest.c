#include "can_selftest.h"
#include "can_driver.h"
#include "can_manager.h"
#include "can_state.h"
#include "can_message_catalog.h"
#include "can_payloads.h"
#include "can_config.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <string.h>

static const char *TAG = "can_selftest";

// =============================================================================
// Tolerance for floating-point comparisons.
// Encode/decode through packed integer representations introduces small rounding
// errors — the 15-bit throttle has massive resolution, but float conversion
// can still drift slightly.
// =============================================================================
#define FLOAT_EPSILON   0.02f
#define DOUBLE_EPSILON  0.001

static bool float_close(float a, float b, float eps)
{
    float d = a - b;
    return d > -eps && d < eps;
}

static bool double_close(double a, double b, double eps)
{
    double d = a - b;
    return d > -eps && d < eps;
}

// How long to wait after TX for the manager task to process the frame.
// In loopback mode the frame completes in microseconds; 50 ms is generous
// margin for the FreeRTOS scheduler.
#define MANAGER_SETTLE_MS   50

// Keep these manual roundtrip checks aligned with catalog copy-routes.
enum {
    CAN_SELFTEST_MANUAL_COPY_ROUTE_TESTS = 4
};

_Static_assert((int)CAN_SELFTEST_MANUAL_COPY_ROUTE_TESTS == (int)CAN_MESSAGE_COPY_ROUTE_COUNT,
               "Update can_selftest copy-route roundtrip coverage for new catalog entries");

typedef struct {
    const char *name;
    uint32_t id;
    uint8_t len;
    void *state_ptr;
    TickType_t *last_rx_tick_ptr;
} CanSelftestCopyRouteDesc_t;

#define MAKE_SELFTEST_COPY_ROUTE_DESC(_name, _id, _payload_t, _state_field, _tick_field) \
    {                                                                                      \
        .name = (_name),                                                                   \
        .id = (_id),                                                                       \
        .len = (uint8_t)sizeof(_payload_t),                                                \
        .state_ptr = &(_state_field),                                                      \
        .last_rx_tick_ptr = &(_tick_field),                                                \
    },

static const CanSelftestCopyRouteDesc_t s_copy_route_desc[] = {
    CAN_MESSAGE_COPY_ROUTE_TABLE(MAKE_SELFTEST_COPY_ROUTE_DESC)
};

// =============================================================================
// Test bookkeeping
// =============================================================================
static CanSelftestResult_t s_result;

static const char *s_test_names[CAN_TEST_COUNT] = {
    [CAN_TEST_PREINIT_GUARD]        = "Pre-init guard",
    [CAN_TEST_INVALID_ARGS]         = "Invalid arguments (len > 8)",
    [CAN_TEST_INVALID_FLAGS]        = "Invalid flags (loopback + listen_only)",
    [CAN_TEST_INIT_LOOPBACK]        = "Init (loopback + self_test flags)",
    [CAN_TEST_MANAGER_INIT]         = "Manager init",
    [CAN_TEST_RX_TIMEOUT]           = "RX timeout on empty queue",
    [CAN_TEST_CATALOG_COPY_ROUTES]  = "Catalog copy-route smoke coverage",
    [CAN_TEST_PEDAL_ROUNDTRIP]      = "Pedal payload roundtrip",
    [CAN_TEST_AUX_ROUNDTRIP]        = "AUX control roundtrip",
    [CAN_TEST_PWR780_ROUNDTRIP]     = "Power 780 roundtrip",
    [CAN_TEST_PWR740_ROUNDTRIP]     = "Power 740 roundtrip",
    [CAN_TEST_ENERGY_ROUNDTRIP]     = "Energy payload roundtrip",
    [CAN_TEST_STALENESS]            = "Staleness detection",
    [CAN_TEST_TX_POOL_EXHAUSTION]   = "TX pool exhaustion + recovery",
    [CAN_TEST_DEINIT]               = "Clean deinit + post-deinit guard",
};

static void pass(CanTestID_t id)
{
    s_result.tests[id].passed = true;
    s_result.tests[id].detail = NULL;
    s_result.passed_count++;
}

static void fail(CanTestID_t id, const char *reason)
{
    s_result.tests[id].passed = false;
    s_result.tests[id].detail = reason;
    s_result.failed_count++;
    ESP_LOGE(TAG, "  FAIL [%s]: %s", s_test_names[id], reason);
}

// =============================================================================
// Individual tests
// =============================================================================

// --- CAN_TEST_PREINIT_GUARD --------------------------------------------------
static void test_preinit_guard(void)
{
    uint8_t dummy = 0xA6;
    esp_err_t ret = can_driver_transmit(CAN_ID_PEDAL, &dummy, 1);
    if (ret == ESP_ERR_INVALID_STATE) {
        pass(CAN_TEST_PREINIT_GUARD);
    } else {
        fail(CAN_TEST_PREINIT_GUARD, "Expected ESP_ERR_INVALID_STATE before init");
    }
}

// --- CAN_TEST_INVALID_FLAGS --------------------------------------------------
static void test_invalid_flags(void)
{
    esp_err_t ret = can_driver_init(GPIO_NUM_1, GPIO_NUM_1, 500000, (CanInitFlags_t){
        .loopback    = 1,
        .listen_only = 1,
    });
    if (ret == ESP_ERR_INVALID_ARG) {
        pass(CAN_TEST_INVALID_FLAGS);
    } else {
        if (ret == ESP_OK) can_driver_deinit();
        fail(CAN_TEST_INVALID_FLAGS, "Expected ESP_ERR_INVALID_ARG for loopback + listen_only");
    }
}

// --- CAN_TEST_INIT_LOOPBACK --------------------------------------------------
static void test_init_loopback(void)
{
    esp_err_t ret = can_driver_init(GPIO_NUM_1, GPIO_NUM_1, 500000, (CanInitFlags_t){ .loopback = 1 });
    if (ret == ESP_OK) {
        pass(CAN_TEST_INIT_LOOPBACK);
    } else {
        fail(CAN_TEST_INIT_LOOPBACK, "can_driver_init(loopback) failed");
    }
}

// --- CAN_TEST_INVALID_ARGS ---------------------------------------------------
static void test_invalid_args(void)
{
    uint8_t buf[5] = {0};
    esp_err_t ret = can_driver_transmit(CAN_ID_PEDAL, buf, 9);
    if (ret == ESP_ERR_INVALID_ARG) {
        pass(CAN_TEST_INVALID_ARGS);
    } else {
        fail(CAN_TEST_INVALID_ARGS, "Expected ESP_ERR_INVALID_ARG for len=9");
    }
}

// --- CAN_TEST_MANAGER_INIT ---------------------------------------------------
static void test_manager_init(void)
{
    esp_err_t ret = can_manager_init();
    if (ret == ESP_OK) pass(CAN_TEST_MANAGER_INIT);
    else fail(CAN_TEST_MANAGER_INIT, "can_manager_init() returned error");
}

// --- CAN_TEST_RX_TIMEOUT -----------------------------------------------------
static void test_rx_timeout(void)
{
    CanRxEvent_t discard;
    while (can_driver_receive(&discard, 0) == ESP_OK) { /* drain */ }

    esp_err_t ret = can_driver_receive(&discard, pdMS_TO_TICKS(5));
    if (ret == ESP_ERR_TIMEOUT) pass(CAN_TEST_RX_TIMEOUT);
    else fail(CAN_TEST_RX_TIMEOUT, "Expected ESP_ERR_TIMEOUT on empty queue");
}

// --- CAN_TEST_CATALOG_COPY_ROUTES --------------------------------------------
static void test_catalog_copy_routes(void)
{
    uint8_t tx_buf[8] = {0};

    for (size_t i = 0; i < (sizeof(s_copy_route_desc) / sizeof(s_copy_route_desc[0])); i++) {
        const CanSelftestCopyRouteDesc_t *route = &s_copy_route_desc[i];

        if (route->len == 0 || route->len > 8) {
            ESP_LOGE(TAG, "Catalog route '%s' has invalid payload length %u",
                     route->name, (unsigned)route->len);
            fail(CAN_TEST_CATALOG_COPY_ROUTES, "Invalid catalog route payload length");
            return;
        }

        memset(route->state_ptr, 0, route->len);
        *route->last_rx_tick_ptr = 0;

        for (uint8_t j = 0; j < route->len; j++) {
            tx_buf[j] = (uint8_t)(0x30 + (uint8_t)i + j);
        }

        if (can_driver_transmit(route->id, tx_buf, route->len) != ESP_OK) {
            ESP_LOGE(TAG, "Catalog route '%s' TX failed", route->name);
            fail(CAN_TEST_CATALOG_COPY_ROUTES, "Catalog route TX failed");
            return;
        }

        vTaskDelay(pdMS_TO_TICKS(MANAGER_SETTLE_MS));

        if (*route->last_rx_tick_ptr == 0) {
            ESP_LOGE(TAG, "Catalog route '%s' did not update last_rx_tick", route->name);
            fail(CAN_TEST_CATALOG_COPY_ROUTES, "Catalog route did not update state timestamp");
            return;
        }

        if (memcmp(route->state_ptr, tx_buf, route->len) != 0) {
            ESP_LOGE(TAG, "Catalog route '%s' payload copy mismatch", route->name);
            fail(CAN_TEST_CATALOG_COPY_ROUTES, "Catalog route payload copy mismatch");
            return;
        }
    }

    pass(CAN_TEST_CATALOG_COPY_ROUTES);
}

// --- CAN_TEST_PEDAL_ROUNDTRIP ------------------------------------------------
static void test_pedal_roundtrip(void)
{
    memset(&g_can_pedal, 0, sizeof(g_can_pedal));

    // Setup expected values
    const uint16_t expected_throttle_raw = 16383; // roughly 50% throttle
    const bool     expected_kill_active  = true;
    const uint8_t  expected_sequence     = 42;

    // Construct the new 6-byte payload using filtered_throttle
    PedalPayload p = {0};
    p.filtered_throttle = (expected_kill_active ? 0x8000 : 0x0000) | (expected_throttle_raw & 0x7FFF);
    p.flags.deadman_active = 0; // Simulated dropped deadman
    p.seq_counter = expected_sequence;
    p.raw_throttle_adc = 12000;

    can_driver_reset_isr_counters();
    
    // Transmit exactly 6 bytes
    if (can_driver_transmit(CAN_ID_PEDAL, (const uint8_t*)&p, sizeof(PedalPayload)) != ESP_OK) {
        fail(CAN_TEST_PEDAL_ROUNDTRIP, "TX failed"); return;
    }
    vTaskDelay(pdMS_TO_TICKS(MANAGER_SETTLE_MS));

    if (g_can_pedal.last_rx_tick == 0) {
        fail(CAN_TEST_PEDAL_ROUNDTRIP, "State not updated — manager did not process frame"); return;
    }
    
    // Use the new payload getter functions
    uint16_t actual_throttle_raw = PedalPayload_getThrottleRaw(&g_can_pedal.data);
    bool     actual_kill_active  = PedalPayload_isKillActive(&g_can_pedal.data);

    if (actual_throttle_raw != expected_throttle_raw) {
        fail(CAN_TEST_PEDAL_ROUNDTRIP, "Throttle value mismatch"); return;
    }
    if (actual_kill_active != expected_kill_active) {
        fail(CAN_TEST_PEDAL_ROUNDTRIP, "Kill flag mismatch"); return;
    }
    if (g_can_pedal.data.seq_counter != expected_sequence) {
        fail(CAN_TEST_PEDAL_ROUNDTRIP, "Sequence counter mismatch"); return;
    }
    if (g_can_pedal.data.flags.deadman_active != 0) {
        fail(CAN_TEST_PEDAL_ROUNDTRIP, "Bitfield flags mismatch"); return;
    }
    
    pass(CAN_TEST_PEDAL_ROUNDTRIP);
}

// --- CAN_TEST_AUX_ROUNDTRIP --------------------------------------------------
static void test_aux_roundtrip(void)
{
    memset(&g_can_aux, 0, sizeof(g_can_aux));

    AuxControlPayload p = {.rawAux = 0};
    p.headlights = 1;
    p.left_turn  = 1;

    if (can_driver_transmit(CAN_ID_AUX_CTRL, &p.rawAux, sizeof(AuxControlPayload)) != ESP_OK) {
        fail(CAN_TEST_AUX_ROUNDTRIP, "TX failed"); return;
    }
    vTaskDelay(pdMS_TO_TICKS(MANAGER_SETTLE_MS));

    if (g_can_aux.last_rx_tick == 0) {
        fail(CAN_TEST_AUX_ROUNDTRIP, "State not updated"); return;
    }
    
    // Access bitfields through the current union field layout.
    if (g_can_aux.data.headlights != 1 || g_can_aux.data.left_turn != 1) {
        fail(CAN_TEST_AUX_ROUNDTRIP, "Set bits mismatch"); return;
    }
    if (g_can_aux.data.brake_light || g_can_aux.data.horn ||
        g_can_aux.data.right_turn  || g_can_aux.data.wipers) {
        fail(CAN_TEST_AUX_ROUNDTRIP, "Unexpected bits set in cleared fields"); return;
    }
    pass(CAN_TEST_AUX_ROUNDTRIP);
}

// --- CAN_TEST_PWR780_ROUNDTRIP -----------------------------------------------
static void test_pwr780_roundtrip(void)
{
    memset(&g_can_pwr780, 0, sizeof(g_can_pwr780));

    const uint16_t raw_v      = 16000;
    const int16_t  raw_a      = 2000;
    const float    expected_v = raw_v * POWER_V_SCALE;
    const float    expected_a = raw_a * POWER_I_780_SCALE;

    PowerPayload p;
    PowerPayload_setRaw(&p, raw_v, raw_a);

    if (can_driver_transmit(CAN_ID_PWR_MONITOR_780, (const uint8_t *)&p, sizeof(PowerPayload)) != ESP_OK) {
        fail(CAN_TEST_PWR780_ROUNDTRIP, "TX failed"); return;
    }
    vTaskDelay(pdMS_TO_TICKS(MANAGER_SETTLE_MS));

    if (g_can_pwr780.last_rx_tick == 0) {
        fail(CAN_TEST_PWR780_ROUNDTRIP, "State not updated"); return;
    }

    float actual_v = PowerPayload_getVoltage(&g_can_pwr780.data);
    float actual_a = PowerPayload_getCurrent_780(&g_can_pwr780.data);

    if (!float_close(actual_v, expected_v, 0.01f)) {
        fail(CAN_TEST_PWR780_ROUNDTRIP, "Voltage outside tolerance"); return;
    }
    if (!float_close(actual_a, expected_a, 0.01f)) {
        fail(CAN_TEST_PWR780_ROUNDTRIP, "Current outside tolerance"); return;
    }
    pass(CAN_TEST_PWR780_ROUNDTRIP);
}

// --- CAN_TEST_PWR740_ROUNDTRIP -----------------------------------------------
static void test_pwr740_roundtrip(void)
{
    memset(&g_can_pwr740, 0, sizeof(g_can_pwr740));

    const uint16_t raw_v      = 16000;
    const int16_t  raw_a      = 2000;
    const float    expected_v = raw_v * POWER_V_SCALE;
    const float    expected_a = raw_a * POWER_I_740_SCALE;

    PowerPayload p;
    PowerPayload_setRaw(&p, raw_v, raw_a);

    if (can_driver_transmit(CAN_ID_PWR_MONITOR_740, (const uint8_t *)&p, sizeof(PowerPayload)) != ESP_OK) {
        fail(CAN_TEST_PWR740_ROUNDTRIP, "TX failed"); return;
    }
    vTaskDelay(pdMS_TO_TICKS(MANAGER_SETTLE_MS));

    if (g_can_pwr740.last_rx_tick == 0) {
        fail(CAN_TEST_PWR740_ROUNDTRIP, "State not updated"); return;
    }

    float actual_v = PowerPayload_getVoltage(&g_can_pwr740.data);
    float actual_a = PowerPayload_getCurrent_740(&g_can_pwr740.data);

    if (!float_close(actual_v, expected_v, 0.01f)) {
        fail(CAN_TEST_PWR740_ROUNDTRIP, "Voltage outside tolerance"); return;
    }
    if (!float_close(actual_a, expected_a, 0.01f)) {
        fail(CAN_TEST_PWR740_ROUNDTRIP, "Current (740 scale) outside tolerance"); return;
    }
    pass(CAN_TEST_PWR740_ROUNDTRIP);
}

// --- CAN_TEST_ENERGY_ROUNDTRIP -----------------------------------------------
static void test_energy_roundtrip(void)
{
    memset(&g_can_energy, 0, sizeof(g_can_energy));

    const uint64_t raw          = 1000000ULL;
    const double   expected_780 = raw * ENERGY_LSB_JOULES_780;
    const double   expected_740 = raw * ENERGY_LSB_JOULES_740;

    EnergyPayload p;
    EnergyPayload_setRaw(&p, raw);

    if (can_driver_transmit(CAN_ID_PWR_ENERGY, p.rawEnergy, sizeof(EnergyPayload)) != ESP_OK) {
        fail(CAN_TEST_ENERGY_ROUNDTRIP, "TX failed"); return;
    }
    vTaskDelay(pdMS_TO_TICKS(MANAGER_SETTLE_MS));

    if (g_can_energy.last_rx_tick == 0) {
        fail(CAN_TEST_ENERGY_ROUNDTRIP, "State not updated"); return;
    }

    // Read the 5-byte payload through the synchronized state accessor.
    EnergyPayload safe_energy_rx;
    can_state_get_energy_raw(&safe_energy_rx);

    double actual_780 = EnergyPayload_getJoules_780(&safe_energy_rx);
    double actual_740 = EnergyPayload_getJoules_740(&safe_energy_rx);

    if (!double_close(actual_780, expected_780, DOUBLE_EPSILON)) {
        fail(CAN_TEST_ENERGY_ROUNDTRIP, "joules_780 outside tolerance"); return;
    }
    if (!double_close(actual_740, expected_740, DOUBLE_EPSILON)) {
        fail(CAN_TEST_ENERGY_ROUNDTRIP, "joules_740 outside tolerance"); return;
    }
    pass(CAN_TEST_ENERGY_ROUNDTRIP);
}

// --- CAN_TEST_STALENESS ------------------------------------------------------
static void test_staleness(void)
{
    if (g_can_dash.last_rx_tick != 0) {
        fail(CAN_TEST_STALENESS, "g_can_dash.last_rx_tick unexpectedly non-zero"); return;
    }
    if (g_can_pedal.last_rx_tick == 0) {
        fail(CAN_TEST_STALENESS, "g_can_pedal.last_rx_tick still zero after receiving a pedal frame"); return;
    }
    if (g_can_pedal.last_rx_tick > xTaskGetTickCount()) {
        fail(CAN_TEST_STALENESS, "last_rx_tick is in the future — ISR tick capture looks wrong"); return;
    }
    pass(CAN_TEST_STALENESS);
}

// --- CAN_TEST_TX_POOL_EXHAUSTION ---------------------------------------------
static void test_tx_pool_exhaustion(void)
{
    int no_mem_count = 0, ok_count = 0;
    const int attempts = CAN_TX_POOL_SIZE + 4;
    uint8_t dummy = 0x00;

    for (int i = 0; i < attempts; i++) {
        esp_err_t ret = can_driver_transmit(CAN_ID_DASH_STAT, &dummy, 1);
        if      (ret == ESP_OK)          ok_count++;
        else if (ret == ESP_ERR_NO_MEM)  no_mem_count++;
        else {
            fail(CAN_TEST_TX_POOL_EXHAUSTION, "Unexpected error code from transmit");
            return;
        }
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    if (can_driver_get_pool_used() != 0) {
        fail(CAN_TEST_TX_POOL_EXHAUSTION, "Pool slots still held after completion wait"); return;
    }

    if (can_driver_transmit(CAN_ID_DASH_STAT, &dummy, 1) != ESP_OK) {
        fail(CAN_TEST_TX_POOL_EXHAUSTION, "Transmit failed after pool fully drained"); return;
    }

    ESP_LOGI(TAG, "  Pool exhaustion: %d OK  %d NO_MEM  of %d attempts",
             ok_count, no_mem_count, attempts);
    pass(CAN_TEST_TX_POOL_EXHAUSTION);
}

// --- CAN_TEST_DEINIT ---------------------------------------------------------
static void test_deinit(void)
{
    if (can_manager_deinit() != ESP_OK) {
        fail(CAN_TEST_DEINIT, "can_manager_deinit() failed"); return;
    }
    if (can_driver_deinit() != ESP_OK) {
        fail(CAN_TEST_DEINIT, "can_driver_deinit() failed"); return;
    }

    uint8_t dummy = 0xFD;
    if (can_driver_transmit(CAN_ID_PEDAL, &dummy, 1) != ESP_ERR_INVALID_STATE) {
        fail(CAN_TEST_DEINIT, "Pre-init guard not restored after deinit"); return;
    }
    pass(CAN_TEST_DEINIT);
}

// =============================================================================
// Report
// =============================================================================
static void print_report(void)
{
    ESP_LOGI(TAG, "--------------------------------------------");
    ESP_LOGI(TAG, " CAN Self-Test Report");
    ESP_LOGI(TAG, "--------------------------------------------");
    for (int i = 0; i < CAN_TEST_COUNT; i++) {
        CanTestResult_t *t = &s_result.tests[i];
        if (t->passed) {
            ESP_LOGI(TAG, "  [PASS]  %s", s_test_names[i]);
        } else {
            ESP_LOGE(TAG, "  [FAIL]  %s  —  %s",
                     s_test_names[i],
                     t->detail ? t->detail : "(no detail)");
        }
    }
    ESP_LOGI(TAG, "--------------------------------------------");
    ESP_LOGI(TAG, "  Result: %d/%d passed%s",
             s_result.passed_count, CAN_TEST_COUNT,
             s_result.all_passed ? " — ALL PASS" : " — FAILURES DETECTED");
    ESP_LOGI(TAG, "--------------------------------------------");
}

// =============================================================================
// Runner
// =============================================================================
CanSelftestResult_t can_selftest_run_detailed(void)
{
    memset(&s_result, 0, sizeof(s_result));
    for (int i = 0; i < CAN_TEST_COUNT; i++) {
        s_result.tests[i].name = s_test_names[i];
    }

    vTaskDelay(pdMS_TO_TICKS(100));

    ESP_LOGI(TAG, "Starting CAN self-test (%d tests)...", CAN_TEST_COUNT);

    // Phase 1: Pre-init
    test_preinit_guard();
    test_invalid_flags();

    // Phase 2: Initialize in loopback mode
    test_init_loopback();
    if (!s_result.tests[CAN_TEST_INIT_LOOPBACK].passed) {
        ESP_LOGE(TAG, "Init failed — aborting remaining tests");
        goto done;
    }

    // Phase 3: Argument checks
    test_invalid_args();

    // Phase 4: Manager
    test_manager_init();
    if (!s_result.tests[CAN_TEST_MANAGER_INIT].passed) {
        ESP_LOGE(TAG, "Manager init failed — aborting remaining tests");
        can_driver_deinit();
        goto done;
    }

    // Phase 5: Functional
    test_rx_timeout();
    test_catalog_copy_routes();
    test_pedal_roundtrip();
    test_aux_roundtrip();
    test_pwr780_roundtrip();
    test_pwr740_roundtrip();
    test_energy_roundtrip();
    test_staleness();
    test_tx_pool_exhaustion();

    // Phase 6: Teardown
    test_deinit();

done:
    s_result.all_passed = (s_result.failed_count == 0) &&
                          (s_result.passed_count  == CAN_TEST_COUNT);
    print_report();
    return s_result;
}

bool can_selftest_run(void)
{
    return can_selftest_run_detailed().all_passed;
}