#include "can_driver.h"
#include "can_manager.h"
#include "can_selftest.h"
#include "can_state.h"
#include "can_payloads.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "can_example";

// =============================================================================
// Staleness helper
//
// The driver writes last_rx_tick on every frame. The application decides what
// "too old" means for its context. This helper centralises that logic.
// Returns true if the node has been silent for longer than max_age_ms.
// =============================================================================
static bool is_stale(TickType_t last_rx_tick, uint32_t max_age_ms)
{
    // last_rx_tick == 0 means no frame has ever been received.
    if (last_rx_tick == 0) {
        return true;
    }
    return (xTaskGetTickCount() - last_rx_tick) > pdMS_TO_TICKS(max_age_ms);
}

// =============================================================================
// Dashboard task — reads telemetry and logs it periodically.
// This is the typical consumer pattern: just read the global state structs.
// =============================================================================
static void dashboard_task(void *arg)
{
    while (1) {
        vTaskDelay(pdMS_TO_TICKS(500));

        // --- Pedal node: 200 ms staleness threshold (safety-relevant) ---
        if (is_stale(g_can_pedal.last_rx_tick, 200)) {
            ESP_LOGW(TAG, "PEDAL node silent — throttle data unreliable");
        } else {
            ESP_LOGI(TAG, "Throttle: %.1f%%  Brake: %s",
                     g_can_pedal.throttle,
                     g_can_pedal.brake ? "ON" : "off");
        }

        // --- Power monitors: 1000 ms staleness threshold (telemetry) ---
        if (!is_stale(g_can_pwr780.last_rx_tick, 1000)) {
            ESP_LOGI(TAG, "Traction — V: %.2f V  I: %.2f A",
                     g_can_pwr780.voltage, g_can_pwr780.current);
        }
        if (!is_stale(g_can_pwr740.last_rx_tick, 1000)) {
            ESP_LOGI(TAG, "Aux      — V: %.2f V  I: %.2f A",
                     g_can_pwr740.voltage, g_can_pwr740.current);
        }

        // --- Energy accumulator: just read, no staleness concern on a dash ---
        ESP_LOGI(TAG, "Energy (traction): %.2f Wh",
                 g_can_energy.joules_780 / 3600.0);
    }
}

// =============================================================================
// AUX control task — demonstrates both on-change and periodic TX.
//
// Pattern:
//   1. Maintain a local "desired state" struct.
//   2. When something changes, transmit immediately (on-change).
//   3. Also transmit on a 100 ms heartbeat regardless of changes.
//      A receiver that misses one frame will self-correct on the next heartbeat.
// =============================================================================
static void aux_control_task(void *arg)
{
    AuxControlPayload desired = {.raw = 0};
    AuxControlPayload last_sent = {.raw = 0xFF}; // force first transmission

    TickType_t last_heartbeat = xTaskGetTickCount();
    const TickType_t heartbeat_interval = pdMS_TO_TICKS(100);

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(10)); // 10 ms control loop

        // --- Application logic: set desired state here ---
        // (In a real application this would come from GPIO, UI, or another task.)
        desired.headlights = 1;
        desired.left_turn  = 0;
        // ...etc.

        bool state_changed   = (desired.raw != last_sent.raw);
        bool heartbeat_due   = (xTaskGetTickCount() - last_heartbeat) >= heartbeat_interval;

        if (state_changed || heartbeat_due) {
            esp_err_t ret = can_driver_transmit(
                CAN_ID_AUX_CTRL,
                &desired.raw,
                sizeof(AuxControlPayload)
            );

            if (ret == ESP_OK) {
                last_sent     = desired;
                last_heartbeat = xTaskGetTickCount();
            } else if (ret == ESP_ERR_NO_MEM) {
                // TX pool exhausted — all slots in-flight. Retry next iteration.
                ESP_LOGW(TAG, "AUX TX pool full, retrying next cycle");
            } else {
                ESP_LOGE(TAG, "AUX TX failed: %s", esp_err_to_name(ret));
            }
        }
    }
}

// =============================================================================
// Throttle TX example — manual encode and transmit.
// In a real system this runs in a motor-controller task at a fixed rate.
// =============================================================================
static void send_throttle(float throttle_pct, bool brake)
{
    PedalPayload p;
    PedalPayload_set(&p, throttle_pct, brake);

    esp_err_t ret = can_driver_transmit(CAN_ID_PEDAL, &p.rawData, sizeof(PedalPayload));
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Throttle TX failed: %s", esp_err_to_name(ret));
    }
}

// =============================================================================
// app_main — run selftest first, then start production driver.
// =============================================================================
void app_main(void)
{
    // -----------------------------------------------------------------------
    // Self-test (runs in loopback mode, manages its own init/deinit cycle).
    // Remove or gate with a Kconfig flag for production release builds.
    // -----------------------------------------------------------------------
    ESP_LOGI(TAG, "Running CAN self-test...");
    if (!can_selftest_run()) {
        ESP_LOGE(TAG, "CAN self-test FAILED — halting. Check logs above.");
        // In production you might trigger a fault LED, write to NVS, or reboot.
        while (1) { vTaskDelay(portMAX_DELAY); }
    }
    ESP_LOGI(TAG, "CAN self-test passed.");

    // -----------------------------------------------------------------------
    // Production init (normal mode, requires external transceiver).
    // -----------------------------------------------------------------------
    ESP_LOGI(TAG, "Initializing CAN (production mode)");
    ESP_ERROR_CHECK(can_driver_init((CanInitFlags_t){0}));
    ESP_ERROR_CHECK(can_manager_init());
    ESP_LOGI(TAG, "CAN ready");

    // Start application tasks.
    xTaskCreate(dashboard_task,    "dash",    2048, NULL, 4, NULL);
    xTaskCreate(aux_control_task,  "aux_tx",  2048, NULL, 4, NULL);

    // Example: send a single throttle frame from app_main context.
    send_throttle(0.0f, false);
}