#include "can_selftest.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "selftest_main";

void app_main(void)
{
    ESP_LOGI(TAG, "=== CAN Driver Self-Test ===");
    ESP_LOGI(TAG, "Running in loopback mode — no transceiver required.");

    CanSelftestResult_t result = can_selftest_run_detailed();

    if (result.all_passed) {
        ESP_LOGI(TAG, "");
        ESP_LOGI(TAG, "*** ALL TESTS PASSED (%d/%d) ***",
                 result.passed_count, CAN_TEST_COUNT);
        ESP_LOGI(TAG, "Driver is functioning correctly.");
    } else {
        ESP_LOGE(TAG, "");
        ESP_LOGE(TAG, "*** %d TEST(S) FAILED ***", result.failed_count);
        ESP_LOGE(TAG, "See failures above. Do not use this driver build.");
    }

    // Sit here so the result stays visible in the monitor without scrolling away.
    while (1) {
        vTaskDelay(portMAX_DELAY);
    }
}