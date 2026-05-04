#include "can_driver.h"
#include "can_payloads.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

#include <string.h>

static const char *TAG = "basic_test";

// Adjust these pins for your board + transceiver wiring when running without
// loopback mode.
#define EXAMPLE_CAN_TX_IO        GPIO_NUM_4
#define EXAMPLE_CAN_RX_IO        GPIO_NUM_5
#define EXAMPLE_CAN_BAUD         500000
#define EXAMPLE_RX_TIMEOUT_MS    150

static bool receive_one(CanRxEvent_t *evt, uint32_t timeout_ms)
{
    esp_err_t ret = can_driver_receive(evt, pdMS_TO_TICKS(timeout_ms));
    if (ret == ESP_OK) {
        return true;
    }
    if (ret == ESP_ERR_TIMEOUT) {
        ESP_LOGW(TAG, "RX timeout (%lu ms)", (unsigned long)timeout_ms);
        return false;
    }

    ESP_LOGE(TAG, "RX failed: %s", esp_err_to_name(ret));
    return false;
}

static bool transmit_and_expect(uint32_t id,
                                const uint8_t *data,
                                uint8_t len,
                                const char *label)
{
    CanRxEvent_t evt = {0};

    esp_err_t tx = can_driver_transmit(id, data, len);
    if (tx != ESP_OK) {
        ESP_LOGE(TAG, "%s: TX failed: %s", label, esp_err_to_name(tx));
        return false;
    }

    if (!receive_one(&evt, EXAMPLE_RX_TIMEOUT_MS)) {
        ESP_LOGE(TAG, "%s: did not receive expected frame", label);
        return false;
    }

    if (evt.id != id || evt.len != len || memcmp(evt.data, data, len) != 0) {
        ESP_LOGE(TAG, "%s: RX mismatch (id=0x%03lX len=%u)",
                 label, (unsigned long)evt.id, (unsigned)evt.len);
        return false;
    }

    ESP_LOGI(TAG, "%s: RX ok id=0x%03lX len=%u", label, (unsigned long)evt.id, (unsigned)evt.len);
    return true;
}

static bool demo_single_filter_auto(void)
{
    // Allow only PEDAL and AUX frames through the hardware filter.
    static const uint32_t allowed_ids[] = {
        CAN_ID_PEDAL,
        CAN_ID_AUX_CTRL,
    };

    esp_err_t ret = can_driver_apply_single_filter_auto(
        allowed_ids,
        sizeof(allowed_ids) / sizeof(allowed_ids[0]),
        false
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "single auto filter failed: %s", esp_err_to_name(ret));
        return false;
    }

    PedalPayload pedal = {0};
    pedal.filtered_throttle = 12345;
    pedal.flags.brake_active = 1;
    pedal.seq_counter = 7;

    if (!transmit_and_expect(CAN_ID_PEDAL, (const uint8_t *)&pedal, sizeof(pedal), "single-auto: pedal")) {
        return false;
    }

    // This ID is not in allowed_ids, so receive should time out.
    uint8_t dash_data[1] = {0xAB};
    ret = can_driver_transmit(CAN_ID_DASH_STAT, dash_data, sizeof(dash_data));
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "single-auto: DASH TX failed: %s", esp_err_to_name(ret));
        return false;
    }

    CanRxEvent_t filtered_evt = {0};
    if (can_driver_receive(&filtered_evt, pdMS_TO_TICKS(50)) != ESP_ERR_TIMEOUT) {
        ESP_LOGE(TAG, "single-auto: expected timeout for filtered-out ID");
        return false;
    }

    ESP_LOGI(TAG, "single-auto: filtered-out ID timed out as expected");
    return true;
}

static bool demo_dual_filter_manual(void)
{
    const uint32_t full_mask_std = 0x7FF;

    esp_err_t ret = can_driver_apply_dual_filter(
        CAN_ID_AUX_CTRL, full_mask_std,
        CAN_ID_PWR_MONITOR_780, full_mask_std,
        false
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "dual manual filter failed: %s", esp_err_to_name(ret));
        return false;
    }

    AuxControlPayload aux = {0};
    aux.headlights = 1;
    aux.left_turn = 1;

    if (!transmit_and_expect(CAN_ID_AUX_CTRL, &aux.rawAux, sizeof(aux), "dual-manual: aux")) {
        return false;
    }

    PowerPayload pwr = {0};
    PowerPayload_setRaw(&pwr, 16000, 2000);
    if (!transmit_and_expect(CAN_ID_PWR_MONITOR_780, (const uint8_t *)&pwr, sizeof(pwr), "dual-manual: pwr780")) {
        return false;
    }

    return true;
}

static bool demo_dual_filter_auto(void)
{
    static const uint32_t ids[] = {
        CAN_ID_PWR_MONITOR_780,
        CAN_ID_PWR_MONITOR_740,
        CAN_ID_PWR_ENERGY,
    };

    esp_err_t ret = can_driver_apply_dual_filter_auto(
        ids,
        sizeof(ids) / sizeof(ids[0]),
        false
    );
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "dual auto filter failed: %s", esp_err_to_name(ret));
        return false;
    }

    PowerPayload pwr = {0};
    PowerPayload_setRaw(&pwr, 12000, 1500);
    return transmit_and_expect(CAN_ID_PWR_MONITOR_740,
                               (const uint8_t *)&pwr,
                               sizeof(pwr),
                               "dual-auto: pwr740");
}

void app_main(void)
{
    // Loopback + self-test mode lets this run on one board without a second CAN node.
    CanInitFlags_t flags = {
        .loopback = 1,
        .self_test = 1,
    };

    ESP_LOGI(TAG, "Starting basic CAN driver example");
    ESP_ERROR_CHECK(can_driver_init(EXAMPLE_CAN_TX_IO, EXAMPLE_CAN_RX_IO, EXAMPLE_CAN_BAUD, flags));

    can_driver_reset_isr_counters();

    bool ok = true;
    ok = ok && demo_single_filter_auto();
    ok = ok && demo_dual_filter_manual();
    ok = ok && demo_dual_filter_auto();

    ESP_LOGI(TAG,
             "Diagnostics: pool_used=%d isr_rx_calls=%lu isr_rx_fail=%lu isr_rx_dropped=%lu",
             can_driver_get_pool_used(),
             (unsigned long)can_driver_get_isr_rx_calls(),
             (unsigned long)can_driver_get_isr_rx_fail(),
             (unsigned long)can_driver_get_isr_rx_dropped());

    ESP_ERROR_CHECK(can_driver_deinit());

    if (ok) {
        ESP_LOGI(TAG, "Basic example completed successfully");
    } else {
        ESP_LOGE(TAG, "Basic example encountered one or more failures");
    }

    while (1) {
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
