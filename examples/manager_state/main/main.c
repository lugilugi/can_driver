#include "can_driver.h"
#include "can_manager.h"
#include "can_payloads.h"
#include "can_state.h"
#include "esp_err.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

static const char *TAG = "mgr_state_ex";

#define EXAMPLE_CAN_TX_IO GPIO_NUM_4
#define EXAMPLE_CAN_RX_IO GPIO_NUM_5
#define EXAMPLE_CAN_BAUD 500000

#define PRODUCER_PERIOD_MS 100
#define CONSUMER_PERIOD_MS 500

static bool is_stale(TickType_t last_rx_tick, uint32_t max_age_ms)
{
    if (last_rx_tick == 0) {
        return true;
    }
    return (xTaskGetTickCount() - last_rx_tick) > pdMS_TO_TICKS(max_age_ms);
}

static void producer_task(void *arg)
{
    uint32_t seq = 0;

    while (1) {
        PedalPayload pedal = {0};
        uint16_t throttle_raw = (uint16_t)((seq * 1500U) % 32768U);
        pedal.filtered_throttle = throttle_raw;
        pedal.flags.brake_active = (seq / 5U) % 2U;
        pedal.flags.deadman_active = 1;
        pedal.seq_counter = (uint8_t)(seq & 0xFFU);
        pedal.raw_throttle_adc = (uint16_t)(1000U + (seq % 200U));

        AuxControlPayload aux = { .rawAux = 0 };
        aux.headlights = 1;
        aux.left_turn = (seq / 10U) % 2U;
        aux.hazards = (seq / 20U) % 2U;

        PowerPayload pwr780 = {0};
        PowerPayload pwr740 = {0};
        PowerPayload_setRaw(&pwr780, 16000, 2000 + (int16_t)(seq % 100U));
        PowerPayload_setRaw(&pwr740, 12000, 1000 + (int16_t)(seq % 50U));

        EnergyPayload energy = {0};
        EnergyPayload_setRaw(&energy, 500000ULL + (uint64_t)seq * 1000ULL);

        (void)can_driver_transmit(CAN_ID_PEDAL, (const uint8_t *)&pedal, sizeof(pedal));
        (void)can_driver_transmit(CAN_ID_AUX_CTRL, &aux.rawAux, sizeof(aux));
        (void)can_driver_transmit(CAN_ID_PWR_MONITOR_780, (const uint8_t *)&pwr780, sizeof(pwr780));
        (void)can_driver_transmit(CAN_ID_PWR_MONITOR_740, (const uint8_t *)&pwr740, sizeof(pwr740));
        (void)can_driver_transmit(CAN_ID_PWR_ENERGY, (const uint8_t *)&energy, sizeof(energy));

        seq++;
        vTaskDelay(pdMS_TO_TICKS(PRODUCER_PERIOD_MS));
    }
}

static void consumer_task(void *arg)
{
    while (1) {
        bool pedal_stale = is_stale(g_can_pedal.last_rx_tick, 250);
        bool aux_stale = is_stale(g_can_aux.last_rx_tick, 500);
        bool pwr780_stale = is_stale(g_can_pwr780.last_rx_tick, 1000);

        if (pedal_stale) {
            ESP_LOGW(TAG, "PEDAL stale");
        } else {
            ESP_LOGI(TAG,
                     "PEDAL throttle=%.1f%% brake=%u seq=%u",
                     PedalPayload_getThrottle(&g_can_pedal.data),
                     (unsigned)g_can_pedal.data.flags.brake_active,
                     (unsigned)g_can_pedal.data.seq_counter);
        }

        if (!aux_stale) {
            ESP_LOGI(TAG,
                     "AUX headlights=%u left=%u hazards=%u",
                     (unsigned)g_can_aux.data.headlights,
                     (unsigned)g_can_aux.data.left_turn,
                     (unsigned)g_can_aux.data.hazards);
        }

        if (!pwr780_stale) {
            ESP_LOGI(TAG,
                     "PWR780 V=%.2f I=%.2f",
                     PowerPayload_getVoltage(&g_can_pwr780.data),
                     PowerPayload_getCurrent_780(&g_can_pwr780.data));
        }

        EnergyPayload energy = {0};
        can_state_get_energy_raw(&energy);
        ESP_LOGI(TAG, "ENERGY780 J=%.2f", EnergyPayload_getJoules_780(&energy));

        vTaskDelay(pdMS_TO_TICKS(CONSUMER_PERIOD_MS));
    }
}

void app_main(void)
{
    CanInitFlags_t flags = {
        .loopback = 1,
        .self_test = 1,
    };

    static const uint32_t allowed_ids[] = {
        CAN_ID_PEDAL,
        CAN_ID_AUX_CTRL,
        CAN_ID_PWR_MONITOR_780,
        CAN_ID_PWR_MONITOR_740,
        CAN_ID_PWR_ENERGY,
    };

    ESP_ERROR_CHECK(can_driver_init(EXAMPLE_CAN_TX_IO, EXAMPLE_CAN_RX_IO, EXAMPLE_CAN_BAUD, flags));
    ESP_ERROR_CHECK(can_driver_apply_single_filter_auto(
        allowed_ids,
        sizeof(allowed_ids) / sizeof(allowed_ids[0]),
        false));
    ESP_ERROR_CHECK(can_manager_init());

    xTaskCreate(producer_task, "can_prod", 3072, NULL, 4, NULL);
    xTaskCreate(consumer_task, "can_cons", 3072, NULL, 4, NULL);

    ESP_LOGI(TAG, "manager/state example running");
}
