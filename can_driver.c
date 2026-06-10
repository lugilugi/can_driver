#include "can_driver.h"
#include "esp_log.h"
#include <string.h>

static const char *TAG = "can_driver";

// =============================================================================
// TWAI Node Initialization & wrapper
// =============================================================================

esp_err_t can_driver_init(gpio_num_t tx_io, gpio_num_t rx_io, uint32_t baud, CanInitFlags_t flags)
{
    // General config
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_io, rx_io, TWAI_MODE_NORMAL);
    
    // Adjust queues if necessary
    g_config.tx_queue_len = 16;
    g_config.rx_queue_len = 32;

    if (flags.listen_only) {
        g_config.mode = TWAI_MODE_LISTEN_ONLY;
    } else if (flags.loopback) {
        // ESP-IDF TWAI driver uses NO_ACK mode for loopback
        g_config.mode = TWAI_MODE_NO_ACK;
    }

    // Bit timing
    twai_timing_config_t t_config;
    switch (baud) {
        case 1000000: t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS(); break;
        case 800000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_800KBITS(); break;
        case 500000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS(); break;
        case 250000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS(); break;
        case 125000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_125KBITS(); break;
        default:
            ESP_LOGE(TAG, "Unsupported baud rate: %lu", (unsigned long)baud);
            return ESP_ERR_INVALID_ARG;
    }

    // Accept all filter
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    esp_err_t ret = twai_driver_install(&g_config, &t_config, &f_config);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to install TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = twai_start();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start TWAI driver: %s", esp_err_to_name(ret));
        twai_driver_uninstall();
        return ret;
    }

    ESP_LOGI(TAG, "TWAI initialized at %lu bps", (unsigned long)baud);
    return ESP_OK;
}

esp_err_t can_driver_deinit(void)
{
    esp_err_t ret = twai_stop();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to stop TWAI driver: %s", esp_err_to_name(ret));
        return ret;
    }

    ret = twai_driver_uninstall();
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to uninstall TWAI driver: %s", esp_err_to_name(ret));
    } else {
        ESP_LOGI(TAG, "TWAI deinitialized");
    }
    return ret;
}

esp_err_t can_driver_transmit(uint32_t id, const uint8_t *data, uint8_t len, TickType_t timeout_ticks)
{
    if (len > 8 || (len > 0 && data == NULL)) {
        return ESP_ERR_INVALID_ARG;
    }

    twai_message_t msg = {
        .identifier = id,
        .data_length_code = len,
        .extd = 0, // Standard 11-bit ID by default
        .rtr = 0
    };
    if (len > 0) {
        memcpy(msg.data, data, len);
    }

    return twai_transmit(&msg, timeout_ticks);
}

esp_err_t can_driver_receive(twai_message_t *msg, TickType_t timeout_ticks)
{
    if (msg == NULL) {
        return ESP_ERR_INVALID_ARG;
    }

    // In a robust implementation, you should check for bus-off status
    // and call twai_initiate_recovery() if needed. For simplicity, we just
    // pass through the receive call.
    return twai_receive(msg, timeout_ticks);
}