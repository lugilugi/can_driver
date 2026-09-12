#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

#include "can_driver.h"
#include "network.h"

static const char *TAG = "main";

// =============================================================================
// Board mode toggle
//   0 = front (headlights)
//   1 = rear  (running + brake)
// =============================================================================
#define LIGHT_BOARD_FRONT  0
#define LIGHT_BOARD_REAR   1
#define LIGHT_BOARD_MODE   LIGHT_BOARD_REAR

// =============================================================================
// GPIO pin assignments
// =============================================================================
#define CAN_TX_PIN              GPIO_NUM_4
#define CAN_RX_PIN              GPIO_NUM_5
#define CAN_STB_PIN             GPIO_NUM_6

#define LIGHT_PIN_RIGHT_SIGNAL  GPIO_NUM_0
#define LIGHT_PIN_LEFT_SIGNAL   GPIO_NUM_1
#define LIGHT_PIN_MAIN_RIGHT    GPIO_NUM_3
#define LIGHT_PIN_MAIN_LEFT     GPIO_NUM_10

// =============================================================================
// Main-light PWM
// =============================================================================
#define LEDC_MODE          LEDC_LOW_SPEED_MODE
#define LEDC_TIMER         LEDC_TIMER_0
#define LEDC_DUTY_RES      LEDC_TIMER_13_BIT
#define LEDC_FREQ_HZ       5000
#define LEDC_CH_MAIN_RIGHT LEDC_CHANNEL_0
#define LEDC_CH_MAIN_LEFT  LEDC_CHANNEL_1

#define MAIN_DUTY_OFF      0
#define MAIN_DUTY_RUNNING  2048
#define MAIN_DUTY_FULL     8191

// =============================================================================
// Blink timing for turn signals / hazards
// =============================================================================
#define SIGNAL_BLINK_PERIOD_MS  333

// =============================================================================
// Local freshness policy
// =============================================================================
#define LIGHT_AUX_STALE_MS      500
#define LIGHT_PEDAL_STALE_MS    500

// =============================================================================
// Application state
//
// AUX_COMMAND and PEDAL_STATUS deliberately have independent freshness
// timestamps. A frame from one message must never refresh the other.
// =============================================================================
static struct network_aux_command_t g_current_aux = {0};
static TickType_t g_last_aux_tick = 0;

static bool g_current_brake_active = false;
static TickType_t g_last_pedal_tick = 0;

static portMUX_TYPE g_state_mux = portMUX_INITIALIZER_UNLOCKED;

// =============================================================================
// Task config
// =============================================================================
#define LIGHT_TASK_PERIOD_MS   20
#define LIGHT_TASK_PRIORITY    3
#define LIGHT_TASK_STACK_SIZE  2048
#define LIGHT_TASK_CORE        tskNO_AFFINITY

static StaticTask_t  s_task_buf;
static StackType_t   s_task_stack[LIGHT_TASK_STACK_SIZE];
static TaskHandle_t  s_task_hdl = NULL;
static volatile bool s_running  = false;

static bool frame_has_length(const twai_frame_t *frame,
                             size_t expected_length,
                             const char *message_name)
{
    if (frame->buffer_len != expected_length) {
        ESP_LOGW(TAG, "%s has invalid length: %u (expected %u)",
                 message_name,
                 (unsigned)frame->buffer_len,
                 (unsigned)expected_length);
        return false;
    }

    return true;
}

static bool is_defined_inhibit_reason(uint8_t reason)
{
    return reason <= NETWORK_PEDAL_STATUS_INHIBIT_REASON_CAN_LINK_CHOICE;
}

// =============================================================================
// CAN RX task
// =============================================================================
static void can_rx_task(void *arg)
{
    (void)arg;

    uint8_t rx_buf[8];
    twai_frame_t rx_msg = {
        .buffer = rx_buf,
        .buffer_len = sizeof(rx_buf),
    };

    ESP_LOGI(TAG, "CAN RX task started.");

    while (true) {
        esp_err_t rx_err = can_driver_receive(&rx_msg,
                                              sizeof(rx_buf),
                                              portMAX_DELAY);
        if (rx_err != ESP_OK) {
            if (rx_err != ESP_ERR_TIMEOUT) {
                ESP_LOGW(TAG, "CAN receive failed: %s",
                         esp_err_to_name(rx_err));
            }

            continue;
        }

        TickType_t received_tick = xTaskGetTickCount();

        if (rx_msg.header.id == NETWORK_AUX_COMMAND_FRAME_ID) {
            struct network_aux_command_t decoded = {0};

            if (!frame_has_length(&rx_msg,
                                  NETWORK_AUX_COMMAND_LENGTH,
                                  NETWORK_AUX_COMMAND_NAME) ||
                network_aux_command_unpack(&decoded,
                                           rx_msg.buffer,
                                           rx_msg.buffer_len) != 0) {
                continue;
            }

            portENTER_CRITICAL(&g_state_mux);
            g_current_aux = decoded;
            g_last_aux_tick = received_tick;
            portEXIT_CRITICAL(&g_state_mux);
        } else if (rx_msg.header.id == NETWORK_PEDAL_STATUS_FRAME_ID) {
            struct network_pedal_status_t decoded = {0};

            if (!frame_has_length(&rx_msg,
                                  NETWORK_PEDAL_STATUS_LENGTH,
                                  NETWORK_PEDAL_STATUS_NAME) ||
                network_pedal_status_unpack(&decoded,
                                            rx_msg.buffer,
                                            rx_msg.buffer_len) != 0) {
                continue;
            }

            if (!is_defined_inhibit_reason(decoded.inhibit_reason)) {
                ESP_LOGW(TAG,
                         "PEDAL_STATUS has reserved inhibit_reason=%u; "
                         "brake_active remains independently decoded",
                         (unsigned)decoded.inhibit_reason);
            }

            portENTER_CRITICAL(&g_state_mux);
            g_current_brake_active = decoded.brake_active != 0;
            g_last_pedal_tick = received_tick;
            portEXIT_CRITICAL(&g_state_mux);
        }
    }
}
// =============================================================================
// Light task logic
// =============================================================================
static bool is_fresh(TickType_t last_rx_tick, uint32_t stale_ms)
{
    if (last_rx_tick == 0) {
        return false;
    }

    return (xTaskGetTickCount() - last_rx_tick) <=
           pdMS_TO_TICKS(stale_ms);
}

static bool signal_blink_on(void)
{
    TickType_t half_period = pdMS_TO_TICKS(SIGNAL_BLINK_PERIOD_MS);

    if (half_period == 0) {
        return true;
    }

    return ((xTaskGetTickCount() / half_period) & 1U) == 0;
}

static void set_main_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, LEDC_CH_MAIN_RIGHT, duty);
    ledc_set_duty(LEDC_MODE, LEDC_CH_MAIN_LEFT, duty);
    ledc_update_duty(LEDC_MODE, LEDC_CH_MAIN_RIGHT);
    ledc_update_duty(LEDC_MODE, LEDC_CH_MAIN_LEFT);
}

static void update(void)
{
    struct network_aux_command_t aux;
    bool brake_active;
    TickType_t last_aux_tick;
    TickType_t last_pedal_tick;

    portENTER_CRITICAL(&g_state_mux);
    aux = g_current_aux;
    brake_active = g_current_brake_active;
    last_aux_tick = g_last_aux_tick;
    last_pedal_tick = g_last_pedal_tick;
    portEXIT_CRITICAL(&g_state_mux);

    bool aux_fresh = is_fresh(last_aux_tick, LIGHT_AUX_STALE_MS);
    bool pedal_fresh = is_fresh(last_pedal_tick, LIGHT_PEDAL_STALE_MS);

    if (!aux_fresh) {
        memset(&aux, 0, sizeof(aux));
    }

    if (!pedal_fresh) {
        brake_active = false;
    }

    bool blink_on = signal_blink_on();

    // -------------------------------------------------------------------------
    // Debug logging
    // -------------------------------------------------------------------------
    {
        static struct network_aux_command_t s_last_aux = {0};
        static bool s_last_brake_active = false;
        static bool s_first_log = true;
        static TickType_t s_last_log_tick = 0;
        TickType_t now = xTaskGetTickCount();

        bool changed = s_first_log ||
                       memcmp(&aux, &s_last_aux, sizeof(aux)) != 0 ||
                       brake_active != s_last_brake_active;
        bool periodic = (now - s_last_log_tick) >= pdMS_TO_TICKS(1000);

        if (changed || periodic) {
            s_last_aux = aux;
            s_last_brake_active = brake_active;
            s_first_log = false;
            s_last_log_tick = now;

            uint32_t aux_age_ms = last_aux_tick == 0
                                      ? 0
                                      : (uint32_t)(now - last_aux_tick) *
                                            portTICK_PERIOD_MS;
            uint32_t pedal_age_ms = last_pedal_tick == 0
                                        ? 0
                                        : (uint32_t)(now - last_pedal_tick) *
                                              portTICK_PERIOD_MS;

            ESP_LOGI(TAG,
                     "AUX fresh=%d age=%lums | "
                     "PEDAL fresh=%d age=%lums brake=%u | "
                     "L=%u R=%u head=%u haz=%u horn=%u wipe=%u",
                     aux_fresh,
                     (unsigned long)aux_age_ms,
                     pedal_fresh,
                     (unsigned long)pedal_age_ms,
                     (unsigned)brake_active,
                     (unsigned)aux.left_turn,
                     (unsigned)aux.right_turn,
                     (unsigned)aux.headlights,
                     (unsigned)aux.hazards,
                     (unsigned)aux.horn,
                     (unsigned)aux.wipers);
        }
    }

    gpio_set_level(LIGHT_PIN_LEFT_SIGNAL,
                   (aux.left_turn || aux.hazards) && blink_on);
    gpio_set_level(LIGHT_PIN_RIGHT_SIGNAL,
                   (aux.right_turn || aux.hazards) && blink_on);

    if (LIGHT_BOARD_MODE == LIGHT_BOARD_FRONT) {
        set_main_duty(aux.headlights ? MAIN_DUTY_FULL : MAIN_DUTY_OFF);
    } else {
        uint32_t duty = (pedal_fresh && brake_active)
                            ? MAIN_DUTY_FULL
                            : MAIN_DUTY_RUNNING;
        set_main_duty(duty);
    }
}

static void light_task(void *arg)
{
    (void)arg;

    ESP_LOGI(TAG,
             "Light task started — %s board",
             LIGHT_BOARD_MODE == LIGHT_BOARD_FRONT ? "FRONT" : "REAR");

    TickType_t last_wake = xTaskGetTickCount();
    while (s_running) {
        update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LIGHT_TASK_PERIOD_MS));
    }

    gpio_set_level(LIGHT_PIN_LEFT_SIGNAL, 0);
    gpio_set_level(LIGHT_PIN_RIGHT_SIGNAL, 0);
    set_main_duty(MAIN_DUTY_OFF);
    s_task_hdl = NULL;
    vTaskDelete(NULL);
}

// =============================================================================
// Entry point
// =============================================================================
void app_main(void)
{
    gpio_config_t ctrl_cfg = {
        .pin_bit_mask = (1ULL << CAN_STB_PIN),
        .mode = GPIO_MODE_OUTPUT,
        .pull_up_en = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ctrl_cfg));
    gpio_set_level(CAN_STB_PIN, 0);

    CanInitFlags_t flags = {0};

    static const uint32_t front_rx_ids[] = {
        NETWORK_AUX_COMMAND_FRAME_ID,
    };
    static const uint32_t rear_rx_ids[] = {
        NETWORK_AUX_COMMAND_FRAME_ID,
        NETWORK_PEDAL_STATUS_FRAME_ID,
    };

    const uint32_t *rx_ids;
    size_t rx_id_count;
    bool use_software_filter;

    if (LIGHT_BOARD_MODE == LIGHT_BOARD_REAR) {
        rx_ids = rear_rx_ids;
        rx_id_count = sizeof(rear_rx_ids) / sizeof(rear_rx_ids[0]);
        use_software_filter = true;
    } else {
        rx_ids = front_rx_ids;
        rx_id_count = sizeof(front_rx_ids) / sizeof(front_rx_ids[0]);
        use_software_filter = false;
    }

    CanFilterConfig_t filter = {
        .ids = rx_ids,
        .id_count = rx_id_count,
        .software_filter = use_software_filter,
    };
    ESP_ERROR_CHECK(can_driver_init(CAN_TX_PIN,
                                    CAN_RX_PIN,
                                    500000,
                                    flags,
                                    &filter));

    if (xTaskCreate(can_rx_task,
                    "can_rx",
                    4096,
                    NULL,
                    5,
                    NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN RX task");
        return;
    }

    const gpio_num_t signal_pins[] = {
        LIGHT_PIN_LEFT_SIGNAL,
        LIGHT_PIN_RIGHT_SIGNAL,
    };
    for (size_t i = 0; i < sizeof(signal_pins) / sizeof(signal_pins[0]); ++i) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << signal_pins[i]),
            .mode = GPIO_MODE_OUTPUT,
            .pull_up_en = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&cfg));
        gpio_set_level(signal_pins[i], 0);
    }

    ledc_timer_config_t ledc_timer = {
        .speed_mode = LEDC_MODE,
        .timer_num = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz = LEDC_FREQ_HZ,
        .clk_cfg = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    const uint32_t boot_duty =
        (LIGHT_BOARD_MODE == LIGHT_BOARD_REAR)
            ? MAIN_DUTY_RUNNING
            : MAIN_DUTY_OFF;

    const struct {
        ledc_channel_t ch;
        gpio_num_t pin;
    } main_channels[] = {
        {LEDC_CH_MAIN_RIGHT, LIGHT_PIN_MAIN_RIGHT},
        {LEDC_CH_MAIN_LEFT, LIGHT_PIN_MAIN_LEFT},
    };

    for (size_t i = 0;
         i < sizeof(main_channels) / sizeof(main_channels[0]);
         ++i) {
        ledc_channel_config_t channel_config = {
            .speed_mode = LEDC_MODE,
            .channel = main_channels[i].ch,
            .timer_sel = LEDC_TIMER,
            .intr_type = LEDC_INTR_DISABLE,
            .gpio_num = main_channels[i].pin,
            .duty = boot_duty,
            .hpoint = 0,
            .flags.output_invert = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&channel_config));
    }

    s_running = true;
    s_task_hdl = xTaskCreateStaticPinnedToCore(
        light_task,
        "light_ctrl",
        LIGHT_TASK_STACK_SIZE,
        NULL,
        LIGHT_TASK_PRIORITY,
        s_task_stack,
        &s_task_buf,
        LIGHT_TASK_CORE);

    if (s_task_hdl == NULL) {
        ESP_LOGE(TAG, "Failed to create light task");
        esp_restart();
    }
}
