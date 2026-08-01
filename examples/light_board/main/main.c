#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "driver/gpio.h"
#include "driver/ledc.h"

// From our newly simplified component and DBC generator
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
#define LIGHT_BOARD_MODE   LIGHT_BOARD_REAR   // set to LIGHT_BOARD_REAR for rear firmware

// =============================================================================
// GPIO pin assignments
// =============================================================================
#define CAN_TX_PIN              GPIO_NUM_4
#define CAN_RX_PIN              GPIO_NUM_5
#define CAN_STB_PIN             GPIO_NUM_6   // TCAN standby

#define LIGHT_PIN_RIGHT_SIGNAL  GPIO_NUM_0   // turn signal (digital, active-high)
#define LIGHT_PIN_LEFT_SIGNAL   GPIO_NUM_1   // turn signal (digital, active-high)
#define LIGHT_PIN_MAIN_RIGHT    GPIO_NUM_3   // main light  (PWM, active-high)
#define LIGHT_PIN_MAIN_LEFT     GPIO_NUM_10  // main light  (PWM, active-high)

// =============================================================================
// Main-light PWM (LEDC). Outputs are active-high: higher duty = brighter.
// =============================================================================
#define LEDC_MODE          LEDC_LOW_SPEED_MODE
#define LEDC_TIMER         LEDC_TIMER_0
#define LEDC_DUTY_RES      LEDC_TIMER_13_BIT   // 0..8191
#define LEDC_FREQ_HZ       5000
#define LEDC_CH_MAIN_RIGHT LEDC_CHANNEL_0
#define LEDC_CH_MAIN_LEFT  LEDC_CHANNEL_1

#define MAIN_DUTY_OFF      0
#define MAIN_DUTY_RUNNING  2048   // ~25% — rear running/tail light
#define MAIN_DUTY_FULL     8191   // 100% — headlights (front) / brake (rear)

// =============================================================================
// Blink timing for turn signals / hazards (~1.5 Hz -> 333 ms half-period)
// =============================================================================
#define SIGNAL_BLINK_PERIOD_MS  333

// =============================================================================
// Staleness threshold — ms since last valid frame before aux is ignored
// =============================================================================
#define LIGHT_AUX_STALE_MS    500

// =============================================================================
// Global State (Replaces the old complex can_state manager)
// =============================================================================
static struct network_aux_ctrl_t g_current_aux = {0};
static TickType_t g_last_aux_tick = 0;
static portMUX_TYPE g_aux_mux = portMUX_INITIALIZER_UNLOCKED;

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

// =============================================================================
// CAN RX task
// =============================================================================
static void can_rx_task(void *arg)
{
    uint8_t rx_buf[8];
    twai_frame_t rx_msg = { .buffer = rx_buf, .buffer_len = sizeof(rx_buf) };
    ESP_LOGI(TAG, "CAN RX Task started.");

    while (1) {
        if (can_driver_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            if (rx_msg.header.id == NETWORK_AUX_CTRL_FRAME_ID) {
                struct network_aux_ctrl_t decoded;
                // Parse standard TWAI frame via DBC generated code
                network_aux_ctrl_unpack(&decoded, rx_msg.buffer, rx_msg.buffer_len);

                // Safely update state
                portENTER_CRITICAL(&g_aux_mux);
                g_current_aux = decoded;
                g_last_aux_tick = xTaskGetTickCount();
                portEXIT_CRITICAL(&g_aux_mux);
            }
        }
    }
}

// =============================================================================
// Light task logic
// =============================================================================

static bool is_fresh(TickType_t last_rx_tick, uint32_t stale_ms)
{
    if (last_rx_tick == 0) return false;
    return (xTaskGetTickCount() - last_rx_tick) <= pdMS_TO_TICKS(stale_ms);
}

static bool signal_blink_on(void)
{
    TickType_t half_period = pdMS_TO_TICKS(SIGNAL_BLINK_PERIOD_MS);
    if (half_period == 0) return true;
    return ((xTaskGetTickCount() / half_period) & 1U) == 0;
}

static void set_main_duty(uint32_t duty)
{
    ledc_set_duty(LEDC_MODE, LEDC_CH_MAIN_RIGHT, duty);
    ledc_set_duty(LEDC_MODE, LEDC_CH_MAIN_LEFT,  duty);
    ledc_update_duty(LEDC_MODE, LEDC_CH_MAIN_RIGHT);
    ledc_update_duty(LEDC_MODE, LEDC_CH_MAIN_LEFT);
}

static void update(void)
{
    struct network_aux_ctrl_t aux;
    TickType_t last_rx_tick;

    portENTER_CRITICAL(&g_aux_mux);
    aux = g_current_aux;
    last_rx_tick = g_last_aux_tick;
    portEXIT_CRITICAL(&g_aux_mux);

    bool aux_fresh = is_fresh(last_rx_tick, LIGHT_AUX_STALE_MS);
    if (!aux_fresh) {
        memset(&aux, 0, sizeof(aux)); // Zero out if stale
    }
    
    bool blink_on = signal_blink_on();

    // -------------------------------------------------------------------------
    // Debug logging
    // -------------------------------------------------------------------------
    {
        static struct network_aux_ctrl_t s_last_aux = {0};
        static bool s_first_log = true;
        static TickType_t s_last_log_tick = 0;
        TickType_t now = xTaskGetTickCount();
        
        bool changed = s_first_log || memcmp(&aux, &s_last_aux, sizeof(aux)) != 0;
        bool periodic = (now - s_last_log_tick) >= pdMS_TO_TICKS(1000);
        
        if (changed || periodic) {
            s_last_aux = aux;
            s_first_log = false;
            s_last_log_tick = now;
            
            if (last_rx_tick == 0) {
                ESP_LOGW(TAG, "CAN RX: no AUX frame ever received");
            } else {
                uint32_t age_ms = (uint32_t)(now - last_rx_tick) * portTICK_PERIOD_MS;
                ESP_LOGI(TAG,
                         "CAN RX aux fresh=%d age=%lums | L=%u R=%u brk=%u head=%u haz=%u horn=%u wipe=%u",
                         aux_fresh, (unsigned long)age_ms,
                         aux.left_turn, aux.right_turn,
                         aux.brake_light, aux.headlights,
                         aux.hazards, aux.horn,
                         aux.wipers);
            }
        }
    }

    // Turn signals — active-high: 1 = ON. Hazards force both.
    gpio_set_level(LIGHT_PIN_LEFT_SIGNAL,  (aux.left_turn  || aux.hazards) && blink_on);
    gpio_set_level(LIGHT_PIN_RIGHT_SIGNAL, (aux.right_turn || aux.hazards) && blink_on);

    if (LIGHT_BOARD_MODE == LIGHT_BOARD_FRONT) {
        // Front headlights: OFF until latched, then full.
        set_main_duty(aux.headlights ? MAIN_DUTY_FULL : MAIN_DUTY_OFF);
    } else {
        // Rear: running/tail lights always on. Brake escalation requires fresh frame.
        uint32_t duty = (aux_fresh && aux.brake_light) ? MAIN_DUTY_FULL : MAIN_DUTY_RUNNING;
        set_main_duty(duty);
    }
}

static void light_task(void *arg)
{
    ESP_LOGI(TAG, "Light task started — %s board",
             LIGHT_BOARD_MODE == LIGHT_BOARD_FRONT ? "FRONT" : "REAR");

    TickType_t last_wake = xTaskGetTickCount();
    while (s_running) {
        update();
        vTaskDelayUntil(&last_wake, pdMS_TO_TICKS(LIGHT_TASK_PERIOD_MS));
    }

    gpio_set_level(LIGHT_PIN_LEFT_SIGNAL,  0);
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
    // CAN transceiver standby control
    gpio_config_t ctrl_cfg = {
        .pin_bit_mask = (1ULL << CAN_STB_PIN),
        .mode         = GPIO_MODE_OUTPUT,
        .pull_up_en   = GPIO_PULLUP_DISABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
        .intr_type    = GPIO_INTR_DISABLE,
    };
    ESP_ERROR_CHECK(gpio_config(&ctrl_cfg));
    gpio_set_level(CAN_STB_PIN, 0);

    // Initialize TWAI Driver (Replaces old driver + manager init).
    // Hardware filter: only AUX_CTRL (0x210) frames wake the CPU; all other
    // bus traffic (pedal, power monitors) is dropped by the peripheral.
    CanInitFlags_t flags = {0};
    static const uint32_t rx_ids[] = { NETWORK_AUX_CTRL_FRAME_ID };
    CanFilterConfig_t filter = { .ids = rx_ids, .id_count = 1 };
    ESP_ERROR_CHECK(can_driver_init(CAN_TX_PIN, CAN_RX_PIN, 500000, flags, &filter));

    // Start our own RX parsing task instead of relying on the old manager!
    xTaskCreate(can_rx_task, "can_rx", 4096, NULL, 5, NULL);

    // Turn-signal GPIO
    const gpio_num_t signal_pins[] = { LIGHT_PIN_LEFT_SIGNAL, LIGHT_PIN_RIGHT_SIGNAL };
    for (int i = 0; i < 2; i++) {
        gpio_config_t cfg = {
            .pin_bit_mask = (1ULL << signal_pins[i]),
            .mode         = GPIO_MODE_OUTPUT,
            .pull_up_en   = GPIO_PULLUP_DISABLE,
            .pull_down_en = GPIO_PULLDOWN_DISABLE,
            .intr_type    = GPIO_INTR_DISABLE,
        };
        ESP_ERROR_CHECK(gpio_config(&cfg));
        gpio_set_level(signal_pins[i], 0); 
    }

    // Main-light PWM
    ledc_timer_config_t ledc_timer = {
        .speed_mode      = LEDC_MODE,
        .timer_num       = LEDC_TIMER,
        .duty_resolution = LEDC_DUTY_RES,
        .freq_hz         = LEDC_FREQ_HZ,
        .clk_cfg         = LEDC_AUTO_CLK,
    };
    ESP_ERROR_CHECK(ledc_timer_config(&ledc_timer));

    const uint32_t boot_duty = (LIGHT_BOARD_MODE == LIGHT_BOARD_REAR) ? MAIN_DUTY_RUNNING : MAIN_DUTY_OFF;

    const struct { ledc_channel_t ch; gpio_num_t pin; } main_ch[] = {
        { LEDC_CH_MAIN_RIGHT, LIGHT_PIN_MAIN_RIGHT },
        { LEDC_CH_MAIN_LEFT,  LIGHT_PIN_MAIN_LEFT  },
    };
    for (int i = 0; i < 2; i++) {
        ledc_channel_config_t ch_cfg = {
            .speed_mode = LEDC_MODE,
            .channel    = main_ch[i].ch,
            .timer_sel  = LEDC_TIMER,
            .intr_type  = LEDC_INTR_DISABLE,
            .gpio_num   = main_ch[i].pin,
            .duty       = boot_duty,
            .hpoint     = 0,
            .flags.output_invert = 0,
        };
        ESP_ERROR_CHECK(ledc_channel_config(&ch_cfg));
    }

    // Light update task
    s_running  = true;
    s_task_hdl = xTaskCreateStaticPinnedToCore(
        light_task, "light_ctrl",
        LIGHT_TASK_STACK_SIZE, NULL,
        LIGHT_TASK_PRIORITY, s_task_stack, &s_task_buf,
        LIGHT_TASK_CORE
    );

    if (s_task_hdl == NULL) {
        ESP_LOGE(TAG, "Failed to create light task");
        esp_restart();
    }
}
