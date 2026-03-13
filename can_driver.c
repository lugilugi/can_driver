#include "can_driver.h"
#include "esp_log.h"
#include "freertos/semphr.h"

static const char* TAG = "CAN";
static bool is_running = false;

// --- Topic & Fan-Out Routing State ---
#define MAX_CAN_TOPICS 32        // Max unique CAN IDs the network can route
#define MAX_SUBS_PER_TOPIC 4     // Max application tasks that can listen to a single ID

typedef struct {
    uint32_t can_id;
    QueueHandle_t subscribers[MAX_SUBS_PER_TOPIC];
    uint8_t sub_count;
} CanTopic_t;

static CanTopic_t topic_table[MAX_CAN_TOPICS];
static uint8_t topic_count = 0;
static SemaphoreHandle_t router_mutex = NULL; // Protects the table during subscription
static TaskHandle_t tx_task_handle = NULL;

// --- Background TX State ---
static QueueHandle_t tx_queue = NULL;

// ===================================================================================
// BACKGROUND SYSTEM TASKS
// ===================================================================================

// 1. Hardware Fault Monitor: Watches the physical TWAI hardware for bus errors
static void twai_monitor_task(void *arg) {
    uint32_t alerts;
    uint32_t alert_mask = TWAI_ALERT_BUS_OFF | TWAI_ALERT_BUS_RECOVERED | 
                          TWAI_ALERT_RX_QUEUE_FULL | TWAI_ALERT_TX_FAILED;
    twai_reconfigure_alerts(alert_mask, NULL);

    while (1) {
        // Block until an alert fires (0% CPU usage while waiting)
        if (twai_read_alerts(&alerts, portMAX_DELAY) == ESP_OK) {
            
            // Critical Fault: Hardware disabled itself due to wiring/EMI issues
            if (alerts & TWAI_ALERT_BUS_OFF) {
                ESP_LOGE(TAG, "Bus-Off! Initiating recovery...");
                // NULL guard before suspend
                if (tx_task_handle != NULL) {
                    vTaskSuspend(tx_task_handle);
                }
                twai_initiate_recovery();
            }
            if (alerts & TWAI_ALERT_BUS_RECOVERED) {
                ESP_LOGI(TAG, "Bus Recovered!");
                twai_start();
                // Flush stale commands before resuming
                xQueueReset(tx_queue);
                if (tx_task_handle != NULL) {
                    vTaskResume(tx_task_handle);
                }
            }
            // Warning: The dispatch_task isn't reading data fast enough
            if (alerts & TWAI_ALERT_RX_QUEUE_FULL) {
                ESP_LOGE(TAG, "Hardware RX Queue Full! Dropping messages.");
            }
        }
    }
}

// 2. Transmit Task: Drains the software queue onto the physical bus
static void tx_task(void* arg) {
    twai_message_t tx_msg;
    while(1) {
        // Wait for an app task to push a message via CAN_PUBLISH
        if (xQueueReceive(tx_queue, &tx_msg, portMAX_DELAY) == pdTRUE) {
            
            // Attempt to send. Gives the hardware up to 50ms to win arbitration 
            // if the physical bus is heavily congested.
            esp_err_t res = twai_transmit(&tx_msg, pdMS_TO_TICKS(50));
            if (res != ESP_OK) {
                ESP_LOGE("CAN_TX", "Hardware TX Failed ID 0x%lX: %s", 
                         tx_msg.identifier, esp_err_to_name(res));
            }
        }
    }
}

// 3. Receive Router: Grabs messages off the wire and fans them out to app tasks
static void dispatch_task(void* arg) {
    twai_message_t rx_msg;
    while(1) {
        if (twai_receive(&rx_msg, portMAX_DELAY) == ESP_OK) {
            
            xSemaphoreTake(router_mutex, portMAX_DELAY);
            
            // Search the routing table for a matching topic
            for (int i = 0; i < topic_count; i++) {
                if (topic_table[i].can_id == rx_msg.identifier) {
                    
                    // FAN-OUT: Push a copy to every task that subscribed to this ID.
                    // A timeout of 0 prevents a stalled app task from freezing this router.
                    for (int j = 0; j < topic_table[i].sub_count; j++) {
                        xQueueSend(topic_table[i].subscribers[j], &rx_msg, 0);
                    }
                    break;
                }
            }
            xSemaphoreGive(router_mutex);
        }
    }
}

// ===================================================================================
// DRIVER IMPLEMENTATION
// ===================================================================================

esp_err_t can_driver_init(gpio_num_t tx_pin, gpio_num_t rx_pin, uint32_t baud_rate) {
    if (is_running) return ESP_OK;

    // Initialize OS primitives
    router_mutex = xSemaphoreCreateMutex();
    if (router_mutex == NULL) {
        ESP_LOGE(TAG, "Failed to create router mutex");
        return ESP_ERR_NO_MEM;
    }

    tx_queue = xQueueCreate(20, sizeof(twai_message_t));
    if (tx_queue == NULL) {
        ESP_LOGE(TAG, "Failed to create TX queue");
        vSemaphoreDelete(router_mutex);
        return ESP_ERR_NO_MEM;
    }

    // Configure TWAI Hardware
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(tx_pin, rx_pin, TWAI_MODE_NORMAL);
    g_config.tx_queue_len = 10; 
    g_config.rx_queue_len = 20;
    g_config.clk_src = TWAI_CLK_SRC_XTAL;

    twai_timing_config_t t_config;
    switch(baud_rate) {
        case 1000000: t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_1MBITS(); break;
        case 500000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_500KBITS(); break;
        case 250000:  t_config = (twai_timing_config_t)TWAI_TIMING_CONFIG_250KBITS(); break;
        default: return ESP_ERR_INVALID_ARG;
    }
    
    twai_filter_config_t f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    ESP_ERROR_CHECK(twai_driver_install(&g_config, &t_config, &f_config));
    ESP_ERROR_CHECK(twai_start());

    // Start Core Tasks
    // Monitor should preempt everything else to handle faults.
    if (xTaskCreate(twai_monitor_task, "CAN_Mon", 2048, NULL, 6, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create monitor task");
        return ESP_FAIL;
    }
    if (xTaskCreate(tx_task, "CAN_Tx", 4096, NULL, 5, &tx_task_handle) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create TX task");
        return ESP_FAIL;
    }
    if (xTaskCreate(dispatch_task, "CAN_Rx", 4096, NULL, 5, NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create dispatch task");
        return ESP_FAIL;
    }

    is_running = true;
    ESP_LOGI(TAG, "Driver Started @ %lu bps", baud_rate);
    return ESP_OK;
}

esp_err_t can_subscribe(uint32_t id, QueueHandle_t rx_queue) {
    if (rx_queue == NULL || router_mutex == NULL) return ESP_ERR_INVALID_ARG;

    xSemaphoreTake(router_mutex, portMAX_DELAY);

    // 1. Attempt to add to an existing topic
    for (int i = 0; i < topic_count; i++) {
        if (topic_table[i].can_id == id) {
            if (topic_table[i].sub_count < MAX_SUBS_PER_TOPIC) {
                topic_table[i].subscribers[topic_table[i].sub_count++] = rx_queue;
                xSemaphoreGive(router_mutex);
                return ESP_OK;
            } else {
                xSemaphoreGive(router_mutex);
                return ESP_ERR_NO_MEM; // Max tasks for this ID reached
            }
        }
    }

    // 2. Create a new topic if space allows
    if (topic_count < MAX_CAN_TOPICS) {
        topic_table[topic_count].can_id = id;
        topic_table[topic_count].subscribers[0] = rx_queue;
        topic_table[topic_count].sub_count = 1;
        topic_count++;
        xSemaphoreGive(router_mutex);
        return ESP_OK;
    }

    xSemaphoreGive(router_mutex);
    return ESP_ERR_NO_MEM; // Topic routing table is full
}

esp_err_t can_send_message(uint32_t id, const uint8_t* data, uint8_t len) {
    if (!is_running || tx_queue == NULL) return ESP_ERR_INVALID_STATE;

    twai_message_t tx_msg = {.identifier = id, .data_length_code = len};
    if (len > 8) return ESP_ERR_INVALID_ARG; // bounds check, TWAI max payload is 8 bytes
    memcpy(tx_msg.data, data, len); 

    // Non-blocking push to the background task queue
    if (xQueueSend(tx_queue, &tx_msg, 0) != pdTRUE) {
        ESP_LOGW(TAG, "Software TX Queue Full! Dropping ID 0x%lX", id);
        return ESP_FAIL;
    }
    
    return ESP_OK;
}