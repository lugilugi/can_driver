#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_driver.h"
#include "can_messages.h"

static const char *TAG = "VEC_TEST";

// ==============================================================================
// 1. MOTOR CONTROLLER TASK (The Receiver)
// ==============================================================================
/**
 * Simulated Motor Controller. 
 * This task waits for throttle data and "commands" the motor.
 */
void motor_controller_task(void *arg) {
    // Zero-Boilerplate Subscription
    QueueHandle_t pedal_q; // create queue that will hold incoming messages for the pedal topic
    CAN_SUBSCRIBE_TOPIC(CAN_ID_PEDAL, &pedal_q, 10);

    PedalPayload pedal_rx_data; // struct to hold the unpacked data from the CAN message
    
    ESP_LOGI(TAG, "Motor Controller Task Started.");

    while (1) {
        // Wait up to 200ms for a message. 
        // If it takes longer, the sender might be dead (Safety Check).
        if (CAN_RECEIVE(pedal_q, &pedal_rx_data, 200)) {
            
            // Extract values using our type-safe getters
            float throttle = PedalPayload_getThrottle(&pedal_rx_data);
            bool brake = PedalPayload_isBrakePressed(&pedal_rx_data);
            
            ESP_LOGI("MOTOR", "Commanding Motor: %.1f%% Throttle | Brake: %s", 
                     throttle, brake ? "ON" : "OFF");

        } else {
            // Safety timeout triggered
            ESP_LOGE("MOTOR", "CRITICAL: Pedal node lost! Forcing safe state.");
        }
    }
}

// ==============================================================================
// 2. PEDAL SENSOR TASK (The Sender)
// ==============================================================================
/**
 * Simulated Pedal Node.
 * This task would usually read an ADC, but here it simulates a sweeping pedal.
 */
void pedal_sensor_task(void *arg) {
    PedalPayload tx_data;
    float simulated_throttle = 0.0f;
    bool direction_up = true;

    ESP_LOGI(TAG, "Pedal Sensor Task Started.");

    while (1) {
        // 1. Update simulated pedal position (0% -> 100% and back)
        if (direction_up) {
            simulated_throttle += 5.0f;
            if (simulated_throttle >= 100.0f) direction_up = false;
        } else {
            simulated_throttle -= 5.0f;
            if (simulated_throttle <= 0.0f) direction_up = true;
        }

        // 2. Pack the data into our 1-byte wire format
        PedalPayload_set(&tx_data, simulated_throttle, false);

        // 3. Publish to the bus (Non-blocking)
        CAN_PUBLISH(CAN_ID_PEDAL, &tx_data);

        // Run at 20Hz (every 50ms)
        vTaskDelay(pdMS_TO_TICKS(50));
    }
}

// ==============================================================================
// 3. MAIN ENTRY POINT
// ==============================================================================
void app_main(void) {
    ESP_LOGI(TAG, "Initializing Eco Archers CAN Driver...");

    // Initialize CAN Driver with your defined pins (IO5/IO4)
    // We use 500kbps as defined in your CanMessages.h
    esp_err_t err = can_driver_init(GPIO_NUM_5, GPIO_NUM_4, 500000);
    
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start CAN driver: %s", esp_err_to_name(err));
        return;
    }

    // Start the application tasks
    // Motor task gets slightly higher priority to ensure it processes messages immediately
    xTaskCreate(motor_controller_task, "Motor_Task", 4096, NULL, 5, NULL);
    xTaskCreate(pedal_sensor_task,     "Pedal_Task", 4096, NULL, 4, NULL);
}