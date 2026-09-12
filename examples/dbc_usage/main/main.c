#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_log.h"
#include "can_driver.h"
#include "network.h"

static const char *TAG = "can_example";

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

static void can_rx_task(void *arg)
{
    (void)arg;

    uint8_t rx_buf[8];
    twai_frame_t rx_msg = {
        .buffer = rx_buf,
        .buffer_len = sizeof(rx_buf),
    };

    ESP_LOGI(TAG, "CAN RX task started. Listening for ECT2026_CAN_V5 frames...");

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

        switch (rx_msg.header.id) {
            case NETWORK_PEDAL_STATUS_FRAME_ID: {
                struct network_pedal_status_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_PEDAL_STATUS_LENGTH,
                                      NETWORK_PEDAL_STATUS_NAME) ||
                    network_pedal_status_unpack(&decoded,
                                                rx_msg.buffer,
                                                rx_msg.buffer_len) != 0) {
                    break;
                }

                if (!is_defined_inhibit_reason(decoded.inhibit_reason)) {
                    ESP_LOGW(TAG,
                             "PEDAL_STATUS has reserved inhibit_reason=%u",
                             (unsigned)decoded.inhibit_reason);
                    break;
                }

                ESP_LOGI(TAG,
                         "PEDAL_STATUS throttle=%u inhibit=%u brake=%u reason=%u seq=%u",
                         (unsigned)decoded.throttle_command,
                         (unsigned)decoded.throttle_inhibit,
                         (unsigned)decoded.brake_active,
                         (unsigned)decoded.inhibit_reason,
                         (unsigned)decoded.seq_counter);
                break;
            }

            case NETWORK_AUX_COMMAND_FRAME_ID: {
                struct network_aux_command_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_AUX_COMMAND_LENGTH,
                                      NETWORK_AUX_COMMAND_NAME) ||
                    network_aux_command_unpack(&decoded,
                                               rx_msg.buffer,
                                               rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG,
                         "AUX_COMMAND left=%u right=%u headlights=%u hazards=%u horn=%u wipers=%u",
                         (unsigned)decoded.left_turn,
                         (unsigned)decoded.right_turn,
                         (unsigned)decoded.headlights,
                         (unsigned)decoded.hazards,
                         (unsigned)decoded.horn,
                         (unsigned)decoded.wipers);
                break;
            }

            case NETWORK_PACK_POWER_FRAME_ID: {
                struct network_pack_power_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_PACK_POWER_LENGTH,
                                      NETWORK_PACK_POWER_NAME) ||
                    network_pack_power_unpack(&decoded,
                                              rx_msg.buffer,
                                              rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG, "PACK_POWER voltage_raw=%u current_raw=%d",
                         (unsigned)decoded.voltage_v,
                         (int)decoded.current_a);
                break;
            }

            case NETWORK_AUX_POWER_FRAME_ID: {
                struct network_aux_power_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_AUX_POWER_LENGTH,
                                      NETWORK_AUX_POWER_NAME) ||
                    network_aux_power_unpack(&decoded,
                                             rx_msg.buffer,
                                             rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG, "AUX_POWER voltage_raw=%u current_raw=%d",
                         (unsigned)decoded.voltage_v,
                         (int)decoded.current_a);
                break;
            }

            case NETWORK_PACK_ENERGY_FRAME_ID: {
                struct network_pack_energy_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_PACK_ENERGY_LENGTH,
                                      NETWORK_PACK_ENERGY_NAME) ||
                    network_pack_energy_unpack(&decoded,
                                               rx_msg.buffer,
                                               rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG, "PACK_ENERGY joules_raw=%llu",
                         (unsigned long long)decoded.energy_j);
                break;
            }

            case NETWORK_VEHICLE_MOTION_FRAME_ID: {
                struct network_vehicle_motion_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_VEHICLE_MOTION_LENGTH,
                                      NETWORK_VEHICLE_MOTION_NAME) ||
                    network_vehicle_motion_unpack(&decoded,
                                                  rx_msg.buffer,
                                                  rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG,
                         "VEHICLE_MOTION speed_raw=%u distance_raw=%lu valid=%u seq=%u",
                         (unsigned)decoded.speed_kmh,
                         (unsigned long)decoded.trip_distance_m,
                         (unsigned)decoded.motion_valid,
                         (unsigned)decoded.seq_counter);
                break;
            }

            case NETWORK_VEHICLE_TIME_FRAME_ID: {
                struct network_vehicle_time_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_VEHICLE_TIME_LENGTH,
                                      NETWORK_VEHICLE_TIME_NAME) ||
                    network_vehicle_time_unpack(&decoded,
                                                rx_msg.buffer,
                                                rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG,
                         "VEHICLE_TIME year=%u month=%u day=%u %02u:%02u:%02u gps_synced=%u rtc_valid=%u",
                         (unsigned)network_vehicle_time_year_decode(decoded.year),
                         (unsigned)decoded.month,
                         (unsigned)decoded.day,
                         (unsigned)decoded.hour,
                         (unsigned)decoded.minute,
                         (unsigned)decoded.second,
                         (unsigned)decoded.gps_synced,
                         (unsigned)decoded.rtc_valid);
                break;
            }

            case NETWORK_GPS_STATUS_FRAME_ID: {
                struct network_gps_status_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_GPS_STATUS_LENGTH,
                                      NETWORK_GPS_STATUS_NAME) ||
                    network_gps_status_unpack(&decoded,
                                              rx_msg.buffer,
                                              rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG,
                         "GPS_STATUS satellites=%u fix=%u time=%u position=%u motion=%u seq=%u",
                         (unsigned)decoded.satellites,
                         (unsigned)decoded.fix_valid,
                         (unsigned)decoded.time_valid,
                         (unsigned)decoded.position_valid,
                         (unsigned)decoded.motion_valid,
                         (unsigned)decoded.seq_counter);
                break;
            }

            case NETWORK_GPS_POSITION_FRAME_ID: {
                struct network_gps_position_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_GPS_POSITION_LENGTH,
                                      NETWORK_GPS_POSITION_NAME) ||
                    network_gps_position_unpack(&decoded,
                                                rx_msg.buffer,
                                                rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG, "GPS_POSITION latitude_raw=%ld longitude_raw=%ld",
                         (long)decoded.latitude_deg,
                         (long)decoded.longitude_deg);
                break;
            }

            case NETWORK_GPS_MOTION_FRAME_ID: {
                struct network_gps_motion_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_GPS_MOTION_LENGTH,
                                      NETWORK_GPS_MOTION_NAME) ||
                    network_gps_motion_unpack(&decoded,
                                              rx_msg.buffer,
                                              rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG, "GPS_MOTION speed_raw=%u heading_raw=%u",
                         (unsigned)decoded.gps_speed_kmh,
                         (unsigned)decoded.heading_deg);
                break;
            }

            case NETWORK_MOTOR_STATE_FRAME_ID: {
                struct network_motor_state_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_MOTOR_STATE_LENGTH,
                                      NETWORK_MOTOR_STATE_NAME) ||
                    network_motor_state_unpack(&decoded,
                                               rx_msg.buffer,
                                               rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG,
                         "MOTOR_STATE speed=%d sequencer=%u status=%lu seq=%u",
                         (int)decoded.motor_speed_rpm,
                         (unsigned)decoded.sequencer_state,
                         (unsigned long)decoded.motor_status,
                         (unsigned)decoded.seq_counter);
                break;
            }

            case NETWORK_MOTOR_CURRENT_FRAME_ID: {
                struct network_motor_current_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_MOTOR_CURRENT_LENGTH,
                                      NETWORK_MOTOR_CURRENT_NAME) ||
                    network_motor_current_unpack(&decoded,
                                                  rx_msg.buffer,
                                                  rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG, "MOTOR_CURRENT iq=%d id=%d motor=%u",
                         (int)decoded.iq_a,
                         (int)decoded.id_a,
                         (unsigned)decoded.motor_current_a);
                break;
            }

            case NETWORK_MOTOR_VOLTAGE_FRAME_ID: {
                struct network_motor_voltage_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_MOTOR_VOLTAGE_LENGTH,
                                      NETWORK_MOTOR_VOLTAGE_NAME) ||
                    network_motor_voltage_unpack(&decoded,
                                                 rx_msg.buffer,
                                                 rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG, "MOTOR_VOLTAGE dc_bus=%u vq=%d vd=%d",
                         (unsigned)decoded.dc_bus_voltage_v,
                         (int)decoded.vq_v,
                         (int)decoded.vd_v);
                break;
            }

            case NETWORK_MOTOR_FAULTS_FRAME_ID: {
                struct network_motor_faults_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_MOTOR_FAULTS_LENGTH,
                                      NETWORK_MOTOR_FAULTS_NAME) ||
                    network_motor_faults_unpack(&decoded,
                                                rx_msg.buffer,
                                                rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG, "MOTOR_FAULTS fault_flags=%lu sw_faults=%lu",
                         (unsigned long)decoded.fault_flags,
                         (unsigned long)decoded.sw_faults);
                break;
            }

            case NETWORK_MOTOR_ESTIMATOR_FRAME_ID: {
                struct network_motor_estimator_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_MOTOR_ESTIMATOR_LENGTH,
                                      NETWORK_MOTOR_ESTIMATOR_NAME) ||
                    network_motor_estimator_unpack(&decoded,
                                                   rx_msg.buffer,
                                                   rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG,
                         "MOTOR_ESTIMATOR hall_angle=%u flux_angle=%u hall_speed=%d flux_speed=%d",
                         (unsigned)decoded.hall_angle_deg,
                         (unsigned)decoded.flux_angle_deg,
                         (int)decoded.hall_speed_rpm,
                         (int)decoded.flux_speed_rpm);
                break;
            }

            case NETWORK_MOTOR_PHASE_CURRENT_FRAME_ID: {
                struct network_motor_phase_current_t decoded = {0};

                if (!frame_has_length(&rx_msg,
                                      NETWORK_MOTOR_PHASE_CURRENT_LENGTH,
                                      NETWORK_MOTOR_PHASE_CURRENT_NAME) ||
                    network_motor_phase_current_unpack(&decoded,
                                                       rx_msg.buffer,
                                                       rx_msg.buffer_len) != 0) {
                    break;
                }

                ESP_LOGI(TAG, "MOTOR_PHASE_CURRENT u=%d v=%d w=%d",
                         (int)decoded.phase_u_current_a,
                         (int)decoded.phase_v_current_a,
                         (int)decoded.phase_w_current_a);
                break;
            }

            default:
                ESP_LOGW(TAG, "Unhandled CAN ID: 0x%lx",
                         (unsigned long)rx_msg.header.id);
                break;
        }
    }
}
void app_main(void)
{
    ESP_LOGI(TAG, "Starting ECT2026_CAN_V5 DBC example...");

    CanInitFlags_t flags = {
        .loopback = 0,
        .listen_only = 0,
    };

    static const uint32_t rx_ids[] = {
        NETWORK_PEDAL_STATUS_FRAME_ID,
        NETWORK_AUX_COMMAND_FRAME_ID,
        NETWORK_PACK_POWER_FRAME_ID,
        NETWORK_AUX_POWER_FRAME_ID,
        NETWORK_PACK_ENERGY_FRAME_ID,
        NETWORK_VEHICLE_MOTION_FRAME_ID,
        NETWORK_VEHICLE_TIME_FRAME_ID,
        NETWORK_GPS_STATUS_FRAME_ID,
        NETWORK_GPS_POSITION_FRAME_ID,
        NETWORK_GPS_MOTION_FRAME_ID,
        NETWORK_MOTOR_STATE_FRAME_ID,
        NETWORK_MOTOR_CURRENT_FRAME_ID,
        NETWORK_MOTOR_VOLTAGE_FRAME_ID,
        NETWORK_MOTOR_FAULTS_FRAME_ID,
        NETWORK_MOTOR_ESTIMATOR_FRAME_ID,
        NETWORK_MOTOR_PHASE_CURRENT_FRAME_ID,
    };
    CanFilterConfig_t filter = {
        .ids = rx_ids,
        .id_count = sizeof(rx_ids) / sizeof(rx_ids[0]),
        .software_filter = 1,
    };

    esp_err_t err = can_driver_init(GPIO_NUM_4,
                                    GPIO_NUM_5,
                                    500000,
                                    flags,
                                    &filter);
    if (err != ESP_OK) {
        ESP_LOGE(TAG, "Failed to initialize CAN driver: %s",
                 esp_err_to_name(err));
        return;
    }

    if (xTaskCreate(can_rx_task,
                    "can_rx_task",
                    4096,
                    NULL,
                    5,
                    NULL) != pdPASS) {
        ESP_LOGE(TAG, "Failed to create CAN RX task");
        return;
    }

    struct network_vehicle_motion_t vehicle_motion = {
        .speed_kmh = 1234,
        .trip_distance_m = 2500,
        .motion_valid =
            NETWORK_VEHICLE_MOTION_MOTION_VALID_VALID_CHOICE,
        .seq_counter = 1,
    };
    uint8_t payload[8];
    int payload_len = network_vehicle_motion_pack(payload,
                                                   &vehicle_motion,
                                                   sizeof(payload));
    if (payload_len != NETWORK_VEHICLE_MOTION_LENGTH) {
        ESP_LOGE(TAG, "Failed to pack VEHICLE_MOTION: %d", payload_len);
        return;
    }

    twai_frame_t tx_msg = {
        .header.id = NETWORK_VEHICLE_MOTION_FRAME_ID,
        .buffer = payload,
        .buffer_len = (size_t)payload_len,
    };

    if (can_driver_transmit(&tx_msg, pdMS_TO_TICKS(50)) != ESP_OK) {
        ESP_LOGW(TAG, "TX slot unavailable after 50ms");
    } else {
        ESP_LOGI(TAG, "Transmitted VEHICLE_MOTION successfully");
    }

    uint32_t tick = 0;
    while (true) {
        vTaskDelay(pdMS_TO_TICKS(1000));

        if (++tick % 5 == 0) {
            CanStatus_t status;
            if (can_driver_get_status(&status) == ESP_OK) {
                ESP_LOGI(TAG,
                         "state=%d TXslots=%lu TWAI_TXq=%lu RXq=%lu "
                         "TXerr=%u RXerr=%u bus_errs=%lu dropped=%lu "
                         "sw_dropped=%lu malformed=%lu recovery_failures=%lu",
                         status.error_state,
                         (unsigned long)status.tx_slots_remaining,
                         (unsigned long)status.twai_tx_queue_remaining,
                         (unsigned long)status.rx_queue_remaining,
                         status.tx_error_count,
                         status.rx_error_count,
                         (unsigned long)status.bus_error_count,
                         (unsigned long)status.rx_dropped_count,
                         (unsigned long)status.software_dropped_count,
                         (unsigned long)status.malformed_frame_count,
                         (unsigned long)status.recovery_failure_count);
            }
        }
    }
}
