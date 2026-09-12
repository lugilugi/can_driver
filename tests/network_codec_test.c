#include <assert.h>
#include <stdint.h>
#include <string.h>

#include "network.h"

static void test_message_contract(void)
{
    assert(NETWORK_PEDAL_STATUS_FRAME_ID == 0x110u);
    assert(NETWORK_AUX_COMMAND_FRAME_ID == 0x210u);
    assert(NETWORK_PACK_POWER_FRAME_ID == 0x310u);
    assert(NETWORK_AUX_POWER_FRAME_ID == 0x311u);
    assert(NETWORK_PACK_ENERGY_FRAME_ID == 0x312u);
    assert(NETWORK_VEHICLE_MOTION_FRAME_ID == 0x400u);
    assert(NETWORK_VEHICLE_TIME_FRAME_ID == 0x401u);
    assert(NETWORK_GPS_STATUS_FRAME_ID == 0x410u);
    assert(NETWORK_GPS_POSITION_FRAME_ID == 0x411u);
    assert(NETWORK_GPS_MOTION_FRAME_ID == 0x412u);
    assert(NETWORK_MOTOR_STATE_FRAME_ID == 0x600u);
    assert(NETWORK_MOTOR_CURRENT_FRAME_ID == 0x601u);
    assert(NETWORK_MOTOR_VOLTAGE_FRAME_ID == 0x602u);
    assert(NETWORK_MOTOR_FAULTS_FRAME_ID == 0x603u);
    assert(NETWORK_MOTOR_ESTIMATOR_FRAME_ID == 0x604u);
    assert(NETWORK_MOTOR_PHASE_CURRENT_FRAME_ID == 0x605u);

    assert(NETWORK_PEDAL_STATUS_LENGTH == 6u);
    assert(NETWORK_AUX_COMMAND_LENGTH == 1u);
    assert(NETWORK_PACK_POWER_LENGTH == 4u);
    assert(NETWORK_AUX_POWER_LENGTH == 4u);
    assert(NETWORK_PACK_ENERGY_LENGTH == 5u);
    assert(NETWORK_VEHICLE_MOTION_LENGTH == 8u);
    assert(NETWORK_VEHICLE_TIME_LENGTH == 7u);
    assert(NETWORK_GPS_STATUS_LENGTH == 4u);
    assert(NETWORK_GPS_POSITION_LENGTH == 8u);
    assert(NETWORK_GPS_MOTION_LENGTH == 4u);
    assert(NETWORK_MOTOR_STATE_LENGTH == 8u);
    assert(NETWORK_MOTOR_CURRENT_LENGTH == 6u);
    assert(NETWORK_MOTOR_VOLTAGE_LENGTH == 6u);
    assert(NETWORK_MOTOR_FAULTS_LENGTH == 8u);
    assert(NETWORK_MOTOR_ESTIMATOR_LENGTH == 8u);
    assert(NETWORK_MOTOR_PHASE_CURRENT_LENGTH == 6u);
}

static void test_inhibit_reason_round_trip(void)
{
    uint8_t buffer[8];
    struct network_pedal_status_t source = {0};
    struct network_pedal_status_t decoded = {0};

    source.throttle_command = 0x1234u;
    source.throttle_inhibit = 1u;
    source.deadman_active = 1u;
    source.adc_fault = 1u;
    source.brake_active = 1u;
    source.is_calibrating = 1u;
    source.seq_counter = 0xA5u;
    source.throttle_adc_raw = 0xBEEFu;

    for (uint8_t reason = 0; reason <= 15u; ++reason) {
        source.inhibit_reason = reason;
        memset(&decoded, 0, sizeof(decoded));

        assert(network_pedal_status_pack(buffer,
                                         &source,
                                         sizeof(buffer)) ==
               NETWORK_PEDAL_STATUS_LENGTH);
        assert(network_pedal_status_unpack(&decoded,
                                           buffer,
                                           NETWORK_PEDAL_STATUS_LENGTH) == 0);
        assert(decoded.inhibit_reason == reason);
        assert(decoded.throttle_command == source.throttle_command);
        assert(decoded.brake_active == source.brake_active);
    }
}

static bool is_defined_inhibit_reason(uint8_t reason)
{
    return reason <= NETWORK_PEDAL_STATUS_INHIBIT_REASON_CAN_LINK_CHOICE;
}

static void test_inhibit_reason_policy(void)
{
    for (uint8_t reason = 0; reason <= 5u; ++reason) {
        assert(is_defined_inhibit_reason(reason));
    }

    for (uint8_t reason = 6u; reason <= 15u; ++reason) {
        assert(!is_defined_inhibit_reason(reason));
    }
}

static void test_aux_reserved_bits(void)
{
    uint8_t buffer[8] = {0};
    struct network_aux_command_t source = {
        .left_turn = 1u,
        .right_turn = 1u,
        .headlights = 1u,
        .hazards = 1u,
        .horn = 1u,
        .wipers = 1u,
    };
    struct network_aux_command_t decoded = {0};

    assert(network_aux_command_pack(buffer,
                                    &source,
                                    sizeof(buffer)) ==
           NETWORK_AUX_COMMAND_LENGTH);
    assert((buffer[0] & (1u << 2)) == 0u);
    assert((buffer[0] & (1u << 7)) == 0u);
    assert(network_aux_command_unpack(&decoded,
                                      buffer,
                                      NETWORK_AUX_COMMAND_LENGTH) == 0);
    assert(decoded.left_turn == source.left_turn);
    assert(decoded.right_turn == source.right_turn);
    assert(decoded.headlights == source.headlights);
    assert(decoded.hazards == source.hazards);
    assert(decoded.horn == source.horn);
    assert(decoded.wipers == source.wipers);
}

static void test_signed_and_scaled_fields(void)
{
    uint8_t buffer[8] = {0};
    struct network_pack_power_t power = {
        .voltage_v = 0xFFFFu,
        .current_a = -12345,
    };
    struct network_pack_power_t decoded_power = {0};

    assert(network_pack_power_pack(buffer,
                                   &power,
                                   sizeof(buffer)) ==
           NETWORK_PACK_POWER_LENGTH);
    assert(network_pack_power_unpack(&decoded_power,
                                     buffer,
                                     NETWORK_PACK_POWER_LENGTH) == 0);
    assert(decoded_power.voltage_v == power.voltage_v);
    assert(decoded_power.current_a == power.current_a);

    struct network_gps_position_t position = {
        .latitude_deg = -900000000,
        .longitude_deg = 1800000000,
    };
    struct network_gps_position_t decoded_position = {0};

    assert(network_gps_position_pack(buffer,
                                     &position,
                                     sizeof(buffer)) ==
           NETWORK_GPS_POSITION_LENGTH);
    assert(network_gps_position_unpack(&decoded_position,
                                       buffer,
                                       NETWORK_GPS_POSITION_LENGTH) == 0);
    assert(decoded_position.latitude_deg == position.latitude_deg);
    assert(decoded_position.longitude_deg == position.longitude_deg);
}

static void test_wide_fields(void)
{
    uint8_t buffer[8] = {0};
    struct network_pack_energy_t energy = {
        .energy_j = 0xFFFFFFFFFFULL,
    };
    struct network_pack_energy_t decoded_energy = {0};

    assert(network_pack_energy_pack(buffer,
                                    &energy,
                                    sizeof(buffer)) ==
           NETWORK_PACK_ENERGY_LENGTH);
    assert(network_pack_energy_unpack(&decoded_energy,
                                      buffer,
                                      NETWORK_PACK_ENERGY_LENGTH) == 0);
    assert(decoded_energy.energy_j == energy.energy_j);

    struct network_vehicle_motion_t motion = {
        .speed_kmh = 65535u,
        .trip_distance_m = 0xFFFFFFFFu,
        .motion_valid =
            NETWORK_VEHICLE_MOTION_MOTION_VALID_VALID_CHOICE,
        .seq_counter = 0xFFu,
    };
    struct network_vehicle_motion_t decoded_motion = {0};

    assert(network_vehicle_motion_pack(buffer,
                                       &motion,
                                       sizeof(buffer)) ==
           NETWORK_VEHICLE_MOTION_LENGTH);
    assert(network_vehicle_motion_unpack(&decoded_motion,
                                         buffer,
                                         NETWORK_VEHICLE_MOTION_LENGTH) == 0);
    assert(decoded_motion.speed_kmh == motion.speed_kmh);
    assert(decoded_motion.trip_distance_m == motion.trip_distance_m);
    assert(decoded_motion.motion_valid == motion.motion_valid);
    assert(decoded_motion.seq_counter == motion.seq_counter);
}

static void test_remaining_message_codecs(void)
{
    uint8_t buffer[8] = {0};

    struct network_aux_power_t aux_power = {
        .voltage_v = 1234u,
        .current_a = -2345,
    };
    struct network_aux_power_t decoded_aux_power = {0};
    assert(network_aux_power_pack(buffer, &aux_power, sizeof(buffer)) ==
           NETWORK_AUX_POWER_LENGTH);
    assert(network_aux_power_unpack(&decoded_aux_power,
                                    buffer,
                                    NETWORK_AUX_POWER_LENGTH) == 0);
    assert(decoded_aux_power.voltage_v == aux_power.voltage_v);
    assert(decoded_aux_power.current_a == aux_power.current_a);

    struct network_vehicle_time_t vehicle_time = {
        .year = 24u,
        .month = 9u,
        .day = 13u,
        .hour = 12u,
        .minute = 34u,
        .second = 56u,
        .gps_synced = 1u,
        .rtc_valid = 1u,
    };
    struct network_vehicle_time_t decoded_vehicle_time = {0};
    assert(network_vehicle_time_pack(buffer,
                                     &vehicle_time,
                                     sizeof(buffer)) ==
           NETWORK_VEHICLE_TIME_LENGTH);
    assert(network_vehicle_time_unpack(&decoded_vehicle_time,
                                       buffer,
                                       NETWORK_VEHICLE_TIME_LENGTH) == 0);
    assert(decoded_vehicle_time.year == vehicle_time.year);
    assert(decoded_vehicle_time.month == vehicle_time.month);
    assert(decoded_vehicle_time.day == vehicle_time.day);
    assert(decoded_vehicle_time.gps_synced == vehicle_time.gps_synced);

    struct network_gps_status_t gps_status = {
        .satellites = 19u,
        .fix_valid = 1u,
        .time_valid = 1u,
        .position_valid = 1u,
        .motion_valid = 1u,
        .seq_counter = 7u,
    };
    struct network_gps_status_t decoded_gps_status = {0};
    assert(network_gps_status_pack(buffer,
                                   &gps_status,
                                   sizeof(buffer)) ==
           NETWORK_GPS_STATUS_LENGTH);
    assert(network_gps_status_unpack(&decoded_gps_status,
                                     buffer,
                                     NETWORK_GPS_STATUS_LENGTH) == 0);
    assert(decoded_gps_status.satellites == gps_status.satellites);
    assert(decoded_gps_status.seq_counter == gps_status.seq_counter);

    struct network_gps_motion_t gps_motion = {
        .gps_speed_kmh = 4321u,
        .heading_deg = 12345u,
    };
    struct network_gps_motion_t decoded_gps_motion = {0};
    assert(network_gps_motion_pack(buffer,
                                   &gps_motion,
                                   sizeof(buffer)) ==
           NETWORK_GPS_MOTION_LENGTH);
    assert(network_gps_motion_unpack(&decoded_gps_motion,
                                     buffer,
                                     NETWORK_GPS_MOTION_LENGTH) == 0);
    assert(decoded_gps_motion.gps_speed_kmh == gps_motion.gps_speed_kmh);
    assert(decoded_gps_motion.heading_deg == gps_motion.heading_deg);

    struct network_motor_state_t motor_state = {
        .motor_speed_rpm = -1234,
        .sequencer_state = 4u,
        .motor_status = 0xA5A5A5A5u,
        .seq_counter = 9u,
    };
    struct network_motor_state_t decoded_motor_state = {0};
    assert(network_motor_state_pack(buffer,
                                    &motor_state,
                                    sizeof(buffer)) ==
           NETWORK_MOTOR_STATE_LENGTH);
    assert(network_motor_state_unpack(&decoded_motor_state,
                                      buffer,
                                      NETWORK_MOTOR_STATE_LENGTH) == 0);
    assert(decoded_motor_state.motor_speed_rpm == motor_state.motor_speed_rpm);
    assert(decoded_motor_state.motor_status == motor_state.motor_status);

    struct network_motor_current_t motor_current = {
        .iq_a = -1234,
        .id_a = 2345,
        .motor_current_a = 4567u,
    };
    struct network_motor_current_t decoded_motor_current = {0};
    assert(network_motor_current_pack(buffer,
                                      &motor_current,
                                      sizeof(buffer)) ==
           NETWORK_MOTOR_CURRENT_LENGTH);
    assert(network_motor_current_unpack(&decoded_motor_current,
                                        buffer,
                                        NETWORK_MOTOR_CURRENT_LENGTH) == 0);
    assert(decoded_motor_current.iq_a == motor_current.iq_a);
    assert(decoded_motor_current.id_a == motor_current.id_a);
    assert(decoded_motor_current.motor_current_a ==
           motor_current.motor_current_a);

    struct network_motor_voltage_t motor_voltage = {
        .dc_bus_voltage_v = 5432u,
        .vq_v = -1234,
        .vd_v = 2345,
    };
    struct network_motor_voltage_t decoded_motor_voltage = {0};
    assert(network_motor_voltage_pack(buffer,
                                      &motor_voltage,
                                      sizeof(buffer)) ==
           NETWORK_MOTOR_VOLTAGE_LENGTH);
    assert(network_motor_voltage_unpack(&decoded_motor_voltage,
                                        buffer,
                                        NETWORK_MOTOR_VOLTAGE_LENGTH) == 0);
    assert(decoded_motor_voltage.dc_bus_voltage_v ==
           motor_voltage.dc_bus_voltage_v);
    assert(decoded_motor_voltage.vq_v == motor_voltage.vq_v);
    assert(decoded_motor_voltage.vd_v == motor_voltage.vd_v);

    struct network_motor_faults_t motor_faults = {
        .fault_flags = 0x01234567u,
        .sw_faults = 0x89ABCDEFu,
    };
    struct network_motor_faults_t decoded_motor_faults = {0};
    assert(network_motor_faults_pack(buffer,
                                     &motor_faults,
                                     sizeof(buffer)) ==
           NETWORK_MOTOR_FAULTS_LENGTH);
    assert(network_motor_faults_unpack(&decoded_motor_faults,
                                       buffer,
                                       NETWORK_MOTOR_FAULTS_LENGTH) == 0);
    assert(decoded_motor_faults.fault_flags == motor_faults.fault_flags);
    assert(decoded_motor_faults.sw_faults == motor_faults.sw_faults);

    struct network_motor_estimator_t estimator = {
        .hall_angle_deg = 1234u,
        .flux_angle_deg = 2345u,
        .hall_speed_rpm = -3456,
        .flux_speed_rpm = 4567,
    };
    struct network_motor_estimator_t decoded_estimator = {0};
    assert(network_motor_estimator_pack(buffer,
                                        &estimator,
                                        sizeof(buffer)) ==
           NETWORK_MOTOR_ESTIMATOR_LENGTH);
    assert(network_motor_estimator_unpack(&decoded_estimator,
                                          buffer,
                                          NETWORK_MOTOR_ESTIMATOR_LENGTH) ==
           0);
    assert(decoded_estimator.hall_angle_deg == estimator.hall_angle_deg);
    assert(decoded_estimator.flux_speed_rpm == estimator.flux_speed_rpm);

    struct network_motor_phase_current_t phase_current = {
        .phase_u_current_a = -1234,
        .phase_v_current_a = 2345,
        .phase_w_current_a = -3456,
    };
    struct network_motor_phase_current_t decoded_phase_current = {0};
    assert(network_motor_phase_current_pack(buffer,
                                            &phase_current,
                                            sizeof(buffer)) ==
           NETWORK_MOTOR_PHASE_CURRENT_LENGTH);
    assert(network_motor_phase_current_unpack(
               &decoded_phase_current,
               buffer,
               NETWORK_MOTOR_PHASE_CURRENT_LENGTH) == 0);
    assert(decoded_phase_current.phase_u_current_a ==
           phase_current.phase_u_current_a);
    assert(decoded_phase_current.phase_v_current_a ==
           phase_current.phase_v_current_a);
    assert(decoded_phase_current.phase_w_current_a ==
           phase_current.phase_w_current_a);
}

int main(void)
{
    test_message_contract();
    test_inhibit_reason_round_trip();
    test_inhibit_reason_policy();
    test_aux_reserved_bits();
    test_signed_and_scaled_fields();
    test_wide_fields();
    test_remaining_message_codecs();
    return 0;
}
