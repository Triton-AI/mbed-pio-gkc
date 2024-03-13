#ifndef VESC_CAN_TOOLS_HPP_
#define VESC_CAN_TOOLS_HPP_

#include <cstdint>
#include <iostream>
#include "mbed.h"
#include "config.hpp"


namespace tritonai::gkc {

    CAN can1(CAN1_RX, CAN1_TX, CAN1_BAUDRATE);
    CAN can2(CAN2_RX, CAN2_TX, CAN2_BAUDRATE);

    static void can_transmit_eid(uint32_t id, const uint8_t *data, uint8_t len) {
        CANMessage *cMsg;
        cMsg = new CANMessage(id, data, len, CANData, CANExtended);

        if (!can2.write(*cMsg)) {
            can2.reset();
            can2.frequency(CAN2_BAUDRATE);

        }
        delete cMsg;

    }

    typedef enum {
        CAN_PACKET_SET_DUTY = 0,
        CAN_PACKET_SET_CURRENT,
        CAN_PACKET_SET_CURRENT_BRAKE,
        CAN_PACKET_SET_RPM,
        CAN_PACKET_SET_POS,
        CAN_PACKET_SET_CURRENT_REL = 10,
        CAN_PACKET_SET_CURRENT_BRAKE_REL,
        CAN_PACKET_SET_CURRENT_HANDBRAKE,
        CAN_PACKET_SET_CURRENT_HANDBRAKE_REL,
        CAN_PACKET_MAKE_ENUM_32_BITS = 0xFFFFFFFF,
    } CAN_PACKET_ID;

    void buffer_append_int16(uint8_t* buffer, int16_t number, int32_t *index) {
        buffer[(*index)++] = number >> 8;
        buffer[(*index)++] = number;
    }

    void buffer_append_int32(uint8_t* buffer, int32_t number, int32_t *index) {
        buffer[(*index)++] = number >> 24;
        buffer[(*index)++] = number >> 16;
        buffer[(*index)++] = number >> 8;
        buffer[(*index)++] = number;
    }

    void buffer_append_float16(uint8_t* buffer, float number, float scale, int32_t *index) {
        buffer_append_int16(buffer, (int16_t)(number * scale), index);
    }

    void buffer_append_float32(uint8_t* buffer, float number, float scale, int32_t *index) {
        buffer_append_int32(buffer, (int32_t)(number * scale), index);
    }

    // Message sending functions.
    void comm_can_set_duty(uint8_t controller_id, float duty) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_int32(buffer, (int32_t)(duty * 100000.0), &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_DUTY << 8), buffer, send_index);
    }

    void comm_can_set_current(uint8_t controller_id, float current) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
    }

    void comm_can_set_current_brake(uint8_t controller_id, float current) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE << 8), buffer, send_index);
    }

    void comm_can_set_rpm(uint8_t controller_id, float rpm) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_int32(buffer, (int32_t)rpm, &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_RPM << 8), buffer, send_index);
    }

    void comm_can_set_pos(uint8_t controller_id, float pos) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_int32(buffer, (int32_t)(pos * 1000000.0), &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_POS << 8), buffer, send_index);
    }

    void comm_can_set_current_rel(uint8_t controller_id, float current_rel) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_float32(buffer, current_rel, 1e5, &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
    }

    /**
     * Same as above, but also sets the off delay. Note that this command uses 6 bytes now. The off delay is useful to set to keep the current controller running for a while even after setting currents below the minimum current.
     */
    void comm_can_set_current_off_delay(uint8_t controller_id, float current, float off_delay) {
        int32_t send_index = 0;
        uint8_t buffer[6];
        buffer_append_int32(buffer, (int32_t)(current * 1000.0), &send_index);
        buffer_append_float16(buffer, off_delay, 1e3, &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT << 8), buffer, send_index);
    }

    void comm_can_set_current_rel_off_delay(uint8_t controller_id, float current_rel, float off_delay) {
        int32_t send_index = 0;
        uint8_t buffer[6];
        buffer_append_float32(buffer, current_rel, 1e5, &send_index);
        buffer_append_float16(buffer, off_delay, 1e3, &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_REL << 8), buffer, send_index);
    }

    void comm_can_set_current_brake_rel(uint8_t controller_id, float current_rel) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_float32(buffer, current_rel, 1e5, &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_BRAKE_REL << 8), buffer, send_index);
    }

    void comm_can_set_handbrake(uint8_t controller_id, float current) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_float32(buffer, current, 1e3, &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE << 8), buffer, send_index);
    }

    void comm_can_set_handbrake_rel(uint8_t controller_id, float current_rel) {
        int32_t send_index = 0;
        uint8_t buffer[4];
        buffer_append_float32(buffer, current_rel, 1e5, &send_index);
        can_transmit_eid(controller_id |
                ((uint32_t)CAN_PACKET_SET_CURRENT_HANDBRAKE_REL << 8), buffer, send_index);
    }

    void comm_can_set_speed(float speed_ms) { // in m/s
        float motor_poles = 5.0;
        float gear_ratio = 59.0/22.0;
        float wheel_circumference = 0.85; // in meters
        float speed_to_erpm = speed_ms * motor_poles * gear_ratio / wheel_circumference * 60.0 ;
        // std::cout << "Speed to erpm: " << (int)(speed_to_erpm) << std::endl;
        comm_can_set_rpm(THROTTLE_CAN_ID, speed_to_erpm);
    }

    void comm_can_set_angle(float angle) { // in radians
        
        float rad_to_deg = 180.0 / 3.14159265358979323846*angle;
        std::cout << "Angle to deg: " << (int)(rad_to_deg) << std::endl;
        comm_can_set_pos(STEER_CAN_ID, rad_to_deg);
    }
        
} // namespace tritonai::gkc

#endif // VESC_CAN_TOOLS_HPP_