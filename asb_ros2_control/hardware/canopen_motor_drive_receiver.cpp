#include <iostream>
#include "asb_ros2_control/canopen_motor_drive_receiver.h"

// RPDO1 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_RPDO1 0x3000
#define SUB_IDX_RPDO1_1_controller_temperature 0x01
#define SUB_IDX_RPDO1_2_motor_temperature 0x02
#define SUB_IDX_RPDO1_3_motor_RPM 0x03
#define SUB_IDX_RPDO1_4_battery_current_display 0x04

// RPDO2 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_RPDO2 0x3001
#define SUB_IDX_RPDO2_1_motor_torque 0x01
#define SUB_IDX_RPDO2_2_BDI_percentage 0x02
#define SUB_IDX_RPDO2_3_keyswitch_voltage 0x03
#define SUB_IDX_RPDO2_4_zero_speed_threshold 0x04

// RPDO3 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_RPDO3 0x3002
#define SUB_IDX_RPDO3_1_motor_drive_status 0x01
#define BIT_IDX_interlock_status 1

// RPDO4 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_RPDO4 0x3003
#define SUB_IDX_RPDO4_1_rotor_position 0x01

// This function gets called every time an RPDO is received.
void CANOpenMotorDriveReceiverNode::OnRpdo(int num, ::std::error_code /*ec*/, const void* /*p*/, ::std::size_t /*n*/) noexcept {

  // RPDO 1 (from motor drive node)
  if (num == 1) {
    controller_temperature_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_1_controller_temperature]);
    motor_temperature_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_2_motor_temperature]);
    motor_RPM_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_3_motor_RPM]);
    battery_current_display_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_4_battery_current_display]);
    last_data_received_time_.store(std::chrono::steady_clock::now());
  }

  // RPDO 2 (from motor drive node)
  if (num == 2) {
    motor_torque_.store((*this)[IDX_RPDO2][SUB_IDX_RPDO2_1_motor_torque]);
    BDI_percentage_.store((*this)[IDX_RPDO2][SUB_IDX_RPDO2_2_BDI_percentage]);
    keyswitch_voltage_.store((*this)[IDX_RPDO2][SUB_IDX_RPDO2_3_keyswitch_voltage]);
    zero_speed_threshold_.store((*this)[IDX_RPDO2][SUB_IDX_RPDO2_4_zero_speed_threshold]);
    last_data_received_time_.store(std::chrono::steady_clock::now());
  }

  // RPDO 3 (from motor drive node)
  if (num == 3) {
    uint16_t motor_drive_status = (*this)[IDX_RPDO3][SUB_IDX_RPDO3_1_motor_drive_status];
    bool interlock_status_bit = (motor_drive_status >> BIT_IDX_interlock_status) & 1;
    interlock_status_.store(interlock_status_bit);
    last_data_received_time_.store(std::chrono::steady_clock::now());
  }

  // RPDO 4 (from motor drive node)
  if (num == 4) {
    rotor_position_.store((*this)[IDX_RPDO4][SUB_IDX_RPDO4_1_rotor_position]);
    last_data_received_time_.store(std::chrono::steady_clock::now());
  }

}
