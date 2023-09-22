#include <iostream>
#include "asb_ros2_control/canopen_motor_drive_receiver.h"

// This function gets called every time a value is written to the local object dictionary by an SDO or RPDO.
void CANOpenMotorDriveReceiverNode::OnWrite(uint16_t idx, uint8_t subidx) noexcept {

  // RPDO 1 (from motor drive node)
  if (idx == IDX_RPDO1 && subidx == SUB_IDX_RPDO1_4_battery_current_display) {
    std::cout << "[" << node_name_ << "]" << " RPDO 1 " << "COB-ID: " << (int)(uint32_t)(*this)[0x1400][0x01] << std::endl << std::endl;
    controller_temperature_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_1_controller_temperature]);
    motor_temperature_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_2_motor_temperature]);
    motor_RPM_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_3_motor_RPM]);
    battery_current_display_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_4_battery_current_display]);
    last_data_received_time_.store(std::chrono::steady_clock::now());
  }

  // RPDO 2 (from motor drive node)
  if (idx == IDX_RPDO2 && subidx == SUB_IDX_RPDO2_4_zero_speed_threshold) {
    std::cout << "[" << node_name_ << "]" << " RPDO 2" << "COB-ID: " << (int)(uint32_t)(*this)[0x1401][0x01] << std::endl << std::endl;
    motor_torque_.store((*this)[IDX_RPDO2][SUB_IDX_RPDO2_1_motor_torque]);
    BDI_percentage_.store((*this)[IDX_RPDO2][SUB_IDX_RPDO2_2_BDI_percentage]);
    keyswitch_voltage_.store((*this)[IDX_RPDO2][SUB_IDX_RPDO2_3_keyswitch_voltage]);
    zero_speed_threshold_.store((*this)[IDX_RPDO2][SUB_IDX_RPDO2_4_zero_speed_threshold]);
    last_data_received_time_.store(std::chrono::steady_clock::now());
  }

  // RPDO 3 (from motor drive node)
  if (idx == IDX_RPDO3 && subidx == SUB_IDX_RPDO3_1_motor_drive_status) {
    std::cout << "[" << node_name_ << "]" << " RPDO 3" << "COB-ID: " << (int)(uint32_t)(*this)[0x1402][0x01] << std::endl << std::endl;
    uint16_t motor_drive_status = (*this)[IDX_RPDO3][SUB_IDX_RPDO3_1_motor_drive_status];
    bool interlock_status_bit = (motor_drive_status >> BIT_IDX_interlock_status) & 1;
    interlock_status_.store(interlock_status_bit);
    last_data_received_time_.store(std::chrono::steady_clock::now());
  }

  // RPDO 4 (from motor drive node)
  if (idx == IDX_RPDO4 && subidx == SUB_IDX_RPDO4_1_rotor_position) {
    std::cout << "[" << node_name_ << "]" << " RPDO 4" << "COB-ID: " << (int)(uint32_t)(*this)[0x1403][0x01] << std::endl << std::endl;
    rotor_position_.store((*this)[IDX_RPDO4][SUB_IDX_RPDO4_1_rotor_position]);
    last_data_received_time_.store(std::chrono::steady_clock::now());
  }

}
