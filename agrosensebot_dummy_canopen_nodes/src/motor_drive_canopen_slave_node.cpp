#include "motor_drive_canopen_slave_node.h"
#include "ros2_bridge_node.h"

// TPDO1 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_TPDO1 0x3800
#define SUB_IDX_TPDO1_1_controller_temperature 0x01
#define SUB_IDX_TPDO1_2_motor_temperature 0x02
#define SUB_IDX_TPDO1_3_motor_RPM 0x03
#define SUB_IDX_TPDO1_4_battery_current_display 0x04

// TPDO2 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_TPDO2 0x3801
#define SUB_IDX_TPDO2_1_motor_torque 0x01
#define SUB_IDX_TPDO2_2_BDI_percentage 0x02
#define SUB_IDX_TPDO2_3_keyswitch_voltage 0x03
#define SUB_IDX_TPDO2_4_zero_speed_threshold 0x04

// TPDO3 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_TPDO3 0x3802
#define SUB_IDX_TPDO3_1_motor_drive_status 0x01
#define BIT_IDX_interlock_status 1

// TPDO4 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_TPDO4 0x3803
#define SUB_IDX_TPDO4_1_rotor_position 0x01


void MotorDriveCANOpenSlaveNode::send_TPDO_1(int16_t controller_temperature, int16_t motor_temperature, int16_t motor_rpm, int16_t battery_current_display) {
  RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[%s] TPDO_1", node_name_.c_str());
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_1_controller_temperature] = controller_temperature;
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_2_motor_temperature] = motor_temperature;
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_3_motor_RPM] = motor_rpm;
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_4_battery_current_display] = battery_current_display;

  this->TpdoEvent(1);
}


void MotorDriveCANOpenSlaveNode::send_TPDO_2(int16_t motor_torque, int16_t bdi_percentage, int16_t keyswitch_voltage, int16_t zero_speed_threshold) {
  RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[%s] TPDO_2", node_name_.c_str());
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_1_motor_torque] = motor_torque;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_2_BDI_percentage] = bdi_percentage;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_3_keyswitch_voltage] = keyswitch_voltage;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_4_zero_speed_threshold] = zero_speed_threshold;

  this->TpdoEvent(2);
}

void MotorDriveCANOpenSlaveNode::send_TPDO_3(bool interlock_status_bit) {
RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[%s] TPDO_3", node_name_.c_str());

  std::bitset<8> motor_drive_status_bitset;
  motor_drive_status_bitset[BIT_IDX_interlock_status] = interlock_status_bit;
  uint16_t motor_drive_status = motor_drive_status_bitset.to_ulong();
  (*this)[IDX_TPDO3][SUB_IDX_TPDO3_1_motor_drive_status] = motor_drive_status;

  this->TpdoEvent(3);
}

void MotorDriveCANOpenSlaveNode::send_TPDO_4(int32_t rotor_position) {
  RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[%s] TPDO_4", node_name_.c_str());
  (*this)[IDX_TPDO4][SUB_IDX_TPDO4_1_rotor_position] = rotor_position;

  this->TpdoEvent(4);
}

