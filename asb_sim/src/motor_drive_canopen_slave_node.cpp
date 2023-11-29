#include "motor_drive_canopen_slave_node.h"
#include "asb_system_test_node.h"

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


void MotorDriveCANOpenSlaveNode::timer() {
  send_TPDO_1();
  send_TPDO_2();
  send_TPDO_3();
  send_TPDO_4();
}

void MotorDriveCANOpenSlaveNode::set_TPDO_1(int16_t controller_temperature, int16_t motor_temperature, int16_t motor_rpm, int16_t battery_current_display) {
  controller_temperature_ = controller_temperature;
  motor_temperature_ = motor_temperature;
  motor_rpm_ = motor_rpm;
  battery_current_display_ = battery_current_display;
  new_TPDO_1_ = true;
}

void MotorDriveCANOpenSlaveNode::send_TPDO_1() {
  if(!new_TPDO_1_) return;
  new_TPDO_1_ = false;
//  RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[%s] TPDO_1", node_name_.c_str());
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_1_controller_temperature] = controller_temperature_;
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_2_motor_temperature] = motor_temperature_;
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_3_motor_RPM] = motor_rpm_;
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_4_battery_current_display] = battery_current_display_;

  this->TpdoEvent(1);
}

void MotorDriveCANOpenSlaveNode::set_TPDO_2(int16_t motor_torque, int16_t bdi_percentage, int16_t keyswitch_voltage, int16_t zero_speed_threshold) {
  motor_torque_ = motor_torque;
  bdi_percentage_ = bdi_percentage;
  keyswitch_voltage_ = keyswitch_voltage;
  zero_speed_threshold_ = zero_speed_threshold;
  new_TPDO_2_ = true;
}

void MotorDriveCANOpenSlaveNode::send_TPDO_2() {
  if(!new_TPDO_2_) return;
  new_TPDO_2_ = false;
//  RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[%s] TPDO_2", node_name_.c_str());
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_1_motor_torque] = motor_torque_;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_2_BDI_percentage] = bdi_percentage_;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_3_keyswitch_voltage] = keyswitch_voltage_;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_4_zero_speed_threshold] = zero_speed_threshold_;

  this->TpdoEvent(2);
}

void MotorDriveCANOpenSlaveNode::set_TPDO_3(bool interlock_status_bit) {
  interlock_status_bit_ = interlock_status_bit;
  new_TPDO_3_ = true;
}

void MotorDriveCANOpenSlaveNode::send_TPDO_3() {
  if(!new_TPDO_3_) return;
  new_TPDO_3_ = false;
//  RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[%s] TPDO_3", node_name_.c_str());

  std::bitset<8> motor_drive_status_bitset;
  motor_drive_status_bitset[BIT_IDX_interlock_status] = interlock_status_bit_;
  uint16_t motor_drive_status = motor_drive_status_bitset.to_ulong();
  (*this)[IDX_TPDO3][SUB_IDX_TPDO3_1_motor_drive_status] = motor_drive_status;

  this->TpdoEvent(3);
}

void MotorDriveCANOpenSlaveNode::set_TPDO_4(int32_t rotor_position_raw) {
  rotor_position_ = rotor_position_raw;
  new_TPDO_4_ = true;
}

void MotorDriveCANOpenSlaveNode::send_TPDO_4() {
  if(!new_TPDO_4_) return;
  new_TPDO_4_ = false;
//  RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[%s] TPDO_4", node_name_.c_str());
  (*this)[IDX_TPDO4][SUB_IDX_TPDO4_1_rotor_position] = rotor_position_;

  this->TpdoEvent(4);
}
