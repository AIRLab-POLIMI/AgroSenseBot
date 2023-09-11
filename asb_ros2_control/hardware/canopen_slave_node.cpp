#include "asb_ros2_control/canopen_slave_node.h"

void CANOpenSlaveNode::send_TPDO_1(uint8_t data) {
  (*this)[IDX_GCU_IS_ALIVE][SUB_IDX_GCU_is_alive] = data;
  this->TpdoEvent(1);
}

void CANOpenSlaveNode::send_TPDO_2(int16_t right_speed_ref, int16_t left_speed_ref) {
  (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_RightSpeedRef] = right_speed_ref;
  (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_LeftSpeedRef] = left_speed_ref;
  this->TpdoEvent(2);
}

// This function gets called every time a value is written to the local object dictionary by an SDO or RPDO.
void CANOpenSlaveNode::OnWrite(uint16_t idx, uint8_t subidx) noexcept {

  // RPDO 1 (from VCU node)
  if (idx == IDX_VCU_IS_ALIVE && subidx == SUB_IDX_control_mode) {
    uint8_t VCU_is_alive = (*this)[IDX_VCU_IS_ALIVE][SUB_IDX_VCU_is_alive];
    VCU_is_alive_bit_.store((VCU_is_alive >> BIT_IDX_VCU_is_alive) & 1);
    // TODO set last_VCU_is_alive_bit_change_ time and when we read the control state use it to compute the hardware interface state value
    // std::cout << "VCU_is_alive_bit_ " << VCU_is_alive_bit_ << std::endl;
    VCU_safety_status_bit_.store((VCU_is_alive >> BIT_IDX_VCU_safety_status) & 1);
    control_mode_.store((*this)[IDX_VCU_IS_ALIVE][SUB_IDX_control_mode]);
  }

  // RPDO 2 (from MDL node)
  if (idx == IDX_MOTOR_DRIVE_DATA && subidx == SUB_IDX_MDL_battery_current_display) {
    controller_temperature_left_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_controller_temperature]);
    motor_temperature_left_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_motor_temperature]);
    motor_RPM_left_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_motor_RPM]);
    battery_current_display_left_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_battery_current_display]);
  }

  // RPDO 3 (from MDR node)
  if (idx == IDX_MOTOR_DRIVE_DATA && subidx == SUB_IDX_MDR_battery_current_display) {
    controller_temperature_right_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_controller_temperature]);
    motor_temperature_right_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_motor_temperature]);
    motor_RPM_right_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_motor_RPM]);
    battery_current_display_right_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_battery_current_display]);
  }

  // RPDO 4 (from FAN node)
  if (idx == IDX_MOTOR_DRIVE_DATA && subidx == SUB_IDX_FAN_battery_current_display) {
    controller_temperature_fan_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_controller_temperature]);
    motor_temperature_fan_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_motor_temperature]);
    motor_RPM_fan_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_motor_RPM]);
    battery_current_display_fan_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_battery_current_display]);
  }

}
