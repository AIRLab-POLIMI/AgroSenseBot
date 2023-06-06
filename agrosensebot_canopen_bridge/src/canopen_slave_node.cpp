#include "canopen_slave_node.h"
#include "ros2_bridge_node.h"

void CANOpenSlaveNode::send_TPDO_1(uint8_t data) {
    RCLCPP_INFO(
            ros2_bridge_node_->get_logger(),
            "send_TPDO_1: writing 0x%X to 0x%X:%X", data, IDX_GCU_IS_ALIVE, SUB_IDX_GCU_is_alive);
    (*this)[IDX_GCU_IS_ALIVE][SUB_IDX_GCU_is_alive] = data;
    this->TpdoEvent(1);
}

void CANOpenSlaveNode::send_TPDO_2(int16_t right_speed_ref, int16_t left_speed_ref) {
    RCLCPP_INFO(ros2_bridge_node_->get_logger(),
                "send_TPDO_2: writing 0x%X to 0x%X:%X", right_speed_ref, IDX_MOTOR_SPEED_REF, SUB_IDX_RightSpeedRef);
    (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_RightSpeedRef] = right_speed_ref;
  RCLCPP_INFO(ros2_bridge_node_->get_logger(),
              "send_TPDO_2: writing 0x%X to 0x%X:%X", left_speed_ref, IDX_MOTOR_SPEED_REF, SUB_IDX_LeftSpeedRef);
    (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_LeftSpeedRef] = left_speed_ref;
    this->TpdoEvent(2);
}

// This function gets called every time a value is written to the local object dictionary by an SDO or RPDO.
void CANOpenSlaveNode::OnWrite(uint16_t idx, uint8_t subidx) noexcept {
    uint32_t val = (*this)[idx][subidx];
    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "OnWrite: received 0x%X on idx: 0x%X:%X", val, idx, subidx);

    if (idx == IDX_MOTOR_DRIVE_DATA && (
            (subidx == SUB_IDX_FAN_controller_temperature) ||
            (subidx == SUB_IDX_FAN_motor_temperature) ||
            (subidx == SUB_IDX_FAN_motor_RPM) ||
            (subidx == SUB_IDX_FAN_battery_current_display)
            )) {

        int16_t FAN_controller_temperature = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_controller_temperature];
        int16_t FAN_motor_temperature = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_motor_temperature];
        int16_t FAN_motor_RPM = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_motor_RPM];
        int16_t FAN_battery_current_display = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_battery_current_display];
      ros2_bridge_node_->motor_drive_canopen_callback(
              FAN_controller_temperature, FAN_motor_temperature, FAN_motor_RPM, FAN_battery_current_display);
    }
}
