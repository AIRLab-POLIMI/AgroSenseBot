#include "canopen_slave_node.h"
#include "ros2_bridge_node.h"

void CANOpenSlaveNode::send_TPDO(uint32_t data) {
    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "send_TPDO: writing 0x%X to 0x4001:0", data);
    (*this)[0x4001][0] = data;
    this->TpdoEvent(0);
}

// This function gets called every time a value is written to the local object dictionary by an SDO or RPDO.
void CANOpenSlaveNode::OnWrite(uint16_t idx, uint8_t subidx) noexcept {
    uint32_t val = (*this)[idx][subidx];
    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "OnWrite: received 0x%X on idx: 0x%X:%X", val, idx, subidx);

    if (
            (idx == IDX_FAN_controller_temperature && subidx == SUB_IDX_FAN_controller_temperature) ||
            (idx == IDX_FAN_motor_temperature && subidx == SUB_IDX_FAN_motor_temperature) ||
            (idx == IDX_FAN_motor_RPM && subidx == SUB_IDX_FAN_motor_RPM) ||
            (idx == IDX_FAN_battery_current_display && subidx == SUB_IDX_FAN_battery_current_display) ||
            ) {

        uint16_t FAN_controller_temperature = (*this)[IDX_FAN_controller_temperature][SUB_IDX_FAN_controller_temperature];
        uint16_t FAN_motor_temperature = (*this)[IDX_FAN_motor_temperature][SUB_IDX_FAN_motor_temperature];
        uint16_t FAN_motor_RPM = (*this)[IDX_FAN_motor_RPM][SUB_IDX_FAN_motor_RPM];
        uint16_t FAN_battery_current_display = (*this)[IDX_FAN_battery_current_display][SUB_IDX_FAN_battery_current_display];
        ros2_bridge_node_->RPDO_4_callback(
                FAN_controller_temperature, FAN_motor_temperature, FAN_motor_RPM,FAN_battery_current_display);
    }
}
