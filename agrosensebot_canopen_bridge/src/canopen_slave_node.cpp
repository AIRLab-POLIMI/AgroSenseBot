#include "canopen_slave_node.h"
#include "ros2_bridge_node.h"

void CANOpenSlaveNode::send_TPDO_1(uint8_t data) {
    RCLCPP_INFO(ros2_bridge_node_->get_logger(),
                "TPDO_1: writing 0x%X to 0x%X:%X", data, IDX_GCU_IS_ALIVE, SUB_IDX_GCU_is_alive);
    (*this)[IDX_GCU_IS_ALIVE][SUB_IDX_GCU_is_alive] = data;
    this->TpdoEvent(1);
}

void CANOpenSlaveNode::send_TPDO_2(int16_t right_speed_ref, int16_t left_speed_ref) {
    RCLCPP_INFO(ros2_bridge_node_->get_logger(),
                "TPDO_2: writing 0x%X to 0x%X:%X", right_speed_ref, IDX_MOTOR_SPEED_REF, SUB_IDX_RightSpeedRef);
    (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_RightSpeedRef] = right_speed_ref;
    RCLCPP_INFO(ros2_bridge_node_->get_logger(),
                "TPDO_2: writing 0x%X to 0x%X:%X", left_speed_ref, IDX_MOTOR_SPEED_REF, SUB_IDX_LeftSpeedRef);
    (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_LeftSpeedRef] = left_speed_ref;
    this->TpdoEvent(2);
}

// This function gets called every time a value is written to the local object dictionary by an SDO or RPDO.
void CANOpenSlaveNode::OnWrite(uint16_t idx, uint8_t subidx) noexcept {

    // RPDO 1 (from VCU node)
    if (idx == IDX_VCU_IS_ALIVE && subidx == SUB_IDX_control_mode) {

        RCLCPP_INFO(ros2_bridge_node_->get_logger(), "RPDO_1: Received data on idx: 0x%X:%X", idx, subidx);
        uint8_t VCU_is_alive = (*this)[IDX_VCU_IS_ALIVE][SUB_IDX_VCU_is_alive];
        bool VCU_is_alive_bit = (VCU_is_alive >> BIT_IDX_VCU_is_alive) & 1;
        bool VCU_safety_status_bit = (VCU_is_alive >> BIT_IDX_VCU_safety_status) & 1;
        uint8_t control_mode = (*this)[IDX_VCU_IS_ALIVE][SUB_IDX_control_mode];
        ros2_bridge_node_->vcu_alive_canopen_callback(VCU_is_alive_bit, VCU_safety_status_bit, control_mode);
    }

    // RPDO 2 (from MDL node)
    if (idx == IDX_MOTOR_DRIVE_DATA && subidx == SUB_IDX_MDL_battery_current_display) {

        RCLCPP_INFO(ros2_bridge_node_->get_logger(), "RPDO_2: Received data on idx: 0x%X:%X", idx, subidx);
        int16_t controller_temperature = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_controller_temperature];
        int16_t motor_temperature = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_motor_temperature];
        int16_t motor_RPM = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_motor_RPM];
        int16_t battery_current_display = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_battery_current_display];
        ros2_bridge_node_->motor_drive_left_canopen_callback(
                controller_temperature, motor_temperature, motor_RPM, battery_current_display);
    }

    // RPDO 3 (from MDR node)
    if (idx == IDX_MOTOR_DRIVE_DATA && subidx == SUB_IDX_MDR_battery_current_display) {

        RCLCPP_INFO(ros2_bridge_node_->get_logger(), "RPDO_3: Received data on idx: 0x%X:%X", idx, subidx);
        int16_t controller_temperature = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_controller_temperature];
        int16_t motor_temperature = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_motor_temperature];
        int16_t motor_RPM = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_motor_RPM];
        int16_t battery_current_display = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_battery_current_display];
        ros2_bridge_node_->motor_drive_right_canopen_callback(
                controller_temperature, motor_temperature, motor_RPM, battery_current_display);
    }

    // RPDO 4 (from FAN node)
    if (idx == IDX_MOTOR_DRIVE_DATA && subidx == SUB_IDX_FAN_battery_current_display) {

        RCLCPP_INFO(ros2_bridge_node_->get_logger(), "RPDO_4: Received data on idx: 0x%X:%X", idx, subidx);
        int16_t controller_temperature = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_controller_temperature];
        int16_t motor_temperature = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_motor_temperature];
        int16_t motor_RPM = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_motor_RPM];
        int16_t battery_current_display = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_battery_current_display];
        ros2_bridge_node_->motor_drive_fan_canopen_callback(
                controller_temperature, motor_temperature, motor_RPM, battery_current_display);
    }

}
