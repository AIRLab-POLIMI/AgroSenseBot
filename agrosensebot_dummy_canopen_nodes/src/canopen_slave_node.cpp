#include "canopen_slave_node.h"
#include "ros2_bridge_node.h"

void CANOpenSlaveNode::send_TPDO_1(bool VCU_is_alive_bit, bool VCU_safety_status_bit, uint8_t control_mode) {
    std::bitset<8> VCU_is_alive_bitset;
    VCU_is_alive_bitset[0] = VCU_is_alive_bit;
    VCU_is_alive_bitset[1] = VCU_safety_status_bit;
    uint8_t VCU_is_alive = VCU_is_alive_bitset.to_ulong();

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_1: writing 0x%X to 0x%X:%X", VCU_is_alive, IDX_VCU_IS_ALIVE, SUB_IDX_VCU_is_alive);
    (*this)[IDX_VCU_IS_ALIVE][SUB_IDX_VCU_is_alive] = VCU_is_alive;

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_1: writing 0x%X to 0x%X:%X", control_mode, IDX_VCU_IS_ALIVE, SUB_IDX_control_mode);
    (*this)[IDX_VCU_IS_ALIVE][SUB_IDX_control_mode] = control_mode;

    this->TpdoEvent(1);
}

void CANOpenSlaveNode::send_TPDO_2(int16_t controller_temperature, int16_t motor_temperature, int16_t motor_rpm, int16_t battery_current_display) {
    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_2: writing 0x%X to 0x%X:%X", controller_temperature, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_controller_temperature);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_controller_temperature] = controller_temperature;

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_2: writing 0x%X to 0x%X:%X", motor_temperature, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_motor_temperature);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_motor_temperature] = motor_temperature;

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_2: writing 0x%X to 0x%X:%X", motor_rpm, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_motor_RPM);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_motor_RPM] = motor_rpm;

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_2: writing 0x%X to 0x%X:%X", battery_current_display, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_battery_current_display);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_battery_current_display] = battery_current_display;

    this->TpdoEvent(2);
}

void CANOpenSlaveNode::send_TPDO_3(int16_t controller_temperature, int16_t motor_temperature, int16_t motor_rpm, int16_t battery_current_display) {
    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_3: writing 0x%X to 0x%X:%X", controller_temperature, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDR_controller_temperature);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_controller_temperature] = controller_temperature;

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_3: writing 0x%X to 0x%X:%X", motor_temperature, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDR_motor_temperature);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_motor_temperature] = motor_temperature;

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_3: writing 0x%X to 0x%X:%X", motor_rpm, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDR_motor_RPM);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_motor_RPM] = motor_rpm;

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_3: writing 0x%X to 0x%X:%X", battery_current_display, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDR_battery_current_display);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_battery_current_display] = battery_current_display;

    this->TpdoEvent(3);
}

void CANOpenSlaveNode::send_TPDO_4(int16_t controller_temperature, int16_t motor_temperature, int16_t motor_rpm, int16_t battery_current_display) {
    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_4: writing 0x%X to 0x%X:%X", controller_temperature, IDX_MOTOR_DRIVE_DATA, SUB_IDX_FAN_controller_temperature);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_controller_temperature] = controller_temperature;

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_4: writing 0x%X to 0x%X:%X", motor_temperature, IDX_MOTOR_DRIVE_DATA, SUB_IDX_FAN_motor_temperature);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_motor_temperature] = motor_temperature;

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_4: writing 0x%X to 0x%X:%X", motor_rpm, IDX_MOTOR_DRIVE_DATA, SUB_IDX_FAN_motor_RPM);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_motor_RPM] = motor_rpm;

    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_4: writing 0x%X to 0x%X:%X", battery_current_display, IDX_MOTOR_DRIVE_DATA, SUB_IDX_FAN_battery_current_display);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_battery_current_display] = battery_current_display;

    this->TpdoEvent(4);
}

// This function gets called every time a value is written to the local object dictionary by an SDO or RPDO.
void CANOpenSlaveNode::OnWrite(uint16_t idx, uint8_t subidx) noexcept {

    // RPDO 1 (from GCU node)
    if (idx == IDX_GCU_IS_ALIVE && subidx == SUB_IDX_GCU_is_alive) {

        RCLCPP_INFO(ros2_bridge_node_->get_logger(), "RPDO_1: Received data on idx: 0x%X:%X", idx, subidx);
        uint8_t GCU_is_alive = (*this)[IDX_GCU_IS_ALIVE][SUB_IDX_GCU_is_alive];
        bool GCU_is_alive_bit = (GCU_is_alive >> BIT_IDX_GCU_is_alive) & 1;
        bool GCU_is_ready_bit = (GCU_is_alive >> BIT_IDX_GCU_is_ready) & 1;
        ros2_bridge_node_->gcu_alive_canopen_callback(GCU_is_alive_bit, GCU_is_ready_bit);
    }

    // RPDO 2 (from GCU node)
    if (idx == IDX_MOTOR_SPEED_REF && subidx == SUB_IDX_LeftSpeedRef) {

        RCLCPP_INFO(ros2_bridge_node_->get_logger(), "RPDO_2: Received data on idx: 0x%X:%X", idx, subidx);
        int16_t right_speed_ref = (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_RightSpeedRef];
        int16_t left_speed_ref = (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_LeftSpeedRef];
        ros2_bridge_node_->speed_ref_canopen_callback(right_speed_ref, left_speed_ref);
    }

}
