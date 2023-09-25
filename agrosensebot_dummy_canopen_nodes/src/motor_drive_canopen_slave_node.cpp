#include "motor_drive_canopen_slave_node.h"
#include "ros2_bridge_node.h"

void MotorDriveCANOpenSlaveNode::send_TPDO_1(int16_t controller_temperature, int16_t motor_temperature, int16_t motor_rpm, int16_t battery_current_display) {
    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_1: writing 0x%X to 0x%X:%X", controller_temperature, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_controller_temperature);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_controller_temperature] = controller_temperature;

//    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_1: writing 0x%X to 0x%X:%X", motor_temperature, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_motor_temperature);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_motor_temperature] = motor_temperature;

//    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_1: writing 0x%X to 0x%X:%X", motor_rpm, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_motor_RPM);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_motor_RPM] = motor_rpm;

//    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_1: writing 0x%X to 0x%X:%X", battery_current_display, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_battery_current_display);
    (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_battery_current_display] = battery_current_display;

    this->TpdoEvent(1);
}

