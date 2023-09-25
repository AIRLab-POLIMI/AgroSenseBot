#include "motor_drive_canopen_slave_node.h"
#include "ros2_bridge_node.h"

// PDO register indices

// TPDO2
#define IDX_TPDO1_MOTOR_DRIVE_DATA 0x2110
#define SUB_IDX_TPDO1_1_controller_temperature 0x01
#define SUB_IDX_TPDO1_2_motor_temperature 0x02
#define SUB_IDX_TPDO1_3_motor_RPM 0x03
#define SUB_IDX_TPDO1_4_battery_current_display 0x04


void MotorDriveCANOpenSlaveNode::send_TPDO_1(int16_t controller_temperature, int16_t motor_temperature, int16_t motor_rpm, int16_t battery_current_display) {
    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_1: writing 0x%X to 0x%X:%X", controller_temperature, IDX_TPDO1_MOTOR_DRIVE_DATA, SUB_IDX_TPDO1_1_controller_temperature);
    (*this)[IDX_TPDO1_MOTOR_DRIVE_DATA][SUB_IDX_TPDO1_1_controller_temperature] = controller_temperature;

//    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_1: writing 0x%X to 0x%X:%X", motor_temperature, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_motor_temperature);
    (*this)[IDX_TPDO1_MOTOR_DRIVE_DATA][SUB_IDX_TPDO1_2_motor_temperature] = motor_temperature;

//    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_1: writing 0x%X to 0x%X:%X", motor_rpm, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_motor_RPM);
    (*this)[IDX_TPDO1_MOTOR_DRIVE_DATA][SUB_IDX_TPDO1_3_motor_RPM] = motor_rpm;

//    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "TPDO_1: writing 0x%X to 0x%X:%X", battery_current_display, IDX_MOTOR_DRIVE_DATA, SUB_IDX_MDL_battery_current_display);
    (*this)[IDX_TPDO1_MOTOR_DRIVE_DATA][SUB_IDX_TPDO1_4_battery_current_display] = battery_current_display;

    this->TpdoEvent(1);
}

