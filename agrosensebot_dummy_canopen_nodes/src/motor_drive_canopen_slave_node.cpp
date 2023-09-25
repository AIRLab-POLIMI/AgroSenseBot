#include "motor_drive_canopen_slave_node.h"
#include "ros2_bridge_node.h"

// PDO register indices
#define IDX_MOTOR_DRIVE_DATA 0x2110

// TPDO2
#define SUB_IDX_MDL_controller_temperature 0x01
#define SUB_IDX_MDL_motor_temperature 0x02
#define SUB_IDX_MDL_motor_RPM 0x03
#define SUB_IDX_MDL_battery_current_display 0x04

//RPDO1
#define SUB_IDX_GCU_is_alive 0x01

#define BIT_IDX_GCU_is_alive 0
#define BIT_IDX_GCU_is_ready 1

//RPDO2
#define SUB_IDX_RightSpeedRef 0x01
#define SUB_IDX_LeftSpeedRef 0x02

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

