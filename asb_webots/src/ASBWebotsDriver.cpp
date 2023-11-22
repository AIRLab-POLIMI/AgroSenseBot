#include "asb_webots/ASBWebotsDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <webots/motor.h>
#include <webots/robot.h>

#define GEARBOX_REDUCTION_RATIO 40.61

namespace asb_webots_driver {
    void ASBWebotsDriver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {

        right_motor = wb_robot_get_device("right motor");
        left_motor = wb_robot_get_device("left motor");

        wb_motor_set_position(left_motor, INFINITY);
        wb_motor_set_velocity(left_motor, 0.0);

        wb_motor_set_position(right_motor, INFINITY);
        wb_motor_set_velocity(right_motor, 0.0);

        sim_state_cmd_subscriber_ = node->create_subscription<asb_msgs::msg::SimStateCmd>(
                "/system_test/state_cmd", rclcpp::SensorDataQoS().reliable(),
                std::bind(&ASBWebotsDriver::sim_state_cmd_callback, this, std::placeholders::_1));

        sim_state_publisher_ = node->create_publisher<asb_msgs::msg::SimState>(
            "/system_test/state", rclcpp::SensorDataQoS().reliable());
    }

    void ASBWebotsDriver::sim_state_cmd_callback(const asb_msgs::msg::SimStateCmd::SharedPtr msg) {
        sim_state_cmd_msg = *msg;
    }

    void ASBWebotsDriver::step() {
        wb_motor_set_velocity(left_motor, sim_state_cmd_msg.left_motor_speed_ref / GEARBOX_REDUCTION_RATIO);
        wb_motor_set_velocity(right_motor, sim_state_cmd_msg.right_motor_speed_ref / GEARBOX_REDUCTION_RATIO);

        auto sim_state_msg = asb_msgs::msg::SimState();
        sim_state_msg.stamp = rclcpp::Clock().now();
        sim_state_msg.left_motor_speed = wb_motor_get_velocity(left_motor) * GEARBOX_REDUCTION_RATIO;
        sim_state_msg.right_motor_speed = wb_motor_get_velocity(right_motor) * GEARBOX_REDUCTION_RATIO;
        sim_state_msg.fan_motor_speed = sim_state_cmd_msg.fan_motor_speed_ref;
        sim_state_publisher_->publish(sim_state_msg);
    }
} // namespace asb_webots_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(asb_webots_driver::ASBWebotsDriver, webots_ros2_driver::PluginInterface)
