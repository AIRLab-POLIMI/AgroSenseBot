#ifndef ASB_WEBOTS_DRIVER_HPP
#define ASB_WEBOTS_DRIVER_HPP

#include "rclcpp/macros.hpp"
#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace asb_webots_driver {
    class ASBWebotsDriver : public webots_ros2_driver::PluginInterface {
    public:
        void step() override;
        void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

    private:
        void cmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);

        rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;
        geometry_msgs::msg::Twist cmd_vel_msg;

        WbDeviceTag right_motor;
        WbDeviceTag left_motor;
    };
} // namespace asb_webots_driver
#endif