#ifndef ASB_WEBOTS_DRIVER_HPP
#define ASB_WEBOTS_DRIVER_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
//#include "geometry_msgs/msg/twist.hpp"
//#include "nav_msgs/msg/odometry.hpp"
#include "asb_msgs/msg/sim_state_cmd.hpp"
#include "asb_msgs/msg/sim_state.hpp"

#include "webots_ros2_driver/PluginInterface.hpp"
#include "webots_ros2_driver/WebotsNode.hpp"

namespace asb_webots_driver {
    class ASBWebotsDriver : public webots_ros2_driver::PluginInterface {
    public:
        void step() override;
        void init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) override;

    private:
        void sim_state_cmd_callback(const asb_msgs::msg::SimStateCmd::SharedPtr msg);

        rclcpp::Subscription<asb_msgs::msg::SimStateCmd>::SharedPtr sim_state_cmd_subscriber_;
        rclcpp::Publisher<asb_msgs::msg::SimState>::SharedPtr sim_state_publisher_;
        asb_msgs::msg::SimStateCmd sim_state_cmd_msg;

        WbDeviceTag right_motor;
        WbDeviceTag left_motor;
    };
} // namespace asb_webots_driver
#endif