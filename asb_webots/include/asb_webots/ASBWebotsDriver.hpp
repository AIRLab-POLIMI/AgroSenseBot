#ifndef ASB_WEBOTS_DRIVER_HPP
#define ASB_WEBOTS_DRIVER_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "asb_msgs/msg/sim_state_cmd.hpp"
#include "asb_msgs/msg/sim_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"

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
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr nav_sat_fix_publisher_;
        asb_msgs::msg::SimStateCmd sim_state_cmd_msg_;

        WbDeviceTag right_motor_;
        WbDeviceTag left_motor_;
        WbDeviceTag gnss_;
    };
} // namespace asb_webots_driver
#endif