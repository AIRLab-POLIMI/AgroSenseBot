#ifndef ASB_WEBOTS_DRIVER_HPP
#define ASB_WEBOTS_DRIVER_HPP

#include "rclcpp/macros.hpp"
#include "rclcpp/rclcpp.hpp"
#include "asb_msgs/msg/sim_state_cmd.hpp"
#include "asb_msgs/msg/sim_state.hpp"
#include "sensor_msgs/msg/nav_sat_fix.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/imu.hpp"

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
        rclcpp::Publisher<sensor_msgs::msg::NavSatFix>::SharedPtr gnss_publisher_;
        rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr inertial_unit_publisher_;
        rclcpp::Publisher<sensor_msgs::msg::Imu>::SharedPtr imu_publisher_;
        asb_msgs::msg::SimStateCmd sim_state_cmd_msg_;

        std::string sim_state_topic_;
        std::string sim_state_cmd_topic_;

        WbDeviceTag gnss_;
        rclcpp::Time last_gnss_fix_;
        std::string gnss_topic_;
        std::string gnss_frame_id_;
        double gnss_update_rate_;
        std::vector<double> gnss_covariance_diagonal_;

        WbDeviceTag inertial_unit_;
        rclcpp::Time last_inertial_unit_update_;
        std::string inertial_unit_topic_;
        std::string inertial_unit_parent_frame_id_;
        std::string inertial_unit_child_frame_id_;
        double inertial_unit_update_rate_;
        std::vector<double> inertial_unit_covariance_diagonal_;

        double imu_update_rate_;
        rclcpp::Time last_imu_update_;
        std::string imu_topic_;
        std::string imu_frame_id_;
        WbDeviceTag gyro_;
        std::string gyro_frame_id_;
        std::vector<double> gyro_covariance_diagonal_;

        WbDeviceTag right_motor_;
        WbDeviceTag left_motor_;
    };
} // namespace asb_webots_driver
#endif