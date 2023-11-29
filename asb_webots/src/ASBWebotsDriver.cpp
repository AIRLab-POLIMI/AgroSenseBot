#include "asb_webots/ASBWebotsDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>
#include <chrono>

using namespace std::chrono_literals;
using namespace sensor_msgs::msg;

#define GEARBOX_REDUCTION_RATIO 40.61
#define TRACK_WHEEL_RADIUS 0.151

namespace asb_webots_driver {
    void ASBWebotsDriver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "initializing webots driver");

        node->declare_parameter<std::string>("sim_state_topic", "sim_state");
        sim_state_topic_ = node->get_parameter("sim_state_topic").as_string();
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "sim_state_topic: %s", sim_state_topic_.c_str());

        node->declare_parameter<std::string>("sim_state_cmd_topic", "sim_state_cmd");
        sim_state_cmd_topic_ = node->get_parameter("sim_state_cmd_topic").as_string();
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "sim_state_cmd_topic: %s", sim_state_cmd_topic_.c_str());

        node->declare_parameter<std::string>("gnss_topic", "gps/fix");
        gnss_topic_ = node->get_parameter("gnss_topic").as_string();
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "gnss_topic: %s", gnss_topic_.c_str());

        node->declare_parameter<std::string>("gnss_frame_id", "gnss_link");
        gnss_frame_id_ = node->get_parameter("gnss_frame_id").as_string();
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "gnss_frame_id: %s", gnss_frame_id_.c_str());

        node->declare_parameter<double>("gnss_update_rate", 10.0);
        gnss_update_rate_ = node->get_parameter("gnss_update_rate").as_double();
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "gnss_update_rate: %f", gnss_update_rate_);

        node->declare_parameter("gnss_covariance_diagonal", rclcpp::PARAMETER_DOUBLE_ARRAY);
        gnss_covariance_diagonal_ = node->get_parameter("gnss_covariance_diagonal").as_double_array();
        if(gnss_covariance_diagonal_.size() != 3) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "gnss_covariance_diagonal must have 3 elements");
            gnss_covariance_diagonal_ = {0.0, 0.0, 0.0};
        } else {
            RCLCPP_INFO(
                rclcpp::get_logger("ASBWebotsDriver"),
                "gnss_covariance_diagonal: %f, %f, %f",
                gnss_covariance_diagonal_[0], gnss_covariance_diagonal_[1], gnss_covariance_diagonal_[2]);
        }

        right_motor_ = wb_robot_get_device("right_motor");
        left_motor_ = wb_robot_get_device("left_motor");
        gnss_ = wb_robot_get_device(gnss_frame_id_.c_str());
        gnss2_ = wb_robot_get_device("gnss_link_2");

        wb_gps_enable(gnss_, 1);
        wb_gps_enable(gnss2_, 1);

        if (wb_gps_get_coordinate_system(gnss_) != WB_GPS_WGS84_COORDINATE) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "The GPS world or sensor is not using 'WGS84' coordinates system");
        }

        wb_motor_set_position(left_motor_, INFINITY);
        wb_motor_set_velocity(left_motor_, 0.0);

        wb_motor_set_position(right_motor_, INFINITY);
        wb_motor_set_velocity(right_motor_, 0.0);

        sim_state_cmd_subscriber_ = node->create_subscription<asb_msgs::msg::SimStateCmd>(
            sim_state_cmd_topic_, rclcpp::SensorDataQoS().reliable(),
                std::bind(&ASBWebotsDriver::sim_state_cmd_callback, this, std::placeholders::_1));

        sim_state_publisher_ = node->create_publisher<asb_msgs::msg::SimState>(
            sim_state_topic_, rclcpp::SensorDataQoS().reliable());

        nav_sat_fix_publisher_ = node->create_publisher<sensor_msgs::msg::NavSatFix>(
            gnss_topic_, rclcpp::SensorDataQoS().reliable());

        nav_sat_fix_publisher2_ = node->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/gps2/fix", rclcpp::SensorDataQoS().reliable());

        last_gnss_fix_ = rclcpp::Clock().now();
    }

    void ASBWebotsDriver::sim_state_cmd_callback(const asb_msgs::msg::SimStateCmd::SharedPtr msg) {
      sim_state_cmd_msg_ = *msg;
    }

    void ASBWebotsDriver::step() {
        // wb_motor_set_velocity sets the linear track velocity (not the track wheel angular velocity),
        // so we need to divide by the wheel radius since sim_state_cmd_msg_.left_motor_speed_ref is an angular velocity.
        wb_motor_set_velocity(left_motor_, sim_state_cmd_msg_.left_motor_speed_ref * TRACK_WHEEL_RADIUS / GEARBOX_REDUCTION_RATIO);
        wb_motor_set_velocity(right_motor_, sim_state_cmd_msg_.right_motor_speed_ref * TRACK_WHEEL_RADIUS / GEARBOX_REDUCTION_RATIO);

        auto sim_state_msg = asb_msgs::msg::SimState();
        sim_state_msg.stamp = rclcpp::Clock().now();
        sim_state_msg.left_motor_speed = wb_motor_get_velocity(left_motor_) * GEARBOX_REDUCTION_RATIO / TRACK_WHEEL_RADIUS;
        sim_state_msg.right_motor_speed = wb_motor_get_velocity(right_motor_) * GEARBOX_REDUCTION_RATIO / TRACK_WHEEL_RADIUS;
        sim_state_msg.fan_motor_speed = sim_state_cmd_msg_.fan_motor_speed_ref;
        sim_state_publisher_->publish(sim_state_msg);

        auto now = rclcpp::Clock().now();

        if(now - last_gnss_fix_ > rclcpp::Duration::from_seconds(1/gnss_update_rate_)) {
            last_gnss_fix_ = now;

            const double *position = wb_gps_get_values(gnss_);
            const double latitude = position[0];
            const double longitude = position[1];
            const double altitude = position[2];
            auto nav_sat_fix_msg = sensor_msgs::msg::NavSatFix();
            nav_sat_fix_msg.header.stamp = now;
            nav_sat_fix_msg.header.frame_id = gnss_frame_id_;
            nav_sat_fix_msg.latitude = latitude;
            nav_sat_fix_msg.longitude = longitude;
            nav_sat_fix_msg.altitude = altitude;
            nav_sat_fix_msg.position_covariance = {
                gnss_covariance_diagonal_[0], 0.0, 0.0,
                0.0, gnss_covariance_diagonal_[1], 0.0,
                0.0, 0.0, gnss_covariance_diagonal_[2]
            };
            nav_sat_fix_msg.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            nav_sat_fix_msg.status.status = NavSatStatus::STATUS_GBAS_FIX;
            nav_sat_fix_msg.status.service =
                NavSatStatus::SERVICE_GPS |
                NavSatStatus::SERVICE_GLONASS |
                NavSatStatus::SERVICE_COMPASS |
                NavSatStatus::SERVICE_GALILEO;
            nav_sat_fix_publisher_->publish(nav_sat_fix_msg);

            const double *position2 = wb_gps_get_values(gnss2_);
            const double latitude2 = position2[0];
            const double longitude2 = position2[1];
            const double altitude2 = position2[2];
            auto nav_sat_fix_msg2 = sensor_msgs::msg::NavSatFix();
            nav_sat_fix_msg2.header.stamp = now;
            nav_sat_fix_msg2.header.frame_id = "gnss_link_2";
            nav_sat_fix_msg2.latitude = latitude2;
            nav_sat_fix_msg2.longitude = longitude2;
            nav_sat_fix_msg2.altitude = altitude2;
            nav_sat_fix_msg2.position_covariance = {
                0.0001, 0.0,    0.0,
                0.0,    0.0001, 0.0,
                0.0,    0.0,    0.0001
            };
            nav_sat_fix_msg2.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            nav_sat_fix_msg.status.status = NavSatStatus::STATUS_GBAS_FIX;
            nav_sat_fix_msg.status.service =
                NavSatStatus::SERVICE_GPS |
                NavSatStatus::SERVICE_GLONASS |
                NavSatStatus::SERVICE_COMPASS |
                NavSatStatus::SERVICE_GALILEO;
            nav_sat_fix_publisher2_->publish(nav_sat_fix_msg2);
        }

    }
} // namespace asb_webots_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(asb_webots_driver::ASBWebotsDriver, webots_ros2_driver::PluginInterface)
