#include "asb_webots/ASBWebotsDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <webots/gps.h>
#include <webots/motor.h>
#include <webots/robot.h>

#define GEARBOX_REDUCTION_RATIO 40.61
#define TRACK_WHEEL_RADIUS 0.151

namespace asb_webots_driver {
    void ASBWebotsDriver::init(webots_ros2_driver::WebotsNode *node, std::unordered_map<std::string, std::string> &parameters) {

//        wb_robot_init();

        right_motor_ = wb_robot_get_device("right_motor");
        left_motor_ = wb_robot_get_device("left_motor");
        gnss_ = wb_robot_get_device("gnss_link");

        wb_gps_enable(gnss_, 1000);

        if (wb_gps_get_coordinate_system(gnss_) != WB_GPS_WGS84_COORDINATE) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "The GPS world or sensor is not using 'WGS84' coordinates system");
        }

        wb_motor_set_position(left_motor_, INFINITY);
        wb_motor_set_velocity(left_motor_, 0.0);

        wb_motor_set_position(right_motor_, INFINITY);
        wb_motor_set_velocity(right_motor_, 0.0);

        sim_state_cmd_subscriber_ = node->create_subscription<asb_msgs::msg::SimStateCmd>(
                "/system_test/state_cmd", rclcpp::SensorDataQoS().reliable(),
                std::bind(&ASBWebotsDriver::sim_state_cmd_callback, this, std::placeholders::_1));

        sim_state_publisher_ = node->create_publisher<asb_msgs::msg::SimState>(
            "/system_test/state", rclcpp::SensorDataQoS().reliable());

        nav_sat_fix_publisher_ = node->create_publisher<sensor_msgs::msg::NavSatFix>(
            "/gps/fix", rclcpp::SensorDataQoS().reliable());

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

        /* get coordinates and speed */
        const double *position = wb_gps_get_values(gnss_);
        const double latitude = position[0];
        const double longitude = position[1];
        const double altitude = position[2];
//        const double speed = wb_gps_get_speed(gnss_);
//
//        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "Latitude is: %lf deg", latitude);
//        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "Longitude is: %lf deg", longitude);
//        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "Altitude is: %lf deg", altitude);
//        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "Speed is: %lf deg", speed);

        auto nav_sat_fix_msg = sensor_msgs::msg::NavSatFix();
        nav_sat_fix_msg.header.stamp = rclcpp::Clock().now();
        nav_sat_fix_msg.header.frame_id = "gnss_link";
        nav_sat_fix_msg.latitude = latitude;
        nav_sat_fix_msg.longitude = longitude;
        nav_sat_fix_msg.altitude = altitude;
        nav_sat_fix_msg.position_covariance = {
            0.0001, 0.0,    0.0,
            0.0,    0.0001, 0.0,
            0.0,    0.0,    0.0001
        };
        nav_sat_fix_msg.position_covariance_type = 2;
        nav_sat_fix_publisher_->publish(nav_sat_fix_msg);

    }
} // namespace asb_webots_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(asb_webots_driver::ASBWebotsDriver, webots_ros2_driver::PluginInterface)
