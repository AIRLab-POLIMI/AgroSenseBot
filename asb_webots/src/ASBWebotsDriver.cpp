#include "asb_webots/ASBWebotsDriver.hpp"

#include "rclcpp/rclcpp.hpp"
#include <functional>
#include <webots/gps.h>
#include <webots/inertial_unit.h>
#include <webots/gyro.h>
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

        try{
            node->declare_parameter<std::string>("sim_state_topic", "sim_state");
            sim_state_topic_ = node->get_parameter("sim_state_topic").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "sim_state_topic: %s", sim_state_topic_.c_str());

        try{
            node->declare_parameter<std::string>("sim_state_cmd_topic", "sim_state_cmd");
            sim_state_cmd_topic_ = node->get_parameter("sim_state_cmd_topic").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "sim_state_cmd_topic: %s", sim_state_cmd_topic_.c_str());

        try{
            node->declare_parameter<std::string>("gnss_1_topic", "gps_1/fix");
            gnss_1_topic_ = node->get_parameter("gnss_1_topic").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "gnss_1_topic: %s", gnss_1_topic_.c_str());

        try{
            node->declare_parameter<std::string>("gnss_2_topic", "gps_2/fix");
            gnss_2_topic_ = node->get_parameter("gnss_2_topic").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "gnss_2_topic: %s", gnss_2_topic_.c_str());

        try{
            node->declare_parameter<std::string>("gnss_1_frame_id", "gnss_1_link");
            gnss_1_frame_id_ = node->get_parameter("gnss_1_frame_id").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "gnss_1_frame_id: %s", gnss_1_frame_id_.c_str());

        try{
            node->declare_parameter<std::string>("gnss_2_frame_id", "gnss_2_link");
            gnss_2_frame_id_ = node->get_parameter("gnss_2_frame_id").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "gnss_2_frame_id: %s", gnss_2_frame_id_.c_str());

        try{
            node->declare_parameter<double>("gnss_update_rate", 10.0);
            gnss_update_rate_ = node->get_parameter("gnss_update_rate").as_double();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "gnss_update_rate: %f", gnss_update_rate_);

        try{
            node->declare_parameter("gnss_covariance_diagonal", rclcpp::PARAMETER_DOUBLE_ARRAY);
            gnss_covariance_diagonal_ = node->get_parameter("gnss_covariance_diagonal").as_double_array();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        if(gnss_covariance_diagonal_.size() != 3) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "gnss_covariance_diagonal must have 3 elements. Using default values.");
            gnss_covariance_diagonal_ = {1.0, 1.0, 1.0};
        }
        RCLCPP_INFO(
            rclcpp::get_logger("ASBWebotsDriver"),
            "gnss_covariance_diagonal: %f, %f, %f",
            gnss_covariance_diagonal_[0], gnss_covariance_diagonal_[1], gnss_covariance_diagonal_[2]);

        try{
            node->declare_parameter<std::string>("inertial_unit_topic", "inertial_unit");
            inertial_unit_topic_ = node->get_parameter("inertial_unit_topic").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "inertial_unit_topic: %s", inertial_unit_topic_.c_str());

        try{
            node->declare_parameter<std::string>("inertial_unit_parent_frame_id", "map");
            inertial_unit_parent_frame_id_ = node->get_parameter("inertial_unit_parent_frame_id").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "inertial_unit_parent_frame_id: %s", inertial_unit_parent_frame_id_.c_str());

        try{
            node->declare_parameter<std::string>("inertial_unit_child_frame_id", "inertial_frame");
            inertial_unit_child_frame_id_ = node->get_parameter("inertial_unit_child_frame_id").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "inertial_unit_child_frame_id: %s", inertial_unit_child_frame_id_.c_str());

        try{
            node->declare_parameter<double>("inertial_unit_update_rate", 10.0);
            inertial_unit_update_rate_ = node->get_parameter("inertial_unit_update_rate").as_double();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "inertial_unit_update_rate: %f", inertial_unit_update_rate_);

        try{
            node->declare_parameter("inertial_unit_covariance_diagonal", rclcpp::PARAMETER_DOUBLE_ARRAY);
            inertial_unit_covariance_diagonal_ = node->get_parameter("inertial_unit_covariance_diagonal").as_double_array();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        if(inertial_unit_covariance_diagonal_.size() != 3) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "inertial_unit_covariance_diagonal must have 3 elements. Using default values.");
            inertial_unit_covariance_diagonal_ = {1.0, 1.0, 1.0};
        }
        RCLCPP_INFO(
            rclcpp::get_logger("ASBWebotsDriver"),
            "inertial_unit_covariance_diagonal: %f, %f, %f",
            inertial_unit_covariance_diagonal_[0], inertial_unit_covariance_diagonal_[1], inertial_unit_covariance_diagonal_[2]);

        try{
            node->declare_parameter<double>("imu_update_rate", 1000.0);
            imu_update_rate_ = node->get_parameter("imu_update_rate").as_double();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "imu_update_rate: %f", imu_update_rate_);

        try{
            node->declare_parameter<std::string>("imu_topic", "imu");
            imu_topic_ = node->get_parameter("imu_topic").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "imu_topic: %s", imu_topic_.c_str());

        try{
            node->declare_parameter<std::string>("imu_frame_id", "imu_link");
            imu_frame_id_ = node->get_parameter("imu_frame_id").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "imu_frame_id: %s", imu_frame_id_.c_str());

        try{
            node->declare_parameter<std::string>("gyro_frame_id", "gyro_link");
            gyro_frame_id_ = node->get_parameter("gyro_frame_id").as_string();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        RCLCPP_INFO(rclcpp::get_logger("ASBWebotsDriver"), "gyro_frame_id: %s", gyro_frame_id_.c_str());

        try{
            node->declare_parameter("gyro_covariance_diagonal", rclcpp::PARAMETER_DOUBLE_ARRAY);
            gyro_covariance_diagonal_ = node->get_parameter("gyro_covariance_diagonal").as_double_array();
        } catch (rclcpp::exceptions::InvalidParameterTypeException &e) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "%s", e.what());
        }
        if(gyro_covariance_diagonal_.size() != 3) {
            RCLCPP_ERROR(rclcpp::get_logger("ASBWebotsDriver"), "gyro_covariance_diagonal must have 3 elements. Using default values.");
            gyro_covariance_diagonal_ = {1.0, 1.0, 1.0};
        }
        RCLCPP_INFO(
            rclcpp::get_logger("ASBWebotsDriver"),
            "gyro_covariance_diagonal: %f, %f, %f",
            gyro_covariance_diagonal_[0], gyro_covariance_diagonal_[1], gyro_covariance_diagonal_[2]);

        gnss_1_ = wb_robot_get_device(gnss_1_frame_id_.c_str());
        gnss_2_ = wb_robot_get_device(gnss_2_frame_id_.c_str());
        inertial_unit_ = wb_robot_get_device(inertial_unit_child_frame_id_.c_str());
        gyro_ = wb_robot_get_device(gyro_frame_id_.c_str());
        right_motor_ = wb_robot_get_device("right_motor");
        left_motor_ = wb_robot_get_device("left_motor");

        wb_gps_enable(gnss_1_, 1);
        wb_gps_enable(gnss_2_, 1);
        wb_inertial_unit_enable(inertial_unit_, 1);
        wb_gyro_enable(gyro_, 1);

        if ((wb_gps_get_coordinate_system(gnss_1_) != WB_GPS_WGS84_COORDINATE)
        || (wb_gps_get_coordinate_system(gnss_2_) != WB_GPS_WGS84_COORDINATE)) {
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

        gnss_1_publisher_ = node->create_publisher<sensor_msgs::msg::NavSatFix>(
            gnss_1_topic_, rclcpp::SensorDataQoS().reliable());

        gnss_2_publisher_ = node->create_publisher<sensor_msgs::msg::NavSatFix>(
            gnss_2_topic_, rclcpp::SensorDataQoS().reliable());

        inertial_unit_publisher_ = node->create_publisher<nav_msgs::msg::Odometry>(
            inertial_unit_topic_, rclcpp::SensorDataQoS().reliable());

        imu_publisher_ = node->create_publisher<sensor_msgs::msg::Imu>(
            imu_topic_, rclcpp::SensorDataQoS().reliable());

        last_gnss_fix_ = rclcpp::Clock().now();
        last_inertial_unit_update_ = rclcpp::Clock().now();
        last_imu_update_ = rclcpp::Clock().now();
    }

    void ASBWebotsDriver::sim_state_cmd_callback(const asb_msgs::msg::SimStateCmd::SharedPtr msg) {
      sim_state_cmd_msg_ = *msg;
    }

    void ASBWebotsDriver::step() {
        // wb_motor_set_velocity sets the linear track velocity (not the track wheel angular velocity),
        // so we need to divide by the wheel radius since sim_state_cmd_msg_.left_motor_speed_ref is an angular velocity.
        wb_motor_set_velocity(left_motor_, sim_state_cmd_msg_.left_motor_speed_ref * TRACK_WHEEL_RADIUS / GEARBOX_REDUCTION_RATIO);
        wb_motor_set_velocity(right_motor_, sim_state_cmd_msg_.right_motor_speed_ref * TRACK_WHEEL_RADIUS / GEARBOX_REDUCTION_RATIO);

        double fan_motor_speed_ref = sim_state_cmd_msg_.fan_motor_speed_ref;
        fan_motor_ref_ring_[fan_motor_ring_index_ % fan_motor_ring_size_] = fan_motor_speed_ref;
        fan_motor_ring_index_++;

        auto sim_state_msg = asb_msgs::msg::SimState();
        sim_state_msg.stamp = rclcpp::Clock().now();
        sim_state_msg.left_motor_speed = wb_motor_get_velocity(left_motor_) * GEARBOX_REDUCTION_RATIO / TRACK_WHEEL_RADIUS;
        sim_state_msg.right_motor_speed = wb_motor_get_velocity(right_motor_) * GEARBOX_REDUCTION_RATIO / TRACK_WHEEL_RADIUS;
        sim_state_msg.fan_motor_speed = std::accumulate(fan_motor_ref_ring_.begin(), fan_motor_ref_ring_.end(), 0.0)/fan_motor_ring_size_;
        sim_state_publisher_->publish(sim_state_msg);

        auto now = rclcpp::Clock().now();

        if(now - last_gnss_fix_ > rclcpp::Duration::from_seconds(1/gnss_update_rate_)) {
            last_gnss_fix_ = now;

            const double *position_1 = wb_gps_get_values(gnss_1_);
            const double latitude_1 = position_1[0];
            const double longitude_1 = position_1[1];
            const double altitude_1 = position_1[2];
            auto gnss_1_msg = sensor_msgs::msg::NavSatFix();
            gnss_1_msg.header.stamp = now;
            gnss_1_msg.header.frame_id = gnss_1_frame_id_;
            gnss_1_msg.latitude = latitude_1;
            gnss_1_msg.longitude = longitude_1;
            gnss_1_msg.altitude = altitude_1;
            gnss_1_msg.position_covariance = {
                gnss_covariance_diagonal_[0], 0.0, 0.0,
                0.0, gnss_covariance_diagonal_[1], 0.0,
                0.0, 0.0, gnss_covariance_diagonal_[2]
            };
            gnss_1_msg.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            gnss_1_msg.status.status = NavSatStatus::STATUS_GBAS_FIX;
            gnss_1_msg.status.service =
                NavSatStatus::SERVICE_GPS |
                NavSatStatus::SERVICE_GLONASS |
                NavSatStatus::SERVICE_COMPASS |
                NavSatStatus::SERVICE_GALILEO;

            const double *position_2 = wb_gps_get_values(gnss_2_);
            const double latitude_2 = position_2[0];
            const double longitude_2 = position_2[1];
            const double altitude_2 = position_2[2];
            auto gnss_2_msg = sensor_msgs::msg::NavSatFix();
            gnss_2_msg.header.stamp = now;
            gnss_2_msg.header.frame_id = gnss_2_frame_id_;
            gnss_2_msg.latitude = latitude_2;
            gnss_2_msg.longitude = longitude_2;
            gnss_2_msg.altitude = altitude_2;
            gnss_2_msg.position_covariance = {
                gnss_covariance_diagonal_[0], 0.0, 0.0,
                0.0, gnss_covariance_diagonal_[1], 0.0,
                0.0, 0.0, gnss_covariance_diagonal_[2]
            };
            gnss_2_msg.position_covariance_type = NavSatFix::COVARIANCE_TYPE_DIAGONAL_KNOWN;
            gnss_2_msg.status.status = NavSatStatus::STATUS_GBAS_FIX;
            gnss_2_msg.status.service =
                NavSatStatus::SERVICE_GPS |
                NavSatStatus::SERVICE_GLONASS |
                NavSatStatus::SERVICE_COMPASS |
                NavSatStatus::SERVICE_GALILEO;

            gnss_1_publisher_->publish(gnss_1_msg);
            gnss_2_publisher_->publish(gnss_2_msg);
        }

        if(now - last_inertial_unit_update_ > rclcpp::Duration::from_seconds(1/inertial_unit_update_rate_)) {
            last_inertial_unit_update_ = now;

            const double *q = wb_inertial_unit_get_quaternion(inertial_unit_);
            const double x = q[0];
            const double y = q[1];
            const double z = q[2];
            const double w = q[3];

            auto inertial_unit_msg = nav_msgs::msg::Odometry();
            inertial_unit_msg.header.stamp = now;
            inertial_unit_msg.header.frame_id = inertial_unit_parent_frame_id_;
            inertial_unit_msg.child_frame_id = inertial_unit_child_frame_id_;
            inertial_unit_msg.pose.pose.orientation.x = x;
            inertial_unit_msg.pose.pose.orientation.y = y;
            inertial_unit_msg.pose.pose.orientation.z = z;
            inertial_unit_msg.pose.pose.orientation.w = w;
            inertial_unit_msg.pose.covariance[21] = inertial_unit_covariance_diagonal_[0];
            inertial_unit_msg.pose.covariance[28] = inertial_unit_covariance_diagonal_[1];
            inertial_unit_msg.pose.covariance[35] = inertial_unit_covariance_diagonal_[2];
            inertial_unit_publisher_->publish(inertial_unit_msg);
        }

        if(now - last_imu_update_ > rclcpp::Duration::from_seconds(1/imu_update_rate_)) {
            last_imu_update_ = now;

            const double *vel = wb_gyro_get_values(gyro_);
            double gyro_x = vel[0];
            double gyro_y = vel[1];
            double gyro_z = vel[2];
            auto imu_msg = sensor_msgs::msg::Imu();
            imu_msg.header.stamp = now;
            imu_msg.header.frame_id = imu_frame_id_;
            imu_msg.angular_velocity.x = gyro_x;
            imu_msg.angular_velocity.y = gyro_y;
            imu_msg.angular_velocity.z = gyro_z;
            imu_msg.angular_velocity_covariance = {
                gyro_covariance_diagonal_[0], 0.0, 0.0,
                0.0, gyro_covariance_diagonal_[1], 0.0,
                0.0, 0.0, gyro_covariance_diagonal_[2]
            };
            imu_publisher_->publish(imu_msg);
        }

    }
} // namespace asb_webots_driver

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(asb_webots_driver::ASBWebotsDriver, webots_ros2_driver::PluginInterface)
