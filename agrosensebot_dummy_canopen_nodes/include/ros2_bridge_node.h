#ifndef TEST_ROS2_CANOPEN_BRIDGE_ROS2_BRIDGE_NODE_H
#define TEST_ROS2_CANOPEN_BRIDGE_ROS2_BRIDGE_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "agrosensebot_canopen_bridge_msgs/msg/motor_drive.hpp"
#include "agrosensebot_canopen_bridge_msgs/msg/speed_ref.hpp"
#include "agrosensebot_canopen_bridge_msgs/msg/vcu_state.hpp"

#include <chrono>

#define INVERSE_RAW_DATA_STEP_VALUE_temperature 10 // 10°C
#define INVERSE_RAW_DATA_STEP_VALUE_current 10 // 10A

using std::placeholders::_1;
using namespace std::chrono_literals;

class VCUCANOpenSlaveNode;
class MotorDriveCANOpenSlaveNode;

class ROS2BridgeNode : public rclcpp_lifecycle::LifecycleNode {
    std::string VCU_canopen_node_config_;
    std::string MDL_canopen_node_config_;
    std::string MDR_canopen_node_config_;
    std::string FAN_canopen_node_config_;
    std::string can_interface_name_;
    std::chrono::milliseconds gcu_is_alive_timeout_ = 100ms;

    std::thread VCU_canopen_node_thread_;
    std::thread MDL_canopen_node_thread_;
    std::thread MDR_canopen_node_thread_;
    std::thread FAN_canopen_node_thread_;
    std::shared_ptr <VCUCANOpenSlaveNode> VCU_canopen_slave_node_ = nullptr;
    std::shared_ptr <MotorDriveCANOpenSlaveNode> MDL_canopen_slave_node_ = nullptr;
    std::shared_ptr <MotorDriveCANOpenSlaveNode> MDR_canopen_slave_node_ = nullptr;
    std::shared_ptr <MotorDriveCANOpenSlaveNode> FAN_canopen_slave_node_ = nullptr;

    std::atomic<bool> lifecycle_node_active_ = false;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt8>::SharedPtr gcu_alive_pub_;
    rclcpp_lifecycle::LifecyclePublisher<agrosensebot_canopen_bridge_msgs::msg::SpeedRef>::SharedPtr speed_ref_pub_;
    rclcpp::Subscription<agrosensebot_canopen_bridge_msgs::msg::MotorDrive>::SharedPtr motor_drive_left_sub_;
    rclcpp::Subscription<agrosensebot_canopen_bridge_msgs::msg::MotorDrive>::SharedPtr motor_drive_right_sub_;
    rclcpp::Subscription<agrosensebot_canopen_bridge_msgs::msg::MotorDrive>::SharedPtr motor_drive_fan_sub_;
    rclcpp::Subscription<agrosensebot_canopen_bridge_msgs::msg::VCUState>::SharedPtr VCU_state_sub_;
    rclcpp::TimerBase::SharedPtr gcu_is_alive_timer_;

    bool last_VCU_alive_bit_ = false;

    rclcpp::Time last_GCU_message_time_ = rclcpp::Time(0);
    rclcpp::Time last_GCU_alive_bit_change_time_ = rclcpp::Time(0);
    bool last_GCU_alive_bit_ = false;

    void vcu_alive_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::VCUState::SharedPtr) ;

    void gcu_is_alive_timer_ros2_callback();

    void run_VCU_canopen_node();
    void run_MDL_canopen_node();
    void run_MDR_canopen_node();
    void run_FAN_canopen_node();

public:
    explicit ROS2BridgeNode(const std::string &node_name, bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(
                    intra_process_comms)) {
        this->declare_parameter<std::string>("dummy_VCU_canopen_node_config", "test_slave.eds");
        this->declare_parameter<std::string>("dummy_MDL_canopen_node_config", "test_slave.eds");
        this->declare_parameter<std::string>("dummy_MDR_canopen_node_config", "test_slave.eds");
        this->declare_parameter<std::string>("dummy_FAN_canopen_node_config", "test_slave.eds");
        this->declare_parameter<std::string>("can_interface_name", "vcan0");

        gcu_alive_pub_ = this->create_publisher<std_msgs::msg::UInt8>("test/gcu_alive", rclcpp::SensorDataQoS());
        speed_ref_pub_ = this->create_publisher<agrosensebot_canopen_bridge_msgs::msg::SpeedRef>("test/speed_ref", rclcpp::SensorDataQoS());

        motor_drive_left_sub_ = this->create_subscription<agrosensebot_canopen_bridge_msgs::msg::MotorDrive>("test/motor_drive_left", rclcpp::SensorDataQoS(), std::bind(&ROS2BridgeNode::motor_drive_left_ros2_callback, this, _1));
        motor_drive_right_sub_ = this->create_subscription<agrosensebot_canopen_bridge_msgs::msg::MotorDrive>("test/motor_drive_right", rclcpp::SensorDataQoS(), std::bind(&ROS2BridgeNode::motor_drive_right_ros2_callback, this, _1));
        motor_drive_fan_sub_ = this->create_subscription<agrosensebot_canopen_bridge_msgs::msg::MotorDrive>("test/motor_drive_fan", rclcpp::SensorDataQoS(), std::bind(&ROS2BridgeNode::motor_drive_fan_ros2_callback, this, _1));
        VCU_state_sub_ = this->create_subscription<agrosensebot_canopen_bridge_msgs::msg::VCUState>("test/vcu_state", rclcpp::SensorDataQoS(), std::bind(&ROS2BridgeNode::vcu_alive_ros2_callback, this, _1));

    };

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &) override;

    void gcu_alive_canopen_callback(bool, bool, bool);

    void speed_ref_canopen_callback(int16_t, int16_t, int16_t);

    void motor_drive_left_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::MotorDrive::SharedPtr);

    void motor_drive_right_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::MotorDrive::SharedPtr);

    void motor_drive_fan_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::MotorDrive::SharedPtr);

};

#endif //TEST_ROS2_CANOPEN_BRIDGE_ROS2_BRIDGE_NODE_H
