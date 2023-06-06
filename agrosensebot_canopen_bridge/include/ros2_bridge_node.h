#ifndef TEST_ROS2_CANOPEN_BRIDGE_ROS2_BRIDGE_NODE_H
#define TEST_ROS2_CANOPEN_BRIDGE_ROS2_BRIDGE_NODE_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/u_int8.hpp"
#include "agrosensebot_canopen_bridge_msgs/msg/motor_drive.hpp"
#include "agrosensebot_canopen_bridge_msgs/msg/speed_ref.hpp"


using std::placeholders::_1;

class CANOpenSlaveNode;

class ROS2BridgeNode : public rclcpp_lifecycle::LifecycleNode {
    std::string canopen_node_config_;
    std::string can_interface_name_;
    std::atomic<bool> active = false;
    std::future<void> slave_done;
    std::mutex a;
    std::thread t;
    rclcpp::Subscription<std_msgs::msg::UInt8>::SharedPtr gcu_alive_sub;
    rclcpp::Subscription<agrosensebot_canopen_bridge_msgs::msg::SpeedRef>::SharedPtr speed_ref_sub;
    rclcpp_lifecycle::LifecyclePublisher<agrosensebot_canopen_bridge_msgs::msg::MotorDrive>::SharedPtr motor_drive_pub;
    std::shared_ptr<CANOpenSlaveNode> canopen_slave_node = nullptr;

    void gcu_alive_ros2_callback(std_msgs::msg::UInt8::SharedPtr) const ;
    void speed_ref_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::SpeedRef::SharedPtr) const ;
    void run_canopen_slave_node()	;

public:
    explicit ROS2BridgeNode(const std::string &node_name, bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)){
      this->declare_parameter<std::string>("canopen_node_config", "test_slave.eds");
      this->declare_parameter<std::string>("can_interface_name", "vcan0");

      gcu_alive_sub = this->create_subscription<std_msgs::msg::UInt8>(
              "test", 10,
              std::bind(&ROS2BridgeNode::gcu_alive_ros2_callback, this, _1));

      speed_ref_sub = this->create_subscription<agrosensebot_canopen_bridge_msgs::msg::SpeedRef>(
              "speed_ref", 10,
              std::bind(&ROS2BridgeNode::speed_ref_ros2_callback, this, _1));

      motor_drive_pub = this->create_publisher<agrosensebot_canopen_bridge_msgs::msg::MotorDrive>("motor_drive", rclcpp::SensorDataQoS());
    };

    void motor_drive_canopen_callback(int16_t FAN_controller_temperature, int16_t FAN_motor_temperature, int16_t FAN_motor_RPM, int16_t FAN_battery_current_display);

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_configure(const rclcpp_lifecycle::State &) override ;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_activate(const rclcpp_lifecycle::State &) override ;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_deactivate(const rclcpp_lifecycle::State &) override ;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_cleanup(const rclcpp_lifecycle::State &) override ;

    rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
    on_shutdown(const rclcpp_lifecycle::State &) override ;

};

#endif //TEST_ROS2_CANOPEN_BRIDGE_ROS2_BRIDGE_NODE_H
