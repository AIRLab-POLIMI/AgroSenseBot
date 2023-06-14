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
    rclcpp::Subscription<agrosensebot_canopen_bridge_msgs::msg::MotorDrive>::SharedPtr motor_drive_sub;
    std::shared_ptr<CANOpenSlaveNode> canopen_slave_node = nullptr;

    void motor_drive_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::MotorDrive::SharedPtr) const ;
//    void gcu_alive_ros2_callback(std_msgs::msg::UInt8::SharedPtr) const ;
//    void speed_ref_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::SpeedRef::SharedPtr) const ;
    void run_canopen_slave_node()	;

public:
    explicit ROS2BridgeNode(const std::string &node_name, bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)){
      this->declare_parameter<std::string>("canopen_node_config", "test_slave.eds");
      this->declare_parameter<std::string>("can_interface_name", "vcan0");

      motor_drive_sub = this->create_subscription<agrosensebot_canopen_bridge_msgs::msg::MotorDrive>(
              "fake_motor_drive", 10,
              std::bind(&ROS2BridgeNode::motor_drive_ros2_callback, this, _1));

    };

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
