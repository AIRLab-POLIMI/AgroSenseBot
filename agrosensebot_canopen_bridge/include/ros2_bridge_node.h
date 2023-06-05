#ifndef TEST_ROS2_CANOPEN_BRIDGE_ROS2_BRIDGE_NODE_H
#define TEST_ROS2_CANOPEN_BRIDGE_ROS2_BRIDGE_NODE_H

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/slave.hpp>

#include <future>
#include <atomic>
#include <mutex>
#include <thread>

#include "rclcpp/rclcpp.hpp"
#include "rclcpp/qos.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/u_int32.hpp"

using std::placeholders::_1;

using namespace lely;
using namespace std::chrono_literals;

class CANOpenSlaveNode;

class ROS2BridgeNode : public rclcpp_lifecycle::LifecycleNode {
    uint8_t canopen_node_id_ = 1;
    std::string canopen_node_config_;
    std::string can_interface_name_;
    std::atomic<bool> active = false;
    std::future<void> slave_done;
    std::mutex a;
    std::thread t;
    rclcpp::Subscription<std_msgs::msg::UInt32>::SharedPtr int_sub;
    rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::UInt32>::SharedPtr int_pub;
    std::shared_ptr<CANOpenSlaveNode> canopen_slave_node = nullptr;

    void topic_callback(std_msgs::msg::UInt32::SharedPtr) const ;
    void run_canopen_slave_node()	;

public:
    explicit ROS2BridgeNode(const std::string &node_name, bool intra_process_comms = false)
            : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms)){
      this->declare_parameter<uint8_t>("canopen_node_id", 1);
      this->declare_parameter<std::string>("canopen_node_config", "test_slave.eds");
      this->declare_parameter<std::string>("can_interface_name", "vcan0");

      int_sub = this->create_subscription<std_msgs::msg::UInt32>(
              "test", 10,
              std::bind(&ROS2BridgeNode::topic_callback, this, _1));

      int_pub = this->create_publisher<std_msgs::msg::UInt32>("test_pub", rclcpp::SensorDataQoS());
    };

    void RPDO_4_callback(uint16_t FAN_controller_temperature, uint16_t FAN_motor_temperature, uint16_t FAN_motor_RPM, uint16_t FAN_battery_current_display);

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
