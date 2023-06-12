#include "ros2_bridge_node.h"
#include "canopen_slave_node.h"

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_configure(const rclcpp_lifecycle::State &) {
    active.store(false);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_activate(const rclcpp_lifecycle::State &) {
    get_parameter("canopen_node_config", canopen_node_config_);
    get_parameter("can_interface_name", can_interface_name_);

    active.store(true);
    motor_drive_pub->on_activate();

    canopen_node_thread = std::thread(std::bind(&ROS2BridgeNode::run_canopen_slave_node, this));

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_deactivate(const rclcpp_lifecycle::State &) {
    active.store(false);
    canopen_node_thread.join();
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_cleanup(const rclcpp_lifecycle::State &) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_shutdown(const rclcpp_lifecycle::State &) {
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ROS2BridgeNode::run_canopen_slave_node() {
    io::IoGuard io_guard;
    io::Context ctx;
    io::Poll poll(ctx);
    ev::Loop loop(poll.get_poll());
    auto exec = loop.get_executor();
    io::Timer timer(poll, exec, CLOCK_MONOTONIC);
    io::CanController ctrl(can_interface_name_.c_str());
    io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    canopen_slave_node = std::make_shared<CANOpenSlaveNode>(timer, chan, canopen_node_config_, "", this);
    canopen_slave_node->Reset();

    while (active.load()) {
        loop.run_one_for(10ms);
    }
    ctx.shutdown();
}

void ROS2BridgeNode::motor_drive_canopen_callback(int16_t FAN_controller_temperature, int16_t FAN_motor_temperature,
                                                  int16_t FAN_motor_RPM, int16_t FAN_battery_current_display) {
    agrosensebot_canopen_bridge_msgs::msg::MotorDrive msg;
    msg.fan_controller_temperature = FAN_controller_temperature;
    msg.fan_motor_temperature = FAN_motor_temperature;
    msg.fan_motor_rpm = FAN_motor_RPM;
    msg.fan_battery_current_display = FAN_battery_current_display;
    motor_drive_pub->publish(msg);
}

void ROS2BridgeNode::gcu_alive_ros2_callback(std_msgs::msg::UInt8::SharedPtr msg) const {
    if (canopen_slave_node != nullptr) {
        canopen_slave_node->send_TPDO_1(msg->data);
    }
}

void ROS2BridgeNode::speed_ref_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::SpeedRef::SharedPtr msg) const {
    if (canopen_slave_node != nullptr) {
        canopen_slave_node->send_TPDO_2(msg->right_speed_ref, msg->left_speed_ref);
    }
}
