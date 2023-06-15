#include "ros2_bridge_node.h"
#include "canopen_slave_node.h"

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_configure(const rclcpp_lifecycle::State &) {
    lifecycle_node_active_.store(false);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_activate(const rclcpp_lifecycle::State &) {
    get_parameter("canopen_node_config", canopen_node_config_);
    get_parameter("can_interface_name", can_interface_name_);

    lifecycle_node_active_.store(true);
    motor_drive_left_pub_->on_activate();
    motor_drive_right_pub_->on_activate();
    motor_drive_fan_pub_->on_activate();
    VCU_state_pub_->on_activate();

    canopen_node_thread_ = std::thread(std::bind(&ROS2BridgeNode::run_canopen_slave_node, this));

    std::chrono::duration vcu_is_alive_timer_period_ = 10ms;
    vcu_is_alive_timer_ = rclcpp::create_timer(
            this, this->get_clock(), rclcpp::Duration(vcu_is_alive_timer_period_),
            std::bind(&ROS2BridgeNode::vcu_is_alive_timer_ros2_callback, this));
    last_VCU_message_time_ = this->get_clock()->now();
    last_VCU_alive_bit_change_time_ = this->get_clock()->now();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_deactivate(const rclcpp_lifecycle::State &) {
    lifecycle_node_active_.store(false);
    canopen_node_thread_.join();
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

    canopen_slave_node_ = std::make_shared<CANOpenSlaveNode>(timer, chan,
                                                             canopen_node_config_, "", this);
    canopen_slave_node_->Reset();

    while (lifecycle_node_active_.load()) {
        loop.run_one_for(10ms);
    }
    ctx.shutdown();
}

void ROS2BridgeNode::vcu_alive_canopen_callback(bool VCU_alive_bit, bool VCU_safety_status_bit, uint8_t control_mode) {
    rclcpp::Time now = this->get_clock()->now();
    last_VCU_message_time_ = now;
    if (last_VCU_alive_bit_ != VCU_alive_bit) last_VCU_alive_bit_change_time_ = now;
    last_VCU_alive_bit_ = VCU_alive_bit;

    agrosensebot_canopen_bridge_msgs::msg::VCUState VCU_state_msg;
    VCU_state_msg.stamp = now;
    VCU_state_msg.vcu_safety_status = VCU_safety_status_bit;
    VCU_state_msg.control_mode = control_mode;
    VCU_state_pub_->publish(VCU_state_msg);
}

void ROS2BridgeNode::motor_drive_left_canopen_callback(int16_t controller_temperature, int16_t motor_temperature,
                                                       int16_t motor_RPM, int16_t battery_current_display) {
    agrosensebot_canopen_bridge_msgs::msg::MotorDrive msg;
    msg.stamp = this->get_clock()->now();
    msg.controller_temperature = controller_temperature;
    msg.motor_temperature = motor_temperature;
    msg.motor_rpm = motor_RPM;
    msg.battery_current_display = battery_current_display;
    motor_drive_left_pub_->publish(msg);
}

void ROS2BridgeNode::motor_drive_right_canopen_callback(int16_t controller_temperature, int16_t motor_temperature,
                                                        int16_t motor_RPM, int16_t battery_current_display) {
    agrosensebot_canopen_bridge_msgs::msg::MotorDrive msg;
    msg.stamp = this->get_clock()->now();
    msg.controller_temperature = controller_temperature;
    msg.motor_temperature = motor_temperature;
    msg.motor_rpm = motor_RPM;
    msg.battery_current_display = battery_current_display;
    motor_drive_right_pub_->publish(msg);
}

void ROS2BridgeNode::motor_drive_fan_canopen_callback(int16_t controller_temperature, int16_t motor_temperature,
                                                      int16_t motor_RPM, int16_t battery_current_display) {
    agrosensebot_canopen_bridge_msgs::msg::MotorDrive msg;
    msg.stamp = this->get_clock()->now();
    msg.controller_temperature = controller_temperature;
    msg.motor_temperature = motor_temperature;
    msg.motor_rpm = motor_RPM;
    msg.battery_current_display = battery_current_display;
    motor_drive_fan_pub_->publish(msg);
}

void ROS2BridgeNode::gcu_alive_ros2_callback(std_msgs::msg::UInt8::SharedPtr msg) const {
    if (canopen_slave_node_ != nullptr) {
        canopen_slave_node_->send_TPDO_1(msg->data);
    }
}

void ROS2BridgeNode::speed_ref_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::SpeedRef::SharedPtr msg) {
    rclcpp::Time now = this->get_clock()->now();
    if (now - msg->stamp > rclcpp::Duration(speed_ref_max_age_)){
        RCLCPP_ERROR(this->get_logger(),
                     "SpeedRef message too old. Received message stamp: %i[s]+%i[ns], current time: %i[s]+%i[ns]",
                     msg->stamp.sec, msg->stamp.nanosec, now.seconds(), now.nanoseconds());
        return;
    }

    if (canopen_slave_node_ != nullptr) {
        canopen_slave_node_->send_TPDO_2(msg->right_speed_ref, msg->left_speed_ref);
    }
}

void ROS2BridgeNode::vcu_is_alive_timer_ros2_callback(){
    rclcpp::Time now = this->get_clock()->now();

    if(now - last_VCU_message_time_ > rclcpp::Duration(vcu_is_alive_timeout_)){
        RCLCPP_ERROR(this->get_logger(), "VCU COMM TIMEOUT");
        return;
    }

    if(now - last_VCU_alive_bit_change_time_ > rclcpp::Duration(vcu_is_alive_timeout_)){
        RCLCPP_ERROR(this->get_logger(), "VCU ALIVE BIT CHANGE TIMEOUT");
    }

}