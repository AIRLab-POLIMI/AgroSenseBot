#include "ros2_bridge_node.h"
#include "VCU_canopen_slave_node.h"
#include "motor_drive_canopen_slave_node.h"

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_configure(const rclcpp_lifecycle::State &) {
    lifecycle_node_active_.store(false);
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_activate(const rclcpp_lifecycle::State &) {
    get_parameter("dummy_VCU_canopen_node_config", VCU_canopen_node_config_);
    get_parameter("dummy_MDL_canopen_node_config", MDL_canopen_node_config_);
    get_parameter("dummy_MDR_canopen_node_config", MDR_canopen_node_config_);
    get_parameter("dummy_FAN_canopen_node_config", FAN_canopen_node_config_);
    get_parameter("can_interface_name", can_interface_name_);
    RCLCPP_INFO(this->get_logger(), "dummy_canopen_node_config: %s", VCU_canopen_node_config_.c_str());
    RCLCPP_INFO(this->get_logger(), "can_interface_name: %s", can_interface_name_.c_str());

    if(can_interface_name_ != "vcan0"){
        RCLCPP_FATAL(this->get_logger(), "Using a CAN interface different than vcan0 for testing!");
        return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
    }

    lifecycle_node_active_.store(true);
    gcu_alive_pub_->on_activate();
    speed_ref_pub_->on_activate();

    VCU_canopen_node_thread_ = std::thread(std::bind(&ROS2BridgeNode::run_VCU_canopen_node, this));
    MDL_canopen_node_thread_ = std::thread(std::bind(&ROS2BridgeNode::run_MDL_canopen_node, this));
//    MDR_canopen_node_thread_ = std::thread(std::bind(&ROS2BridgeNode::run_MDR_canopen_node, this));
//    FAN_canopen_node_thread_ = std::thread(std::bind(&ROS2BridgeNode::run_FAN_canopen_node, this));

    std::chrono::duration gcu_is_alive_timer_period_ = 10ms;
    gcu_is_alive_timer_ = rclcpp::create_timer(
            this, this->get_clock(), rclcpp::Duration(gcu_is_alive_timer_period_),
            std::bind(&ROS2BridgeNode::gcu_is_alive_timer_ros2_callback, this));
    last_GCU_message_time_ = this->get_clock()->now();
    last_GCU_alive_bit_change_time_ = this->get_clock()->now();

    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ROS2BridgeNode::on_deactivate(const rclcpp_lifecycle::State &) {
    lifecycle_node_active_.store(false);
    VCU_canopen_node_thread_.join();
    MDL_canopen_node_thread_.join();
    MDR_canopen_node_thread_.join();
    FAN_canopen_node_thread_.join();
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

void ROS2BridgeNode::run_VCU_canopen_node() {
  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);
  io::CanController ctrl(can_interface_name_.c_str());
  io::CanChannel chan(poll, exec);
  chan.open(ctrl);

  VCU_canopen_slave_node_ = std::make_shared<VCUCANOpenSlaveNode>(timer, chan,
                                                                  VCU_canopen_node_config_, "", this);
  VCU_canopen_slave_node_->Reset();

  while (lifecycle_node_active_.load()) {
    loop.run_one_for(10ms);
  }
  ctx.shutdown();
}

void ROS2BridgeNode::run_MDL_canopen_node() {
  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);
  io::CanController ctrl(can_interface_name_.c_str());
  io::CanChannel chan(poll, exec);
  chan.open(ctrl);

  MDL_canopen_slave_node_ = std::make_shared<MotorDriveCANOpenSlaveNode>(timer, chan,
                                                                         MDL_canopen_node_config_, "", this);
  MDL_canopen_slave_node_->Reset();

  while (lifecycle_node_active_.load()) {
    loop.run_one_for(10ms);
  }
  ctx.shutdown();
}

void ROS2BridgeNode::run_MDR_canopen_node() {
  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);
  io::CanController ctrl(can_interface_name_.c_str());
  io::CanChannel chan(poll, exec);
  chan.open(ctrl);

  MDR_canopen_slave_node_ = std::make_shared<MotorDriveCANOpenSlaveNode>(timer, chan,
                                                                         MDR_canopen_node_config_, "", this);
  MDR_canopen_slave_node_->Reset();

  while (lifecycle_node_active_.load()) {
    loop.run_one_for(10ms);
  }
  ctx.shutdown();
}

void ROS2BridgeNode::run_FAN_canopen_node() {
  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);
  io::CanController ctrl(can_interface_name_.c_str());
  io::CanChannel chan(poll, exec);
  chan.open(ctrl);

  FAN_canopen_slave_node_ = std::make_shared<MotorDriveCANOpenSlaveNode>(timer, chan,
                                                                         FAN_canopen_node_config_, "", this);
  FAN_canopen_slave_node_->Reset();

  while (lifecycle_node_active_.load()) {
    loop.run_one_for(10ms);
  }
  ctx.shutdown();
}

void ROS2BridgeNode::vcu_alive_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::VCUState::SharedPtr VCU_state_msg) {
    last_VCU_alive_bit_ = not last_VCU_alive_bit_;

    if (VCU_canopen_slave_node_ != nullptr) {
        VCU_canopen_slave_node_->send_TPDO_1(last_VCU_alive_bit_, VCU_state_msg->vcu_safety_status, VCU_state_msg->control_mode);
    }
}

void ROS2BridgeNode::motor_drive_left_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::MotorDrive::SharedPtr msg) {
    if (MDL_canopen_slave_node_ != nullptr) {
      MDL_canopen_slave_node_->send_TPDO_1(
                (int16_t)(msg->controller_temperature * INVERSE_RAW_DATA_STEP_VALUE_temperature),
                (int16_t)(msg->motor_temperature * INVERSE_RAW_DATA_STEP_VALUE_temperature),
                msg->motor_rpm,
                (int16_t)(msg->battery_current_display * INVERSE_RAW_DATA_STEP_VALUE_current));
    }
}

void ROS2BridgeNode::motor_drive_right_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::MotorDrive::SharedPtr msg) {
    if (MDR_canopen_slave_node_ != nullptr) {
        MDR_canopen_slave_node_->send_TPDO_1(
                (int16_t)(msg->controller_temperature * INVERSE_RAW_DATA_STEP_VALUE_temperature),
                (int16_t)(msg->motor_temperature * INVERSE_RAW_DATA_STEP_VALUE_temperature),
                msg->motor_rpm,
                (int16_t)(msg->battery_current_display * INVERSE_RAW_DATA_STEP_VALUE_current));
    }
}

void ROS2BridgeNode::motor_drive_fan_ros2_callback(agrosensebot_canopen_bridge_msgs::msg::MotorDrive::SharedPtr msg) {
    if (FAN_canopen_slave_node_ != nullptr) {
        FAN_canopen_slave_node_->send_TPDO_1(
                (int16_t)(msg->controller_temperature * INVERSE_RAW_DATA_STEP_VALUE_temperature),
                (int16_t)(msg->motor_temperature * INVERSE_RAW_DATA_STEP_VALUE_temperature),
                msg->motor_rpm,
                (int16_t)(msg->battery_current_display * INVERSE_RAW_DATA_STEP_VALUE_current));
    }
}

void ROS2BridgeNode::gcu_is_alive_timer_ros2_callback(){
    rclcpp::Time now = this->get_clock()->now();

    if(now - last_GCU_message_time_ > rclcpp::Duration(gcu_is_alive_timeout_)){
        RCLCPP_ERROR(this->get_logger(), "GCU COMM TIMEOUT");
        return;
    }

    if(now - last_GCU_alive_bit_change_time_ > rclcpp::Duration(gcu_is_alive_timeout_)){
        RCLCPP_ERROR(this->get_logger(), "GCU ALIVE BIT CHANGE TIMEOUT");
    }
}

void ROS2BridgeNode::gcu_alive_canopen_callback(bool GCU_is_alive_bit, bool /*GCU_is_ready_bit*/) {
//    RCLCPP_INFO(this->get_logger(), "gcu_alive_canopen_callback GCU_is_alive_bit: %i, GCU_is_ready_bit: %i", GCU_is_alive_bit, GCU_is_ready_bit);

    rclcpp::Time now = this->get_clock()->now();
    last_GCU_message_time_ = now;
    if (last_GCU_alive_bit_ != GCU_is_alive_bit) last_GCU_alive_bit_change_time_ = now;
    last_GCU_alive_bit_ = GCU_is_alive_bit;
//  TODO pub msg
}

void ROS2BridgeNode::speed_ref_canopen_callback(int16_t right_speed_ref, int16_t left_speed_ref) {
//    RCLCPP_INFO(this->get_logger(), "speed_ref_canopen_callback right_speed_ref: %i, left_speed_ref: %i", right_speed_ref, left_speed_ref);

    agrosensebot_canopen_bridge_msgs::msg::SpeedRef msg;
    msg.stamp = this->get_clock()->now();
    msg.right_speed_ref = right_speed_ref;
    msg.left_speed_ref = left_speed_ref;
    speed_ref_pub_->publish(msg);
}
