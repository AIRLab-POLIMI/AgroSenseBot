#include "asb_system_test_node.h"
#include "VCU_canopen_slave_node.h"
#include "motor_drive_canopen_slave_node.h"

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ASBSystemTestNode::on_configure(const rclcpp_lifecycle::State &) {
  lifecycle_node_active_.store(false);
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ASBSystemTestNode::on_activate(const rclcpp_lifecycle::State &) {
  get_parameter("print_debug", print_debug_);
  get_parameter("use_simulator", use_simulator_);
  get_parameter("dummy_VCU_canopen_node_config", VCU_canopen_node_config_);
  get_parameter("dummy_MDL_canopen_node_config", MDL_canopen_node_config_);
  get_parameter("dummy_MDR_canopen_node_config", MDR_canopen_node_config_);
  get_parameter("dummy_FAN_canopen_node_config", FAN_canopen_node_config_);
  get_parameter("can_interface_name", can_interface_name_);

  RCLCPP_INFO(this->get_logger(), "use_simulator: %s", use_simulator_ ? "True" : "False");
  RCLCPP_INFO(this->get_logger(), "dummy_canopen_node_config: %s", VCU_canopen_node_config_.c_str());
  RCLCPP_INFO(this->get_logger(), "can_interface_name: %s", can_interface_name_.c_str());

  if (can_interface_name_ != "vcan0") {
    RCLCPP_FATAL(this->get_logger(), "Using a CAN interface different than vcan0 for testing!");
    return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::FAILURE;
  }

  control_mode_subscriber_ = this->create_subscription<std_msgs::msg::Int16>(
      "control_mode", rclcpp::SensorDataQoS().reliable(), std::bind(&ASBSystemTestNode::control_mode_ros2_callback, this, _1));
  sim_state_subscriber_ = this->create_subscription<asb_msgs::msg::SimState>(
      "state", rclcpp::SensorDataQoS().reliable(), std::bind(&ASBSystemTestNode::sim_ros2_callback, this, _1));
  sim_state_cmd_publisher_ = this->create_publisher<asb_msgs::msg::SimStateCmd>(
      "state_cmd", rclcpp::SensorDataQoS().reliable());

  lifecycle_node_active_.store(true);

  canopen_nodes_thread_ = std::thread(std::bind(&ASBSystemTestNode::run_canopen_nodes, this));

  std::chrono::duration gcu_is_alive_timer_period_ = 10ms;
  gcu_is_alive_timer_ = rclcpp::create_timer(
          this, this->get_clock(), rclcpp::Duration(gcu_is_alive_timer_period_),
          std::bind(&ASBSystemTestNode::gcu_is_alive_timer_ros2_callback, this));
  last_GCU_message_time_ = this->get_clock()->now();
  last_GCU_alive_bit_change_time_ = this->get_clock()->now();

  std::chrono::duration test_loop_timer_period_ = 10ms;  // must be higher than the timer loop period of the VCU
  test_loop_timer_ = rclcpp::create_timer(
          this, this->get_clock(), rclcpp::Duration(test_loop_timer_period_),
          std::bind(&ASBSystemTestNode::test_loop_timer_ros2_callback, this));

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ASBSystemTestNode::on_deactivate(const rclcpp_lifecycle::State &) {

  gcu_is_alive_timer_->cancel();
  test_loop_timer_->cancel();

  lifecycle_node_active_.store(false);
  canopen_nodes_thread_.join();

  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ASBSystemTestNode::on_cleanup(const rclcpp_lifecycle::State &) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
ASBSystemTestNode::on_shutdown(const rclcpp_lifecycle::State &) {
  return rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn::SUCCESS;
}

void ASBSystemTestNode::run_canopen_nodes() {
  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();

  io::Timer timer_VCU(poll, exec, CLOCK_MONOTONIC);
  io::CanController ctrl_VCU(can_interface_name_.c_str());
  io::CanChannel chan_VCU(poll, exec);
  chan_VCU.open(ctrl_VCU);

  io::Timer timer_MDL(poll, exec, CLOCK_MONOTONIC);
  io::CanController ctrl_MDL(can_interface_name_.c_str());
  io::CanChannel chan_MDL(poll, exec);
  chan_MDL.open(ctrl_MDL);

  io::Timer timer_MDR(poll, exec, CLOCK_MONOTONIC);
  io::CanController ctrl_MDR(can_interface_name_.c_str());
  io::CanChannel chan_MDR(poll, exec);
  chan_MDR.open(ctrl_MDR);

  io::Timer timer_FAN(poll, exec, CLOCK_MONOTONIC);
  io::CanController ctrl_FAN(can_interface_name_.c_str());
  io::CanChannel chan_FAN(poll, exec);
  chan_FAN.open(ctrl_FAN);

  VCU_canopen_slave_node_ = std::make_shared<VCUCANOpenSlaveNode>(
          timer_VCU, chan_VCU, VCU_canopen_node_config_, "", this);

  MDL_canopen_slave_node_ = std::make_shared<MotorDriveCANOpenSlaveNode>(
          timer_MDL, chan_MDL, MDL_canopen_node_config_, "");
  MDL_canopen_slave_node_->node_name_ = "dummy_MDL";

  MDR_canopen_slave_node_ = std::make_shared<MotorDriveCANOpenSlaveNode>(
          timer_MDR, chan_MDR, MDR_canopen_node_config_, "");
  MDR_canopen_slave_node_->node_name_ = "dummy_MDR";

  FAN_canopen_slave_node_ = std::make_shared<MotorDriveCANOpenSlaveNode>(
          timer_FAN, chan_FAN, FAN_canopen_node_config_, "");
  FAN_canopen_slave_node_->node_name_ = "dummy_FAN";

  VCU_canopen_slave_node_->Reset();
  MDL_canopen_slave_node_->Reset();
  MDR_canopen_slave_node_->Reset();
  FAN_canopen_slave_node_->Reset();

  while (lifecycle_node_active_.load()) {
    loop.run_for(1ms);
    VCU_canopen_slave_node_->timer();
    MDL_canopen_slave_node_->timer();
    MDR_canopen_slave_node_->timer();
    FAN_canopen_slave_node_->timer();
  }
  ctx.shutdown();
}

void ASBSystemTestNode::test_loop_timer_ros2_callback() {
  rclcpp::Time now = this->get_clock()->now();
  rclcpp::Duration time_delta = now - last_test_loop_time_;
  last_test_loop_time_ = now;

  if(print_debug_ && control_mode_test_state_ == ControlMode::STOP) {
    RCLCPP_INFO(this->get_logger(), "\ncontrol mode                : STOP\n");
  }
  if(print_debug_ && control_mode_test_state_ == ControlMode::RCU) {
    RCLCPP_INFO(this->get_logger(), "\ncontrol mode                : RCU\n");
  }
  if(print_debug_ && control_mode_test_state_ == ControlMode::GCU) {
    RCLCPP_INFO(this->get_logger(), "\ncontrol mode                : GCU\n");
  }
  if(print_debug_ && control_mode_test_state_ == ControlMode::WAIT) {
    RCLCPP_INFO(this->get_logger(), "\ncontrol mode                : WAIT\n");
  }

  if(control_mode_test_state_ != ControlMode::GCU) {
    pump_test_state_ = false;
    right_motor_drive_test_state_.speed_ref = 0;
    left_motor_drive_test_state_.speed_ref = 0;
    fan_motor_drive_test_state_.speed_ref = 0;
  }

  if(use_simulator_) {
    left_motor_drive_test_state_.apply_motor_speed(time_delta);
  } else {
    left_motor_drive_test_state_.apply_motor_speed_ref(time_delta);
  }
  if(print_debug_) RCLCPP_INFO(
      this->get_logger(),
      "\n"
      "left    time_delta          : %f s\n"
      "        motor_rpm           : %i RPM\n"
      "        rotor_position_raw  : %i raw\n"
      "        rotor_position      : %f rev\n",
      time_delta.seconds(),
      left_motor_drive_test_state_.motor_rpm,
      left_motor_drive_test_state_.rotor_position_raw,
      left_motor_drive_test_state_.rotor_position()
      );

  if(use_simulator_) {
    right_motor_drive_test_state_.apply_motor_speed(time_delta);
  } else {
    right_motor_drive_test_state_.apply_motor_speed_ref(time_delta);
  }
  if(print_debug_) RCLCPP_INFO(
      this->get_logger(),
      "\n"
      "right   motor_rpm           : %i RPM\n"
      "        rotor_position_raw  : %i raw\n"
      "        rotor_position      : %f rev\n",
      right_motor_drive_test_state_.motor_rpm,
      right_motor_drive_test_state_.rotor_position_raw,
      right_motor_drive_test_state_.rotor_position()
      );

  if(use_simulator_) {
    fan_motor_drive_test_state_.apply_motor_speed(time_delta);
  } else {
    fan_motor_drive_test_state_.apply_motor_speed_ref(time_delta);
  }
  if(print_debug_) RCLCPP_INFO(
      this->get_logger(),
      "\n"
      "fan     motor_rpm           : %i RPM\n"
      "        rotor_position_raw  : %i raw\n"
      "        rotor_position      : %f rev\n",
      fan_motor_drive_test_state_.motor_rpm,
      fan_motor_drive_test_state_.rotor_position_raw,
      fan_motor_drive_test_state_.rotor_position()
      );

  vcu_alive_test_callback(pump_test_state_, true,
                          control_mode_test_state_,
                          0, 0);

  motor_drive_left_test_callback(
          42.5, 57.2, left_motor_drive_test_state_.motor_rpm, 0.2,
          0, 63, 50.0, 5,
          false,
          left_motor_drive_test_state_.rotor_position_raw);

  motor_drive_right_test_callback(
          42.5, 57.2, right_motor_drive_test_state_.motor_rpm, 0.2,
          0, 63, 50.0, 5,
          false,
          right_motor_drive_test_state_.rotor_position_raw);

  motor_drive_fan_test_callback(
          42.5, 57.2, fan_motor_drive_test_state_.motor_rpm, 0.2,
          0, 63, 50.0, 5,
          false,
          fan_motor_drive_test_state_.rotor_position_raw);

  asb_msgs::msg::SimStateCmd sim_state_cmd_msg;
  sim_state_cmd_msg.left_motor_speed_ref = 2 * M_PI * left_motor_drive_test_state_.speed_ref / 60.;
  sim_state_cmd_msg.right_motor_speed_ref = 2 * M_PI * right_motor_drive_test_state_.speed_ref / 60.;
  sim_state_cmd_msg.fan_motor_speed_ref = 2 * M_PI * fan_motor_drive_test_state_.speed_ref / 60.;
  sim_state_cmd_publisher_->publish(sim_state_cmd_msg);

}

void ASBSystemTestNode::control_mode_ros2_callback(const std_msgs::msg::Int16::SharedPtr msg) {
  if(print_debug_) RCLCPP_INFO(this->get_logger(), "control_mode: %i\n", msg->data);
  control_mode_test_state_ = (ControlMode) msg->data;
}

void ASBSystemTestNode::sim_ros2_callback(const asb_msgs::msg::SimState::SharedPtr msg) {
  if(print_debug_) RCLCPP_INFO(
      this->get_logger(),
      "sim_state.left_motor_speed  : %f\n"
      "sim_state.right_motor_speed : %f\n"
      "sim_state.fan_motor_speed   : %f\n",
      msg->left_motor_speed,
      msg->right_motor_speed,
      msg->fan_motor_speed
    );
  if(use_simulator_) {
    left_motor_drive_test_state_.motor_rpm = (int) (60 * msg->left_motor_speed / (2 * M_PI));
    right_motor_drive_test_state_.motor_rpm = (int) (60 * msg->right_motor_speed / (2 * M_PI));
    fan_motor_drive_test_state_.motor_rpm = (int) (60 * msg->fan_motor_speed / (2 * M_PI));
  }
}

void ASBSystemTestNode::gcu_is_alive_timer_ros2_callback() {
  rclcpp::Time now = this->get_clock()->now();

  if(comm_started_) {
    if (now - last_GCU_message_time_ > rclcpp::Duration(gcu_is_alive_timeout_)) {
      RCLCPP_ERROR(this->get_logger(), "GCU COMM TIMEOUT (%f s)", (now - last_GCU_message_time_).seconds());
      return;
    }

    if (now - last_GCU_alive_bit_change_time_ > rclcpp::Duration(gcu_is_alive_timeout_)) {
      if(control_mode_test_state_ != ControlMode::STOP) {
        RCLCPP_INFO(this->get_logger(), "GCU ALIVE BIT CHANGE TIMEOUT (%f s)", (now - last_GCU_alive_bit_change_time_).seconds());
      }
      control_mode_test_state_ = ControlMode::STOP;
    }

  } else {
    auto& throttle_clock = *this->get_clock();
    RCLCPP_INFO_THROTTLE(this->get_logger(), throttle_clock, 1000, "WAITING FOR GCU COMM (first GCU alive bit change)");
  }

}

void ASBSystemTestNode::vcu_alive_test_callback(
        bool pump_status_bit, bool vcu_safety_status,
        uint8_t control_mode,
        uint8_t more_recent_alarm_id_to_confirm, uint8_t more_recent_active_alarm_id) {
  VCU_alive_bit_ = not VCU_alive_bit_;

  if (VCU_canopen_slave_node_ != nullptr) {
    VCU_canopen_slave_node_->set_TPDO_1(VCU_alive_bit_, vcu_safety_status, pump_status_bit,
                                         control_mode,
                                         more_recent_alarm_id_to_confirm, more_recent_active_alarm_id);
  }
}

void ASBSystemTestNode::motor_drive_left_test_callback(
        double controller_temperature, double motor_temperature, int motor_rpm, double battery_current_display,
        double motor_torque, double bdi_percentage, double keyswitch_voltage, int zero_speed_threshold,
        bool interlock_status,
        int32_t rotor_position_raw) {
  if (MDL_canopen_slave_node_ != nullptr) {
    MDL_canopen_slave_node_->set_TPDO_1(
            (int16_t) (controller_temperature / RAW_DATA_STEP_VALUE_temperature),
            (int16_t) (motor_temperature / RAW_DATA_STEP_VALUE_temperature),
            (int16_t) motor_rpm,
            (int16_t) (battery_current_display / RAW_DATA_STEP_VALUE_current));
    MDL_canopen_slave_node_->set_TPDO_2(
            (int16_t) (motor_torque / RAW_DATA_STEP_VALUE_torque),
            (int16_t) (bdi_percentage / RAW_DATA_STEP_VALUE_bdi_percentage),
            (int16_t) (keyswitch_voltage / RAW_DATA_STEP_VALUE_voltage),
            (int16_t) zero_speed_threshold);
    MDL_canopen_slave_node_->set_TPDO_3(interlock_status);
    MDL_canopen_slave_node_->set_TPDO_4(rotor_position_raw);
  }

}

void ASBSystemTestNode::motor_drive_right_test_callback(
        double controller_temperature, double motor_temperature, int motor_rpm, double battery_current_display,
        double motor_torque, double bdi_percentage, double keyswitch_voltage, int zero_speed_threshold,
        bool interlock_status,
        int32_t rotor_position_raw) {
  if (MDR_canopen_slave_node_ != nullptr) {
    MDR_canopen_slave_node_->set_TPDO_1(
            (int16_t) (controller_temperature / RAW_DATA_STEP_VALUE_temperature),
            (int16_t) (motor_temperature / RAW_DATA_STEP_VALUE_temperature),
            (int16_t) motor_rpm,
            (int16_t) (battery_current_display / RAW_DATA_STEP_VALUE_current));
    MDR_canopen_slave_node_->set_TPDO_2(
            (int16_t) (motor_torque / RAW_DATA_STEP_VALUE_torque),
            (int16_t) (bdi_percentage / RAW_DATA_STEP_VALUE_bdi_percentage),
            (int16_t) (keyswitch_voltage / RAW_DATA_STEP_VALUE_voltage),
            (int16_t) zero_speed_threshold);
    MDR_canopen_slave_node_->set_TPDO_3(interlock_status);
    MDR_canopen_slave_node_->set_TPDO_4(rotor_position_raw);
  }
}

void ASBSystemTestNode::motor_drive_fan_test_callback(
        double controller_temperature, double motor_temperature, int motor_rpm, double battery_current_display,
        double motor_torque, double bdi_percentage, double keyswitch_voltage, int zero_speed_threshold,
        bool interlock_status,
        int32_t rotor_position_raw) {
  if (FAN_canopen_slave_node_ != nullptr) {
    FAN_canopen_slave_node_->set_TPDO_1(
            (int16_t) (controller_temperature / RAW_DATA_STEP_VALUE_temperature),
            (int16_t) (motor_temperature / RAW_DATA_STEP_VALUE_temperature),
            (int16_t) motor_rpm,
            (int16_t) (battery_current_display / RAW_DATA_STEP_VALUE_current));
    FAN_canopen_slave_node_->set_TPDO_2(
            (int16_t) (motor_torque / RAW_DATA_STEP_VALUE_torque),
            (int16_t) (bdi_percentage / RAW_DATA_STEP_VALUE_bdi_percentage),
            (int16_t) (keyswitch_voltage / RAW_DATA_STEP_VALUE_voltage),
            (int16_t) zero_speed_threshold);
    FAN_canopen_slave_node_->set_TPDO_3(interlock_status);
    FAN_canopen_slave_node_->set_TPDO_4(rotor_position_raw);
  }
}

void ASBSystemTestNode::gcu_alive_canopen_callback(bool GCU_is_alive_bit, bool pump_cmd_bit) {
  rclcpp::Time now = this->get_clock()->now();
  if(print_debug_) RCLCPP_INFO(this->get_logger(), "           GCU COMM AGE (%f s)", (now - last_GCU_message_time_).seconds());
  if(print_debug_) RCLCPP_INFO(this->get_logger(), "           GCU ALIVE BIT CHANGE AGE (%f s)", (now - last_GCU_alive_bit_change_time_).seconds());
  last_GCU_message_time_ = now;
  if (last_GCU_alive_bit_ != GCU_is_alive_bit) {
    last_GCU_alive_bit_change_time_ = now;
    comm_started_ = true;
  }
  last_GCU_alive_bit_ = GCU_is_alive_bit;

  pump_test_state_ = pump_cmd_bit;
}

void ASBSystemTestNode::speed_ref_canopen_callback(int16_t right_speed_ref, int16_t left_speed_ref, int16_t fan_speed_ref) {
  if(control_mode_test_state_ == ControlMode::GCU) {
    right_motor_drive_test_state_.speed_ref = right_speed_ref;
    left_motor_drive_test_state_.speed_ref = left_speed_ref;
    fan_motor_drive_test_state_.speed_ref = fan_speed_ref;
  }
}
