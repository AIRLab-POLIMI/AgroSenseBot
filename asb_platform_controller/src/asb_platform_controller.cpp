// Copyright 2020 PAL Robotics S.L.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/*
 * Author: Enrico Piazza
 */

#include <memory>
#include <string>
#include <utility>
#include <vector>
#include <functional>

#include "asb_platform_controller/asb_platform_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"


namespace asb_platform_controller
{

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using lifecycle_msgs::msg::State;

enum ControlMode {
  STOP = 0,
  RCU = 1,
  GCU = 2,
  WAIT = 3,
};

ASBPlatformController::ASBPlatformController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ASBPlatformController::on_init() {
  RCLCPP_INFO(get_node()->get_logger(), "heartbeat_timeout_ms: %li", heartbeat_timeout_.count());

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration ASBPlatformController::command_interface_configuration() const {
  std::vector<std::string> conf_names;

  conf_names.emplace_back("platform_state/heartbeat_alive_bit");

  conf_names.emplace_back("platform_state/set_software_emergency_stop");

  conf_names.emplace_back("platform_state/pump_command");

  conf_names.emplace_back("fan_motor_joint/velocity");

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration ASBPlatformController::state_interface_configuration() const {
  std::vector<std::string> conf_names;

  conf_names.emplace_back("left_track_joint/position");
  conf_names.emplace_back("left_track_joint/velocity");

  conf_names.emplace_back("right_track_joint/position");
  conf_names.emplace_back("right_track_joint/velocity");

  conf_names.emplace_back("fan_motor_joint/position");
  conf_names.emplace_back("fan_motor_joint/velocity");

  conf_names.emplace_back("platform_state/vcu_comm_ok");
  conf_names.emplace_back("platform_state/vcu_comm_started");
  conf_names.emplace_back("platform_state/gcu_comm_started");
  conf_names.emplace_back("platform_state/gcu_alive_bit_rate_low");
  conf_names.emplace_back("platform_state/gcu_alive_bit_rate_critical");
  conf_names.emplace_back("platform_state/vcu_safety_status");
  conf_names.emplace_back("platform_state/control_mode");
  conf_names.emplace_back("platform_state/more_recent_alarm_id_to_confirm");
  conf_names.emplace_back("platform_state/more_recent_active_alarm_id");

  conf_names.emplace_back("platform_state/software_emergency_stop");

  conf_names.emplace_back("platform_state/pump_state");

  conf_names.emplace_back("platform_state/left_motor_velocity_setpoint");
  conf_names.emplace_back("platform_state/left_motor_controller_temperature");
  conf_names.emplace_back("platform_state/left_motor_temperature");
  conf_names.emplace_back("platform_state/left_motor_battery_current");
  conf_names.emplace_back("platform_state/left_motor_torque");
  conf_names.emplace_back("platform_state/left_motor_BDI_percentage");
  conf_names.emplace_back("platform_state/left_motor_keyswitch_voltage");
  conf_names.emplace_back("platform_state/left_motor_zero_speed_threshold");

  conf_names.emplace_back("platform_state/right_motor_velocity_setpoint");
  conf_names.emplace_back("platform_state/right_motor_controller_temperature");
  conf_names.emplace_back("platform_state/right_motor_temperature");
  conf_names.emplace_back("platform_state/right_motor_battery_current");
  conf_names.emplace_back("platform_state/right_motor_torque");
  conf_names.emplace_back("platform_state/right_motor_BDI_percentage");
  conf_names.emplace_back("platform_state/right_motor_keyswitch_voltage");
  conf_names.emplace_back("platform_state/right_motor_zero_speed_threshold");

  conf_names.emplace_back("platform_state/fan_motor_velocity_setpoint_rpm");
  conf_names.emplace_back("platform_state/fan_motor_controller_temperature");
  conf_names.emplace_back("platform_state/fan_motor_temperature");
  conf_names.emplace_back("platform_state/fan_motor_battery_current");
  conf_names.emplace_back("platform_state/fan_motor_torque");
  conf_names.emplace_back("platform_state/fan_motor_BDI_percentage");
  conf_names.emplace_back("platform_state/fan_motor_keyswitch_voltage");
  conf_names.emplace_back("platform_state/fan_motor_zero_speed_threshold");


  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn ASBPlatformController::on_configure(const rclcpp_lifecycle::State &) {
  auto logger = get_node()->get_logger();

  reset();

  heartbeat_subscriber_ = get_node()->create_subscription<asb_msgs::msg::Heartbeat>(
          "~/heartbeat", rclcpp::SystemDefaultsQoS(),
          std::bind(&ASBPlatformController::heartbeat_callback, this, _1));

  emergency_stop_cmd_subscriber_ = get_node()->create_subscription<asb_msgs::msg::EmergencyStopCmd>(
          "~/emergency_stop_cmd", rclcpp::SystemDefaultsQoS(),
          std::bind(&ASBPlatformController::emergency_stop_cmd_callback, this, _1));

  pump_cmd_subscriber_ = get_node()->create_subscription<asb_msgs::msg::PumpCmd>(
          "~/pump_cmd", rclcpp::SystemDefaultsQoS(),
          std::bind(&ASBPlatformController::pump_cmd_callback, this, _1));

  fan_cmd_subscriber_ = get_node()->create_subscription<asb_msgs::msg::FanCmd>(
          "~/fan_cmd", rclcpp::SystemDefaultsQoS(),
          std::bind(&ASBPlatformController::fan_cmd_callback, this, _1));

  platform_state_publisher_ = get_node()->create_publisher<asb_msgs::msg::PlatformState>(
          "~/platform_state", rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ASBPlatformController::on_activate(const rclcpp_lifecycle::State &) {
  startup_time_ = get_node()->get_clock()->now();
  pump_cmd_time_ = get_node()->get_clock()->now();
  fan_cmd_time_ = get_node()->get_clock()->now();

  for (auto &interface : state_interfaces_) {
    std::string name = interface.get_prefix_name() + "/" + interface.get_interface_name();
    named_state_interface_.insert(std::pair(name, std::ref(interface)));
  }

  for (hardware_interface::LoanedCommandInterface& interface : command_interfaces_) {
    std::string name = interface.get_prefix_name() + "/" + interface.get_interface_name();
    named_command_interface_.insert(std::pair(name, std::ref(interface)));
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

double ASBPlatformController::get_value(std::string name){
  return named_state_interface_.find(name)->second.get().get_value();
}

void ASBPlatformController::set_value(std::string name, double value){
  named_command_interface_.find(name)->second.get().set_value(value);
}

controller_interface::return_type ASBPlatformController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/){
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

  ControlMode control_mode = (ControlMode)(uint8_t)std::round(get_value("platform_state/control_mode"));

  // check the heartbeat
  const auto heartbeat_age = time - last_heartbeat_msg_.stamp;
  if((last_heartbeat_msg_.stamp.sec == 0) && (last_heartbeat_msg_.stamp.nanosec == 0)) {
    auto throttle_clock = rclcpp::Clock();
    RCLCPP_INFO_THROTTLE(logger, throttle_clock, 10000, "WAITING FIRST HEARTBEAT");
  } else if (heartbeat_age > heartbeat_timeout_ && control_mode == ControlMode::GCU) {
    RCLCPP_INFO(logger, "Heartbeat period too low [%fs]. Control system will transition to STOP.", heartbeat_age.seconds());
  }

  // set the command interfaces from the controller state variables
  set_value("platform_state/heartbeat_alive_bit", last_heartbeat_msg_.alive_bit);

  set_value("platform_state/set_software_emergency_stop", emergency_stop_cmd_);

  if(time - pump_cmd_time_ < pump_cmd_timeout_) {
    set_value("platform_state/pump_command", pump_cmd_);
  } else {
    set_value("platform_state/pump_command", false);
  }

  if(time - fan_cmd_time_ < fan_cmd_timeout_) {
    set_value("fan_motor_joint/velocity", fan_cmd_);
  } else {
    set_value("fan_motor_joint/velocity", 0);
  }

  // publish the state interface values
  asb_msgs::msg::PlatformState platform_state_msg;
  platform_state_msg.stamp = time;
  platform_state_msg.vcu_comm_ok = (bool)std::round(get_value("platform_state/vcu_comm_ok"));
  platform_state_msg.vcu_comm_started = (bool)std::round(get_value("platform_state/vcu_comm_started"));
  platform_state_msg.gcu_comm_started = (bool)std::round(get_value("platform_state/gcu_comm_started"));
  platform_state_msg.gcu_alive_bit_rate_low = (bool)std::round(get_value("platform_state/gcu_alive_bit_rate_low"));
  platform_state_msg.gcu_alive_bit_rate_critical = (bool)std::round(get_value("platform_state/gcu_alive_bit_rate_critical"));
  platform_state_msg.vcu_safety_status = (bool)std::round(get_value("platform_state/vcu_safety_status"));
  platform_state_msg.control_mode = (uint8_t)std::round(get_value("platform_state/control_mode"));
  platform_state_msg.more_recent_alarm_id_to_confirm = (uint8_t)std::round(get_value("platform_state/more_recent_alarm_id_to_confirm"));
  platform_state_msg.more_recent_active_alarm_id = (uint8_t)std::round(get_value("platform_state/more_recent_active_alarm_id"));

  platform_state_msg.software_emergency_stop = (uint8_t)std::round(get_value("platform_state/software_emergency_stop"));

  platform_state_msg.pump_state = (bool)std::round(get_value("platform_state/pump_state"));

  platform_state_msg.left_motor_velocity_setpoint = get_value("platform_state/left_motor_velocity_setpoint");
  platform_state_msg.left_motor_controller_temperature = get_value("platform_state/left_motor_controller_temperature");
  platform_state_msg.left_motor_temperature = get_value("platform_state/left_motor_temperature");
  platform_state_msg.left_motor_battery_current = get_value("platform_state/left_motor_battery_current");
  platform_state_msg.left_motor_torque = get_value("platform_state/left_motor_torque");
  platform_state_msg.left_motor_bdi_percentage = (int)std::round(get_value("platform_state/left_motor_BDI_percentage"));
  platform_state_msg.left_motor_keyswitch_voltage = get_value("platform_state/left_motor_keyswitch_voltage");
  platform_state_msg.left_motor_zero_speed_threshold = (int64_t)get_value("platform_state/left_motor_zero_speed_threshold");

  platform_state_msg.left_motor_position = get_value("left_track_joint/position");
  platform_state_msg.left_motor_velocity = get_value("left_track_joint/velocity");

  platform_state_msg.right_motor_velocity_setpoint = get_value("platform_state/right_motor_velocity_setpoint");
  platform_state_msg.right_motor_controller_temperature = get_value("platform_state/right_motor_controller_temperature");
  platform_state_msg.right_motor_temperature = get_value("platform_state/right_motor_temperature");
  platform_state_msg.right_motor_battery_current = get_value("platform_state/right_motor_battery_current");
  platform_state_msg.right_motor_torque = get_value("platform_state/right_motor_torque");
  platform_state_msg.right_motor_bdi_percentage = (int)std::round(get_value("platform_state/right_motor_BDI_percentage"));
  platform_state_msg.right_motor_keyswitch_voltage = get_value("platform_state/right_motor_keyswitch_voltage");
  platform_state_msg.right_motor_zero_speed_threshold = (int64_t)get_value("platform_state/right_motor_zero_speed_threshold");

  platform_state_msg.right_motor_position = get_value("right_track_joint/position");
  platform_state_msg.right_motor_velocity = get_value("right_track_joint/velocity");

  platform_state_msg.fan_motor_velocity_setpoint_rpm = get_value("platform_state/fan_motor_velocity_setpoint_rpm");
  platform_state_msg.fan_motor_controller_temperature = get_value("platform_state/fan_motor_controller_temperature");
  platform_state_msg.fan_motor_temperature = get_value("platform_state/fan_motor_temperature");
  platform_state_msg.fan_motor_battery_current = get_value("platform_state/fan_motor_battery_current");
  platform_state_msg.fan_motor_torque = get_value("platform_state/fan_motor_torque");
  platform_state_msg.fan_motor_bdi_percentage = (int)std::round(get_value("platform_state/fan_motor_BDI_percentage"));
  platform_state_msg.fan_motor_keyswitch_voltage = get_value("platform_state/fan_motor_keyswitch_voltage");
  platform_state_msg.fan_motor_zero_speed_threshold = (int64_t)get_value("platform_state/fan_motor_zero_speed_threshold");

  platform_state_msg.fan_motor_position = get_value("fan_motor_joint/position");
  platform_state_msg.fan_motor_velocity_rpm = get_value("fan_motor_joint/velocity");

  platform_state_publisher_->publish(platform_state_msg);

  return controller_interface::return_type::OK;
}

void ASBPlatformController::heartbeat_callback(const std::shared_ptr<asb_msgs::msg::Heartbeat> msg) {
  if (!subscriber_is_active_) {
    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }
  if ((msg->stamp.sec == 0) && (msg->stamp.nanosec == 0)) {
    RCLCPP_ERROR(get_node()->get_logger(), "Received heartbeat header with zero timestamp. Ignoring heartbeat message.");
    return;
  }
  if(heartbeat_subscriber_->get_publisher_count() > 1) {
    RCLCPP_ERROR(
      get_node()->get_logger(), "More than one heartbeat publisher is active [%li]. Rejecting all heartbeat messages.",
      heartbeat_subscriber_->get_publisher_count());
    return;
  }
  last_heartbeat_msg_ = *msg;
}

void ASBPlatformController::emergency_stop_cmd_callback(const std::shared_ptr<asb_msgs::msg::EmergencyStopCmd> msg) {
  if (!subscriber_is_active_) {
    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }

  rclcpp::Time now = get_node()->get_clock()->now();
  rclcpp::Duration msg_age = now - msg->stamp;
  if (msg_age > emergency_stop_cmd_timeout_) {
    RCLCPP_WARN(get_node()->get_logger(), "Emergency stop message too old (%f s), enabling emergency stop.", msg_age.seconds());
    emergency_stop_cmd_ = true;
    return;
  }

  emergency_stop_cmd_ = msg->set_software_emergency_stop;

}

void ASBPlatformController::pump_cmd_callback(const std::shared_ptr<asb_msgs::msg::PumpCmd> msg) {
  if (!subscriber_is_active_) {
    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }

  rclcpp::Time now = get_node()->get_clock()->now();
  rclcpp::Duration msg_age = now - msg->stamp;
  if (msg_age > pump_cmd_timeout_) {
    RCLCPP_WARN(get_node()->get_logger(), "Pump command message too old (%f s), disabling pump.", msg_age.seconds());
    pump_cmd_ = false;
    return;
  }

  pump_cmd_ = msg->pump_cmd;
  pump_cmd_time_ = msg->stamp;

}

void ASBPlatformController::fan_cmd_callback(const std::shared_ptr<asb_msgs::msg::FanCmd> msg) {
  if (!subscriber_is_active_) {
    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }

  rclcpp::Time now = get_node()->get_clock()->now();
  rclcpp::Duration msg_age = now - msg->stamp;
  if (msg_age > fan_cmd_timeout_) {
    RCLCPP_WARN(get_node()->get_logger(), "Fan command message too old (%f s), setting fan speed to 0 RPM.", msg_age.seconds());
    fan_cmd_ = 0;
    return;
  }

  fan_cmd_ = msg->velocity_rpm;
  fan_cmd_time_ = msg->stamp;

}

void ASBPlatformController::halt() {
  //  TODO set emergency stop?
}

controller_interface::CallbackReturn ASBPlatformController::on_deactivate( const rclcpp_lifecycle::State &) {
  subscriber_is_active_ = false;
  if (!is_halted) {
    halt();
    is_halted = true;
  }
  std::cout << "ASBPlatformController::on_deactivate" << std::endl;

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ASBPlatformController::on_cleanup(const rclcpp_lifecycle::State &) {
  std::cout << "ASBPlatformController::on_cleanup" << std::endl;
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ASBPlatformController::on_error(const rclcpp_lifecycle::State &) {
  std::cout << "ASBPlatformController::on_error" << std::endl;
  return controller_interface::CallbackReturn::SUCCESS;
}

bool ASBPlatformController::reset() {
  subscriber_is_active_ = false;
  heartbeat_subscriber_.reset();
  is_halted = false;
  return true;
}

}  // namespace asb_platform_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  asb_platform_controller::ASBPlatformController, controller_interface::ControllerInterface)
