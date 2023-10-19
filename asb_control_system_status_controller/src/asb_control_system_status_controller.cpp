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

#include "asb_control_system_status_controller/asb_control_system_status_controller.hpp"
#include "lifecycle_msgs/msg/state.hpp"
#include "rclcpp/logging.hpp"


namespace asb_control_system_status_controller
{

using namespace std::chrono_literals;
using controller_interface::interface_configuration_type;
using controller_interface::InterfaceConfiguration;
using lifecycle_msgs::msg::State;

ASBControlSystemStatusController::ASBControlSystemStatusController() : controller_interface::ControllerInterface() {}

controller_interface::CallbackReturn ASBControlSystemStatusController::on_init() {
  auto heartbeat_timeout_ms = auto_declare("heartbeat_timeout", 200);
  heartbeat_timeout_ = std::chrono::milliseconds(heartbeat_timeout_ms);
  RCLCPP_INFO(get_node()->get_logger(), "heartbeat_timeout_ms: %li", heartbeat_timeout_.count());

  return controller_interface::CallbackReturn::SUCCESS;
}

InterfaceConfiguration ASBControlSystemStatusController::command_interface_configuration() const {
  std::vector<std::string> conf_names;

  conf_names.push_back("control_system_state/set_software_emergency_stop");

  conf_names.push_back("control_system_state/pump_command");

  conf_names.push_back("fan_motor_joint/velocity");

  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration ASBControlSystemStatusController::state_interface_configuration() const {
  std::vector<std::string> conf_names;

  conf_names.push_back("left_track_joint/position");
  conf_names.push_back("left_track_joint/velocity");

  conf_names.push_back("right_track_joint/position");
  conf_names.push_back("right_track_joint/velocity");

  conf_names.push_back("fan_motor_joint/position");
  conf_names.push_back("fan_motor_joint/velocity");

  conf_names.push_back("control_system_state/vcu_comm_ok");
  conf_names.push_back("control_system_state/vcu_safety_status");
  conf_names.push_back("control_system_state/control_mode");

  conf_names.push_back("control_system_state/software_emergency_stop");

  conf_names.push_back("control_system_state/pump_state");

  conf_names.push_back("control_system_state/left_motor_controller_temperature");
  conf_names.push_back("control_system_state/left_motor_temperature");
  conf_names.push_back("control_system_state/left_motor_battery_current");
  conf_names.push_back("control_system_state/left_motor_torque");
  conf_names.push_back("control_system_state/left_motor_BDI_percentage");
  conf_names.push_back("control_system_state/left_motor_keyswitch_voltage");
  conf_names.push_back("control_system_state/left_motor_zero_speed_threshold");

  conf_names.push_back("control_system_state/right_motor_controller_temperature");
  conf_names.push_back("control_system_state/right_motor_temperature");
  conf_names.push_back("control_system_state/right_motor_battery_current");
  conf_names.push_back("control_system_state/right_motor_torque");
  conf_names.push_back("control_system_state/right_motor_BDI_percentage");
  conf_names.push_back("control_system_state/right_motor_keyswitch_voltage");
  conf_names.push_back("control_system_state/right_motor_zero_speed_threshold");

  conf_names.push_back("control_system_state/fan_motor_controller_temperature");
  conf_names.push_back("control_system_state/fan_motor_temperature");
  conf_names.push_back("control_system_state/fan_motor_battery_current");
  conf_names.push_back("control_system_state/fan_motor_torque");
  conf_names.push_back("control_system_state/fan_motor_BDI_percentage");
  conf_names.push_back("control_system_state/fan_motor_keyswitch_voltage");
  conf_names.push_back("control_system_state/fan_motor_zero_speed_threshold");


  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn ASBControlSystemStatusController::on_configure(const rclcpp_lifecycle::State &) {
  auto logger = get_node()->get_logger();

  reset();

  heartbeat_subscriber_ = get_node()->create_subscription<std_msgs::msg::Header>(
          "~/heartbeat", rclcpp::SystemDefaultsQoS(),
          std::bind(&ASBControlSystemStatusController::heartbeat_callback, this, _1));

  emergency_stop_cmd_subscriber_ = get_node()->create_subscription<asb_msgs::msg::EmergencyStopCmd>(
          "~/emergency_stop_cmd", rclcpp::SystemDefaultsQoS(),
          std::bind(&ASBControlSystemStatusController::emergency_stop_cmd_callback, this, _1));

  pump_cmd_subscriber_ = get_node()->create_subscription<asb_msgs::msg::PumpCmd>(
          "~/pump_cmd", rclcpp::SystemDefaultsQoS(),
          std::bind(&ASBControlSystemStatusController::pump_cmd_callback, this, _1));

  fan_cmd_subscriber_ = get_node()->create_subscription<asb_msgs::msg::FanCmd>(
          "~/fan_cmd", rclcpp::SystemDefaultsQoS(),
          std::bind(&ASBControlSystemStatusController::fan_cmd_callback, this, _1));

  control_system_state_publisher_ = get_node()->create_publisher<asb_msgs::msg::ControlSystemState>(
          "~/control_system_state", rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ASBControlSystemStatusController::on_activate(const rclcpp_lifecycle::State &) {
  startup_time_ = get_node()->get_clock()->now();
  pump_cmd_time_ = get_node()->get_clock()->now();
  fan_cmd_time_ = get_node()->get_clock()->now();

  for (hardware_interface::LoanedStateInterface& loaned_state_interface : state_interfaces_) {
    std::string name = loaned_state_interface.get_prefix_name() + "/" + loaned_state_interface.get_interface_name();
    named_state_interface_[name] = static_cast<std::shared_ptr<hardware_interface::LoanedStateInterface>>(&loaned_state_interface);
  }

  for (hardware_interface::LoanedCommandInterface& loaned_command_interface : command_interfaces_) {
    std::string name = loaned_command_interface.get_prefix_name() + "/" + loaned_command_interface.get_interface_name();
    named_command_interface_[name] = static_cast<std::shared_ptr<hardware_interface::LoanedCommandInterface>>(&loaned_command_interface);
  }

  is_halted = false;
  subscriber_is_active_ = true;

  RCLCPP_DEBUG(get_node()->get_logger(), "Subscriber and publisher are now active.");
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::return_type ASBControlSystemStatusController::update(const rclcpp::Time & time, const rclcpp::Duration & /*period*/){
  auto logger = get_node()->get_logger();
  if (get_state().id() == State::PRIMARY_STATE_INACTIVE) {
    if (!is_halted) {
      halt();
      is_halted = true;
    }
    return controller_interface::return_type::OK;
  }

//  RCLCPP_INFO(get_node()->get_logger(), "up time: %f", (time - startup_time_).seconds());

  // check the heartbeat
  const auto heartbeat_age = time - last_heartbeat_msg_.stamp;
  if((last_heartbeat_msg_.stamp.sec == 0) && (last_heartbeat_msg_.stamp.nanosec == 0)) {
//    RCLCPP_WARN(logger, "No Heartbeat received yet");
  } else if (heartbeat_age > heartbeat_timeout_) {  // TODO set software emergency stop !
//    RCLCPP_WARN(logger, "Heartbeat age: %f", heartbeat_age.seconds());
  }

  // set the command interfaces from the controller state variables
  named_command_interface_["control_system_state/set_software_emergency_stop"]->set_value(emergency_stop_cmd_);

  if(time - pump_cmd_time_ < pump_cmd_timeout_) {
    named_command_interface_["control_system_state/pump_command"]->set_value(pump_cmd_);
  } else {
    named_command_interface_["control_system_state/pump_command"]->set_value(false);
  }

  if(time - fan_cmd_time_ < fan_cmd_timeout_) {
    named_command_interface_["fan_motor_joint/velocity"]->set_value(fan_cmd_);
  } else {
    named_command_interface_["fan_motor_joint/velocity"]->set_value(0);
  }

  // publish the state interface values
  asb_msgs::msg::ControlSystemState control_system_state_msg;
  control_system_state_msg.stamp = time;
  control_system_state_msg.vcu_comm_ok = (bool)std::round(named_state_interface_["control_system_state/vcu_comm_ok"]->get_value());
  control_system_state_msg.vcu_safety_status = (bool)std::round(named_state_interface_["control_system_state/vcu_safety_status"]->get_value());
  control_system_state_msg.control_mode = (uint8_t)std::round(named_state_interface_["control_system_state/control_mode"]->get_value());

  control_system_state_msg.software_emergency_stop = (uint8_t)std::round(named_state_interface_["control_system_state/software_emergency_stop"]->get_value());

  control_system_state_msg.pump_state = (bool)std::round(named_state_interface_["control_system_state/pump_state"]->get_value());

  control_system_state_msg.left_motor_controller_temperature = named_state_interface_["control_system_state/left_motor_controller_temperature"]->get_value();
  control_system_state_msg.left_motor_temperature = named_state_interface_["control_system_state/left_motor_temperature"]->get_value();
  control_system_state_msg.left_motor_battery_current = named_state_interface_["control_system_state/left_motor_battery_current"]->get_value();
  control_system_state_msg.left_motor_torque = named_state_interface_["control_system_state/left_motor_torque"]->get_value();
  control_system_state_msg.left_motor_bdi_percentage = std::round(named_state_interface_["control_system_state/left_motor_BDI_percentage"]->get_value());
  control_system_state_msg.left_motor_keyswitch_voltage = named_state_interface_["control_system_state/left_motor_keyswitch_voltage"]->get_value();
  control_system_state_msg.left_motor_zero_speed_threshold = (int64_t)named_state_interface_["control_system_state/left_motor_zero_speed_threshold"]->get_value();

  control_system_state_msg.left_motor_position = named_state_interface_["left_track_joint/position"]->get_value();
  control_system_state_msg.left_motor_velocity = named_state_interface_["left_track_joint/velocity"]->get_value();

  control_system_state_msg.right_motor_controller_temperature = named_state_interface_["control_system_state/right_motor_controller_temperature"]->get_value();
  control_system_state_msg.right_motor_temperature = named_state_interface_["control_system_state/right_motor_temperature"]->get_value();
  control_system_state_msg.right_motor_battery_current = named_state_interface_["control_system_state/right_motor_battery_current"]->get_value();
  control_system_state_msg.right_motor_torque = named_state_interface_["control_system_state/right_motor_torque"]->get_value();
  control_system_state_msg.right_motor_bdi_percentage = std::round(named_state_interface_["control_system_state/right_motor_BDI_percentage"]->get_value());
  control_system_state_msg.right_motor_keyswitch_voltage = named_state_interface_["control_system_state/right_motor_keyswitch_voltage"]->get_value();
  control_system_state_msg.right_motor_zero_speed_threshold = (int64_t)named_state_interface_["control_system_state/right_motor_zero_speed_threshold"]->get_value();

  control_system_state_msg.right_motor_position = named_state_interface_["right_track_joint/position"]->get_value();
  control_system_state_msg.right_motor_velocity = named_state_interface_["right_track_joint/velocity"]->get_value();

  control_system_state_msg.fan_motor_controller_temperature = named_state_interface_["control_system_state/fan_motor_controller_temperature"]->get_value();
  control_system_state_msg.fan_motor_temperature = named_state_interface_["control_system_state/fan_motor_temperature"]->get_value();
  control_system_state_msg.fan_motor_battery_current = named_state_interface_["control_system_state/fan_motor_battery_current"]->get_value();
  control_system_state_msg.fan_motor_torque = named_state_interface_["control_system_state/fan_motor_torque"]->get_value();
  control_system_state_msg.fan_motor_bdi_percentage = std::round(named_state_interface_["control_system_state/fan_motor_BDI_percentage"]->get_value());
  control_system_state_msg.fan_motor_keyswitch_voltage = named_state_interface_["control_system_state/fan_motor_keyswitch_voltage"]->get_value();
  control_system_state_msg.fan_motor_zero_speed_threshold = (int64_t)named_state_interface_["control_system_state/fan_motor_zero_speed_threshold"]->get_value();

  control_system_state_msg.fan_motor_position = named_state_interface_["fan_motor_joint/position"]->get_value();
  control_system_state_msg.fan_motor_velocity_rpm = named_state_interface_["fan_motor_joint/velocity"]->get_value();

  control_system_state_publisher_->publish(control_system_state_msg);

  return controller_interface::return_type::OK;
}

void ASBControlSystemStatusController::heartbeat_callback(const std::shared_ptr<std_msgs::msg::Header> msg) {
  if (!subscriber_is_active_) {
    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }
  if ((msg->stamp.sec == 0) && (msg->stamp.nanosec == 0)) {
    RCLCPP_WARN(get_node()->get_logger(), "Received Header with zero timestamp");
  }
  last_heartbeat_msg_ = *msg;
}

void ASBControlSystemStatusController::emergency_stop_cmd_callback(const std::shared_ptr<asb_msgs::msg::EmergencyStopCmd> msg) {
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

void ASBControlSystemStatusController::pump_cmd_callback(const std::shared_ptr<asb_msgs::msg::PumpCmd> msg) {
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

void ASBControlSystemStatusController::fan_cmd_callback(const std::shared_ptr<asb_msgs::msg::FanCmd> msg) {
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

void ASBControlSystemStatusController::halt() {
  //  TODO set emergency stop?
}

controller_interface::CallbackReturn ASBControlSystemStatusController::on_deactivate( const rclcpp_lifecycle::State &) {
  subscriber_is_active_ = false;
  if (!is_halted) {
    halt();
    is_halted = true;
  }
//  TODO: release hw interfaces?
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ASBControlSystemStatusController::on_cleanup(const rclcpp_lifecycle::State &) {
//  TODO do something here?
  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ASBControlSystemStatusController::on_error(const rclcpp_lifecycle::State &) {
//  TODO do something here?
  return controller_interface::CallbackReturn::SUCCESS;
}

bool ASBControlSystemStatusController::reset() {
  subscriber_is_active_ = false;
  heartbeat_subscriber_.reset();
  is_halted = false;
  return true;
}

controller_interface::CallbackReturn ASBControlSystemStatusController::on_shutdown(const rclcpp_lifecycle::State &) {
  return controller_interface::CallbackReturn::SUCCESS;
}

}  // namespace asb_control_system_status_controller

#include "class_loader/register_macro.hpp"

CLASS_LOADER_REGISTER_CLASS(
  asb_control_system_status_controller::ASBControlSystemStatusController, controller_interface::ControllerInterface)
