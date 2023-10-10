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

namespace
{
constexpr auto DEFAULT_HEARTBEAT_TOPIC = "~/heartbeat";
constexpr auto DEFAULT_TEST_TOPIC = "~/test_info";
}  // namespace

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
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

InterfaceConfiguration ASBControlSystemStatusController::state_interface_configuration() const {
  std::vector<std::string> conf_names;
  conf_names.push_back("control_system_state/vcu_comm_ok");
  conf_names.push_back("control_system_state/software_emergency_stop");
  return {interface_configuration_type::INDIVIDUAL, conf_names};
}

controller_interface::CallbackReturn ASBControlSystemStatusController::on_configure(const rclcpp_lifecycle::State &) {
  auto logger = get_node()->get_logger();

  reset();

  heartbeat_subscriber_ = get_node()->create_subscription<std_msgs::msg::Header>(
          DEFAULT_HEARTBEAT_TOPIC, rclcpp::SystemDefaultsQoS(),
          std::bind(&ASBControlSystemStatusController::heartbeat_callback, this, _1));

  emergency_stop_subscriber_ = get_node()->create_subscription<std_msgs::msg::Empty>(
          "~/emergency_stop_cmd", rclcpp::SystemDefaultsQoS(),
          std::bind(&ASBControlSystemStatusController::emergency_stop_cmd_callback, this, _1));

  test_publisher_ = get_node()->create_publisher<std_msgs::msg::Header>(DEFAULT_TEST_TOPIC, rclcpp::SystemDefaultsQoS());

  return controller_interface::CallbackReturn::SUCCESS;
}

controller_interface::CallbackReturn ASBControlSystemStatusController::on_activate(const rclcpp_lifecycle::State &) {
  startup_time_ = get_node()->get_clock()->now();

  for (hardware_interface::LoanedStateInterface& loaned_state_interface : state_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "STATE   prefix_name: %s interface_name: %s, get_interface_name", loaned_state_interface.get_prefix_name().c_str(), loaned_state_interface.get_interface_name().c_str());
    named_state_interface_[loaned_state_interface.get_interface_name()] = static_cast<std::shared_ptr<hardware_interface::LoanedStateInterface>>(&loaned_state_interface);
  }

  for (hardware_interface::LoanedCommandInterface& loaned_command_interface : command_interfaces_) {
    RCLCPP_INFO(get_node()->get_logger(), "COMMAND prefix_name: %s interface_name: %s, get_interface_name", loaned_command_interface.get_prefix_name().c_str(), loaned_command_interface.get_interface_name().c_str());
    named_command_interface_[loaned_command_interface.get_interface_name()] = static_cast<std::shared_ptr<hardware_interface::LoanedCommandInterface>>(&loaned_command_interface);
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
  named_command_interface_["set_software_emergency_stop"]->set_value(emergency_stop_cmd_);

  // publish the state interface values variables
  // TODO
  // RCLCPP_INFO(logger, "vcu_comm_ok: %i", (bool)std::round(named_state_interface_["vcu_comm_ok"]->get_value()));
  // RCLCPP_INFO(logger, "software_emergency_stop: %i", (bool)std::round(named_state_interface_["software_emergency_stop"]->get_value()));

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

void ASBControlSystemStatusController::emergency_stop_cmd_callback(const std::shared_ptr<std_msgs::msg::Empty> /*msg*/) {
  if (!subscriber_is_active_) {
    RCLCPP_WARN(get_node()->get_logger(), "Can't accept new commands. subscriber is inactive");
    return;
  }

  emergency_stop_cmd_ = true;

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
