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

#ifndef ASB_CONTROL_SYSTEM_STATUS_CONTROLLER__ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_HPP_
#define ASB_CONTROL_SYSTEM_STATUS_CONTROLLER__ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "std_msgs/msg/header.hpp"
#include "std_msgs/msg/empty.hpp"
#include "std_msgs/msg/bool.hpp"

#include "controller_interface/controller_interface.hpp"
#include "asb_control_system_status_controller/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "realtime_tools/realtime_box.h"
#include "realtime_tools/realtime_buffer.h"
#include "realtime_tools/realtime_publisher.h"

using std::placeholders::_1;
using namespace std::chrono_literals;

namespace asb_control_system_status_controller
{

class ASBControlSystemStatusController : public controller_interface::ControllerInterface {

public:
  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  ASBControlSystemStatusController();

  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

  ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_shutdown(
    const rclcpp_lifecycle::State & previous_state) override;

protected:

  void heartbeat_callback(std::shared_ptr<std_msgs::msg::Header> msg);

  void emergency_stop_cmd_callback(std::shared_ptr<std_msgs::msg::Empty> msg);

  bool reset();

  void halt();

  std::map<std::string, std::shared_ptr<hardware_interface::LoanedStateInterface>> named_state_interface_;
  std::map<std::string, std::shared_ptr<hardware_interface::LoanedCommandInterface>> named_command_interface_;

  // Timeout to consider heartbeat too old
  std::chrono::milliseconds heartbeat_timeout_ = 500ms;

  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Header>> test_publisher_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<std_msgs::msg::Header>::SharedPtr heartbeat_subscriber_ = nullptr;
  rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr emergency_stop_subscriber_ = nullptr;

  // state variables
  std_msgs::msg::Header last_heartbeat_msg_;
  rclcpp::Time startup_time_;

  bool emergency_stop_cmd_ = false;

  bool is_halted = false;

};
}  // namespace asb_control_system_status_controller
#endif  // ASB_CONTROL_SYSTEM_STATUS_CONTROLLER__ASB_CONTROL_SYSTEM_STATUS_CONTROLLER_HPP_
