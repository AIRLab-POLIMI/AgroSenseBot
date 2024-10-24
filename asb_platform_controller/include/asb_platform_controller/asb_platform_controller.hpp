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

#ifndef ASB_PLATFORM_CONTROLLER__ASB_PLATFORM_CONTROLLER_HPP_
#define ASB_PLATFORM_CONTROLLER__ASB_PLATFORM_CONTROLLER_HPP_

#include <chrono>
#include <cmath>
#include <memory>
#include <queue>
#include <string>
#include <vector>

#include "std_msgs/msg/header.hpp"
#include "asb_msgs/msg/platform_state.hpp"
#include "asb_msgs/msg/emergency_stop_cmd.hpp"
#include "asb_msgs/msg/pump_cmd.hpp"
#include "asb_msgs/msg/fan_cmd.hpp"
#include "asb_msgs/msg/heartbeat.hpp"

#include "controller_interface/controller_interface.hpp"
#include "asb_platform_controller/visibility_control.h"
#include "hardware_interface/handle.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"


using std::placeholders::_1;
using namespace std::chrono_literals;

namespace asb_platform_controller
{

class ASBPlatformController : public controller_interface::ControllerInterface {

public:
  ASB_PLATFORM_CONTROLLER_PUBLIC
  ASBPlatformController();

  ASB_PLATFORM_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration command_interface_configuration() const override;

  ASB_PLATFORM_CONTROLLER_PUBLIC
  controller_interface::InterfaceConfiguration state_interface_configuration() const override;

  ASB_PLATFORM_CONTROLLER_PUBLIC
  controller_interface::return_type update(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ASB_PLATFORM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_init() override;

  ASB_PLATFORM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  ASB_PLATFORM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  ASB_PLATFORM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  ASB_PLATFORM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  ASB_PLATFORM_CONTROLLER_PUBLIC
  controller_interface::CallbackReturn on_error(
    const rclcpp_lifecycle::State & previous_state) override;

private:

  double get_value(std::string name);

  void set_value(std::string name, double value);

protected:

  void heartbeat_callback(std::shared_ptr<asb_msgs::msg::Heartbeat> msg);
  void emergency_stop_cmd_callback(std::shared_ptr<asb_msgs::msg::EmergencyStopCmd> msg);
  void pump_cmd_callback(std::shared_ptr<asb_msgs::msg::PumpCmd> msg);
  void fan_cmd_callback(std::shared_ptr<asb_msgs::msg::FanCmd> msg);

  bool reset();

  void halt();


  std::map<std::string, std::reference_wrapper<hardware_interface::LoanedStateInterface>> named_state_interface_;
  std::map<std::string, std::reference_wrapper<hardware_interface::LoanedCommandInterface>> named_command_interface_;

  // Timeout to consider messages too old
  std::chrono::milliseconds heartbeat_timeout_ = 200ms;
  std::chrono::milliseconds emergency_stop_cmd_timeout_ = 100ms;
  std::chrono::milliseconds pump_cmd_timeout_ = 100ms;
  std::chrono::milliseconds fan_cmd_timeout_ = 100ms;

  std::shared_ptr<rclcpp::Publisher<asb_msgs::msg::PlatformState>> platform_state_publisher_ = nullptr;

  bool subscriber_is_active_ = false;
  rclcpp::Subscription<asb_msgs::msg::Heartbeat>::SharedPtr heartbeat_subscriber_ = nullptr;
  rclcpp::Subscription<asb_msgs::msg::EmergencyStopCmd>::SharedPtr emergency_stop_cmd_subscriber_ = nullptr;
  rclcpp::Subscription<asb_msgs::msg::PumpCmd>::SharedPtr pump_cmd_subscriber_ = nullptr;
  rclcpp::Subscription<asb_msgs::msg::FanCmd>::SharedPtr fan_cmd_subscriber_ = nullptr;

  // state variables
  asb_msgs::msg::Heartbeat last_heartbeat_msg_;
  rclcpp::Time startup_time_;

  bool emergency_stop_cmd_ = false;
  bool pump_cmd_ = false;
  rclcpp::Time pump_cmd_time_;
  int16_t fan_cmd_ = 0;
  rclcpp::Time fan_cmd_time_;

  bool is_halted = false;

};
}  // namespace asb_platform_controller
#endif  // ASB_PLATFORM_CONTROLLER__ASB_PLATFORM_CONTROLLER_HPP_
