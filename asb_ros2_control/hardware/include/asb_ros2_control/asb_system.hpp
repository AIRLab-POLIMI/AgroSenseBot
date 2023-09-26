// Copyright 2021 ros2_control Development Team
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

#ifndef ASB_ROS2_CONTROL__ASB_SYSTEM_HPP_
#define ASB_ROS2_CONTROL__ASB_SYSTEM_HPP_

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/clock.hpp"
#include "rclcpp/duration.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/time.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "asb_ros2_control/visibility_control.h"
#include "asb_ros2_control/canopen_gcu_slave_node.h"
#include "asb_ros2_control/canopen_motor_drive_receiver.h"

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <thread>
#include <bitset>

// Conversion values for current and temperature
#define RAW_DATA_STEP_VALUE_temperature 0.1 // 0.1°C
#define RAW_DATA_STEP_VALUE_current 0.1 // 0.1A
#define RAW_DATA_STEP_VALUE_rotor_position (1/4096.0) // 1/4096 = 0.000244141 rotations, see documentation for Position_Rotor in "Curtis Manual AC F2-T and AC F2-D – FOS 5.0 April 2023", Section 5 — SYSTEM MONITOR MENU, page 144.

using namespace std::chrono_literals;

namespace asb_ros2_control
{
class ASBSystemHardware : public hardware_interface::SystemInterface
{

struct Config
{
    std::string GCU_canopen_node_config;
    std::string motor_left_receiver_canopen_node_config;
    std::string motor_right_receiver_canopen_node_config;
    std::string motor_fan_receiver_canopen_node_config;
    std::string can_interface_name;
    std::chrono::seconds canopen_init_timeout = 5s;
    int tracks_maximum_velocity_rpm_;
    int fan_maximum_velocity_rpm_;
};

public:
  RCLCPP_SHARED_PTR_DEFINITIONS(ASBSystemHardware);

  ASB_ROS2_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_init(const hardware_interface::HardwareInfo & info) override;

  ASB_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  ASB_ROS2_CONTROL_PUBLIC
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  ASB_ROS2_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_configure(const rclcpp_lifecycle::State & previous_state) override;

  ASB_ROS2_CONTROL_PUBLIC
  hardware_interface::CallbackReturn on_cleanup(const rclcpp_lifecycle::State & previous_state) override;

  ASB_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type read(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  ASB_ROS2_CONTROL_PUBLIC
  hardware_interface::return_type write(const rclcpp::Time & time, const rclcpp::Duration & period) override;

  void run_GCU_canopen_node();
  void run_motor_drive_left_receiver_node();
  void run_motor_drive_right_receiver_node();
  void run_motor_drive_fan_receiver_node();

  void timer();

private:
  // Configuration parameters
  Config cfg_;

  // canopen node objects
  std::atomic<bool> lifecycle_state_is_active_ = false;
  std::atomic<bool> GCU_initialized_ = true;
  std::atomic<bool> motor_left_receiver_initialized_ = true;
  std::atomic<bool> motor_right_receiver_initialized_ = true;
  std::atomic<bool> motor_fan_receiver_initialized_ = true;
  std::thread GCU_thread_;
  std::thread motor_left_receiver_thread_;
  std::thread motor_right_receiver_thread_;
  std::thread motor_fan_receiver_thread_;
  std::shared_ptr <CANOpenGCUNode> GCU_ = nullptr;
  std::shared_ptr <CANOpenMotorDriveReceiverNode> motor_left_receiver_ = nullptr;
  std::shared_ptr <CANOpenMotorDriveReceiverNode> motor_right_receiver_ = nullptr;
  std::shared_ptr <CANOpenMotorDriveReceiverNode> motor_fan_receiver_ = nullptr;

  // internal state variables
  bool gcu_alive_bit_current_value_ = false;
  std::chrono::steady_clock::time_point gcu_alive_bit_last_value_change_;
  bool software_emergency_stop_ = false;

  // exported interface for the control system (all variables need to be doubles for ros2_control reasons)
  // control system state and commands
  double vcu_comm_ok_bool_state_ = 1.0;
  double vcu_safety_status_bool_state_ = 0.0;
  double control_mode_int_state_ = 0.0;

  // software emergency stop state and command
  double set_software_emergency_stop_bool_command_ = 0.0;
  double software_emergency_stop_bool_state_ = 0.0;

  // exported interface for pump control
  double pump_bool_command_ = 0.0;
  double pump_bool_state_ = 0.0;

  // exported interface for left track control and additional motor drive information
  double track_left_position_state_ = 0;
  double track_left_velocity_state_ = 0;
  double track_left_velocity_command_ = 0;
  double track_left_controller_temperature_state_ = 0;
  double track_left_motor_temperature_state_ = 0;
  double track_left_battery_current_state_ = 0;

  // exported interface for right track control and additional motor drive information
  double track_right_position_state_ = 0;
  double track_right_velocity_state_ = 0;
  double track_right_velocity_command_ = 0;
  double track_right_controller_temperature_state_ = 0;
  double track_right_motor_temperature_state_ = 0;
  double track_right_battery_current_state_ = 0;

  // exported interface for fan control and additional motor drive information
  double fan_position_revs_state_ = 0;
  double fan_speed_rpm_state_ = 0;
  double fan_speed_ref_rpm_command_ = 0;
  double fan_controller_temperature_state_ = 0;
  double fan_motor_temperature_state_ = 0;
  double fan_battery_current_state_ = 0;

};

}  // namespace asb_ros2_control

#endif  // ASB_ROS2_CONTROL__ASB_SYSTEM_HPP_
