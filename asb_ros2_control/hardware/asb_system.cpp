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

#include "rclcpp/rclcpp.hpp"

#include "asb_ros2_control/asb_system.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"

#include <chrono>
#include <memory>
#include <vector>
#include <filesystem>

namespace asb_ros2_control
{
hardware_interface::CallbackReturn ASBSystemHardware::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // read params from ros2_control xacro
  cfg_.canopen_node_config = info_.hardware_parameters["canopen_node_config"];
  if (std::filesystem::exists(cfg_.canopen_node_config))
  {
    RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
                "canopen_node_config: '%s'", cfg_.canopen_node_config.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "canopen_node_config FILE DOES NOT EXISTS: '%s'", cfg_.canopen_node_config.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.can_interface_name = info_.hardware_parameters["can_interface_name"];
  if (!cfg_.can_interface_name.empty())
  {
    RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
                "can_interface_name: '%s'", cfg_.can_interface_name.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                "can_interface_name can not be an empty string");
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.tracks_maximum_velocity_rpm_ = std::stoi(info_.hardware_parameters["tracks_maximum_velocity_rpm"]);
  if (cfg_.tracks_maximum_velocity_rpm_ > 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
                "tracks_maximum_velocity_rpm: %i", cfg_.tracks_maximum_velocity_rpm_);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "tracks_maximum_velocity_rpm [%i] must be strictly positive", cfg_.tracks_maximum_velocity_rpm_);
    return hardware_interface::CallbackReturn::ERROR;
  }


  for (const hardware_interface::ComponentInfo & joint : info_.joints)
  {
    // ASBSystem has exactly two states and one command interface on each joint
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ASBSystemHardware"),
        "Joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
        joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ASBSystemHardware"),
        "Joint '%s' have %s command interfaces found. '%s' expected.", joint.name.c_str(),
        joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ASBSystemHardware"),
        "Joint '%s' has %zu state interface. 2 expected.", joint.name.c_str(),
        joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ASBSystemHardware"),
        "Joint '%s' have '%s' as first state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces[1].name != hardware_interface::HW_IF_VELOCITY)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("ASBSystemHardware"),
        "Joint '%s' have '%s' as second state interface. '%s' expected.", joint.name.c_str(),
        joint.state_interfaces[1].name.c_str(), hardware_interface::HW_IF_VELOCITY);
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  track_left_joint_name_ = info_.hardware_parameters["left_track_joint_name"];
  if (info_.joints[0].name == track_left_joint_name_)
  {
    RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
                "track_left_joint_name: '%s'", track_left_joint_name_.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "track_left_joint_name [%s] not in ros2_control xacro configuration as first joint", track_left_joint_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  track_right_joint_name_ = info_.hardware_parameters["right_track_joint_name"];
  if (info_.joints[1].name == track_right_joint_name_)
  {
    RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
                "track_right_joint_name: '%s'", track_right_joint_name_.c_str());
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "track_right_joint_name [%s] not in ros2_control xacro configuration as second joint", track_right_joint_name_.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ASBSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  state_interfaces.emplace_back(
                  track_left_joint_name_,
                  hardware_interface::HW_IF_POSITION,
                  &track_left_position_state_);

  state_interfaces.emplace_back(
                  track_left_joint_name_,
                  hardware_interface::HW_IF_VELOCITY,
                  &track_left_velocity_state_);

  state_interfaces.emplace_back(
                  track_right_joint_name_,
                  hardware_interface::HW_IF_POSITION,
                  &track_right_position_state_);

  state_interfaces.emplace_back(
                  track_right_joint_name_,
                  hardware_interface::HW_IF_VELOCITY,
                  &track_right_velocity_state_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ASBSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  command_interfaces.emplace_back(
                  track_left_joint_name_,
                  hardware_interface::HW_IF_VELOCITY,
                  &track_left_velocity_command_);

  command_interfaces.emplace_back(
                  track_right_joint_name_,
                  hardware_interface::HW_IF_VELOCITY,
                  &track_right_velocity_command_);

  return command_interfaces;
}

void ASBSystemHardware::run_canopen_slave_node() {

  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();
  io::Timer timer(poll, exec, CLOCK_MONOTONIC);

  try {
    io::CanController ctrl(cfg_.can_interface_name.c_str());
    io::CanChannel chan(poll, exec);
    chan.open(ctrl);

    canopen_node_ = std::make_shared<CANOpenSlaveNode>(timer, chan, cfg_.canopen_node_config, "");
    canopen_node_->Reset();

    canopen_node_initialized_.store(true);

    while (lifecycle_state_is_active_.load()) {
      loop.run_one_for(10ms);
    }
    RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "ctx.shutdown()");  // TODO this is not executed when shutting down the node by ctrl+C, terminate the thread properly somewhere
    ctx.shutdown();

  } catch (const std::system_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "Could not open CAN network channel [%s]. Check the CAN adapter is set up and working.", e.what());
    canopen_node_initialized_.store(false);
    ctx.shutdown();
    return;
  }
}

hardware_interface::CallbackReturn ASBSystemHardware::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
              "Configuring hardware interface, initializing GCU CANOpen node");
  lifecycle_state_is_active_.store(true);
  canopen_node_initialized_.store(false);
  auto canopen_timeout_start = std::chrono::system_clock::now();

  canopen_node_thread_ = std::thread(std::bind(&ASBSystemHardware::run_canopen_slave_node, this));

  auto now = std::chrono::system_clock::now();
  while(!canopen_node_initialized_.load())
  {
    now = std::chrono::system_clock::now();
    if(now - canopen_timeout_start > cfg_.canopen_init_timeout)
    {
      auto canopen_init_timeout_milliseconds =
              std::chrono::duration_cast<std::chrono::milliseconds>(cfg_.canopen_init_timeout);
      RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                   "slave CANOpen node failed to initialize before timeout (%li ms)",
                   canopen_init_timeout_milliseconds.count());
      lifecycle_state_is_active_.store(false);
      return hardware_interface::CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(10ms);
  }

  auto canopen_init_duration_milliseconds =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - canopen_timeout_start);
  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
              "Successfully activated, it took %li ms (or less)", canopen_init_duration_milliseconds.count());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn ASBSystemHardware::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
              "Cleaning up hardware interface, terminating CANOpen node");
  lifecycle_state_is_active_.store(false);
  canopen_node_thread_.join();
  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "Successfully deactivated");

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::return_type ASBSystemHardware::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // TODO maybe use lock()

  track_left_velocity_state_ = canopen_node_->motor_RPM_left_.load();
  track_right_velocity_state_ = canopen_node_->motor_RPM_right_.load();

  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "read");
  return hardware_interface::return_type::OK;
}

hardware_interface::return_type asb_ros2_control ::ASBSystemHardware::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  double left_speed_ref_rpm = track_left_velocity_command_ * 60 / (2*M_PI);
  double right_speed_ref_rpm = track_right_velocity_command_ * 60 / (2*M_PI);
  auto left_speed_ref_percentage = (int16_t)std::round(100 * left_speed_ref_rpm / cfg_.tracks_maximum_velocity_rpm_);
  auto right_speed_ref_percentage = (int16_t)std::round(100 * right_speed_ref_rpm / cfg_.tracks_maximum_velocity_rpm_);

  canopen_node_->send_TPDO_2(right_speed_ref_percentage, left_speed_ref_percentage);

  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
              "write rad/s      l: %f  r: %f", track_left_velocity_command_, track_right_velocity_command_);
  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
              "write rpm        l: %f  r: %f", left_speed_ref_rpm, right_speed_ref_rpm);
  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
              "write percentage l: %i  r: %i", left_speed_ref_percentage, right_speed_ref_percentage);

  return hardware_interface::return_type::OK;
}

}  // namespace asb_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  asb_ros2_control::ASBSystemHardware, hardware_interface::SystemInterface)
