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
#include <bitset>
#include <functional>

namespace asb_ros2_control
{

enum ControlMode {
  STOP = 0,
  RCU = 1,
  GCU = 2,
  WAIT = 3,
};

hardware_interface::CallbackReturn ASBSystemHardware::on_init(const hardware_interface::HardwareInfo & info)
{

  rclcpp::on_shutdown(std::bind(&ASBSystemHardware::shutdown_callback, this));

  if (hardware_interface::SystemInterface::on_init(info) != hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // read params from ros2_control xacro
  cfg_.GCU_canopen_node_config = info_.hardware_parameters["GCU_canopen_node_config"];
  if (!std::filesystem::exists(cfg_.GCU_canopen_node_config))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "GCU_canopen_node_config FILE DOES NOT EXISTS: '%s'", cfg_.GCU_canopen_node_config.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.motor_left_receiver_canopen_node_config = info_.hardware_parameters["motor_left_receiver_canopen_node_config"];
  if (!std::filesystem::exists(cfg_.motor_left_receiver_canopen_node_config))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "motor_left_receiver_canopen_node_config FILE DOES NOT EXISTS: '%s'", cfg_.motor_left_receiver_canopen_node_config.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.motor_right_receiver_canopen_node_config = info_.hardware_parameters["motor_right_receiver_canopen_node_config"];
  if (!std::filesystem::exists(cfg_.motor_right_receiver_canopen_node_config))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "motor_right_receiver_canopen_node_config FILE DOES NOT EXISTS: '%s'", cfg_.motor_right_receiver_canopen_node_config.c_str());
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.motor_fan_receiver_canopen_node_config = info_.hardware_parameters["motor_fan_receiver_canopen_node_config"];
  if (!std::filesystem::exists(cfg_.motor_fan_receiver_canopen_node_config))
  {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "motor_fan_receiver_canopen_node_config FILE DOES NOT EXISTS: '%s'", cfg_.motor_fan_receiver_canopen_node_config.c_str());
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

  cfg_.tracks_maximum_velocity_rpm = (int16_t)std::stoi(info_.hardware_parameters["tracks_maximum_velocity_rpm"]);
  if (cfg_.tracks_maximum_velocity_rpm > 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
                "tracks_maximum_velocity_rpm: %i", cfg_.tracks_maximum_velocity_rpm);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "tracks_maximum_velocity_rpm [%i] must be strictly positive", cfg_.tracks_maximum_velocity_rpm);
    return hardware_interface::CallbackReturn::ERROR;
  }

  cfg_.fan_maximum_velocity_rpm = (int16_t)std::stoi(info_.hardware_parameters["fan_maximum_velocity_rpm"]);
  if (cfg_.fan_maximum_velocity_rpm > 0)
  {
    RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
                "fan_maximum_velocity_rpm: %i", cfg_.fan_maximum_velocity_rpm);
  } else {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "fan_maximum_velocity_rpm [%i] must be strictly positive", cfg_.fan_maximum_velocity_rpm);
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

  if (info_.gpios[0].name != "platform_state")
  {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "platform_state gpio param in ros2_control.xacro config file");
    return hardware_interface::CallbackReturn::ERROR;
  }

  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface> ASBSystemHardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  // left joint
  state_interfaces.emplace_back("left_track_joint", hardware_interface::HW_IF_POSITION, &track_left_position_state_);
  state_interfaces.emplace_back("left_track_joint", hardware_interface::HW_IF_VELOCITY, &track_left_velocity_state_);

  // right joint
  state_interfaces.emplace_back("right_track_joint", hardware_interface::HW_IF_POSITION, &track_right_position_state_);
  state_interfaces.emplace_back("right_track_joint", hardware_interface::HW_IF_VELOCITY, &track_right_velocity_state_);

  // fan motor command interface
  state_interfaces.emplace_back("fan_motor_joint", hardware_interface::HW_IF_POSITION, &fan_position_revs_state_);
  state_interfaces.emplace_back("fan_motor_joint", hardware_interface::HW_IF_VELOCITY, &fan_speed_rpm_state_);

  // control system state
  state_interfaces.emplace_back("platform_state", "vcu_comm_ok", &vcu_comm_ok_bool_state_);
  state_interfaces.emplace_back("platform_state", "vcu_comm_started", &vcu_comm_started_bool_state_);
  state_interfaces.emplace_back("platform_state", "gcu_comm_started", &gcu_comm_started_bool_state_);
  state_interfaces.emplace_back("platform_state", "gcu_alive_bit_rate_low", &gcu_alive_bit_rate_low_bool_state_);
  state_interfaces.emplace_back("platform_state", "gcu_alive_bit_rate_critical", &gcu_alive_bit_rate_critical_bool_state_);
  state_interfaces.emplace_back("platform_state", "vcu_safety_status", &vcu_safety_status_bool_state_);
  state_interfaces.emplace_back("platform_state", "control_mode", &control_mode_int_state_);
  state_interfaces.emplace_back("platform_state", "more_recent_alarm_id_to_confirm", &more_recent_alarm_id_to_confirm_int_state_);
  state_interfaces.emplace_back("platform_state", "more_recent_active_alarm_id", &more_recent_active_alarm_id_int_state_);

  // software emergency stop
  state_interfaces.emplace_back("platform_state", "software_emergency_stop", &software_emergency_stop_bool_state_);

  // pump
  state_interfaces.emplace_back("platform_state", "pump_state", &pump_bool_state_);

  // left motor additional information
  state_interfaces.emplace_back("platform_state", "left_motor_velocity_setpoint", &track_left_velocity_setpoint_state_);
  state_interfaces.emplace_back("platform_state", "left_motor_controller_temperature", &track_left_controller_temperature_state_);
  state_interfaces.emplace_back("platform_state", "left_motor_temperature", &track_left_motor_temperature_state_);
  state_interfaces.emplace_back("platform_state", "left_motor_battery_current", &track_left_battery_current_state_);
  state_interfaces.emplace_back("platform_state", "left_motor_torque", &track_left_motor_torque_state_);
  state_interfaces.emplace_back("platform_state", "left_motor_BDI_percentage", &track_left_BDI_percentage_state_);
  state_interfaces.emplace_back("platform_state", "left_motor_keyswitch_voltage", &track_left_keyswitch_voltage_state_);
  state_interfaces.emplace_back("platform_state", "left_motor_zero_speed_threshold", &track_left_zero_speed_threshold_state_);
  state_interfaces.emplace_back("platform_state", "left_motor_interlock", &track_left_interlock_bool_state_);

  // right motor additional information
  state_interfaces.emplace_back("platform_state", "right_motor_velocity_setpoint", &track_right_velocity_setpoint_state_);
  state_interfaces.emplace_back("platform_state", "right_motor_controller_temperature", &track_right_controller_temperature_state_);
  state_interfaces.emplace_back("platform_state", "right_motor_temperature", &track_right_motor_temperature_state_);
  state_interfaces.emplace_back("platform_state", "right_motor_battery_current", &track_right_battery_current_state_);
  state_interfaces.emplace_back("platform_state", "right_motor_torque", &track_right_motor_torque_state_);
  state_interfaces.emplace_back("platform_state", "right_motor_BDI_percentage", &track_right_BDI_percentage_state_);
  state_interfaces.emplace_back("platform_state", "right_motor_keyswitch_voltage", &track_right_keyswitch_voltage_state_);
  state_interfaces.emplace_back("platform_state", "right_motor_zero_speed_threshold", &track_right_zero_speed_threshold_state_);
  state_interfaces.emplace_back("platform_state", "right_motor_interlock", &track_right_interlock_bool_state_);

  // fan motor additional information
  state_interfaces.emplace_back("platform_state", "fan_motor_velocity_setpoint_rpm", &fan_speed_setpoint_rpm_state_);
  state_interfaces.emplace_back("platform_state", "fan_motor_controller_temperature", &fan_controller_temperature_state_);
  state_interfaces.emplace_back("platform_state", "fan_motor_temperature", &fan_motor_temperature_state_);
  state_interfaces.emplace_back("platform_state", "fan_motor_battery_current", &fan_battery_current_state_);
  state_interfaces.emplace_back("platform_state", "fan_motor_torque", &fan_motor_torque_state_);
  state_interfaces.emplace_back("platform_state", "fan_motor_BDI_percentage", &fan_BDI_percentage_state_);
  state_interfaces.emplace_back("platform_state", "fan_motor_keyswitch_voltage", &fan_keyswitch_voltage_state_);
  state_interfaces.emplace_back("platform_state", "fan_motor_zero_speed_threshold", &fan_zero_speed_threshold_state_);
  state_interfaces.emplace_back("platform_state", "fan_motor_interlock", &fan_interlock_bool_state_);

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface> ASBSystemHardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  // left and right track command interface
  command_interfaces.emplace_back("left_track_joint", hardware_interface::HW_IF_VELOCITY, &track_left_velocity_command_);
  command_interfaces.emplace_back("right_track_joint", hardware_interface::HW_IF_VELOCITY, &track_right_velocity_command_);

  // fan motor command interface
  command_interfaces.emplace_back("fan_motor_joint", hardware_interface::HW_IF_VELOCITY, &fan_speed_ref_rpm_command_);

  // heartbeat alive bit command interface
  command_interfaces.emplace_back("platform_state", "heartbeat_alive_bit", &heartbeat_alive_bit_bool_command_);

  // software emergency stop command interface
  command_interfaces.emplace_back("platform_state", "set_software_emergency_stop", &set_software_emergency_stop_bool_command_);

  // pump command interface
  command_interfaces.emplace_back("platform_state", "pump_command", &pump_bool_command_);

  return command_interfaces;
}

void ASBSystemHardware::timer() {
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  if(!GCU_->VCU_comm_ok_.load())
  {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"), "VCU COMM TIMEOUT");
  }

  if(first_heartbeat_received_.load())
  {
    if ((ControlMode) (uint8_t) control_mode_int_state_ == ControlMode::GCU)
    {
      // if we are in control mode GCU, the alive bit should always change within the appropriate timing
      if (now - gcu_alive_bit_last_value_change_.load() >= 200ms)
      {
        gcu_alive_bit_rate_critical_.store(true);
        gcu_alive_bit_rate_low_.store(true);
        RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"), "ALIVE BIT CHANGE PERIOD TOO LOW, VCU GO TO CONTROL MODE STOP");
      } else if (now - gcu_alive_bit_last_value_change_.load() >= 100ms) {
        gcu_alive_bit_rate_critical_.store(false);
        gcu_alive_bit_rate_low_.store(true);
      } else {
        gcu_alive_bit_rate_critical_.store(false);
        gcu_alive_bit_rate_low_.store(false);
      }
    } else {
      // if we are not in control mode GCU, the alive bit does not need to change, it may or not
      gcu_alive_bit_rate_critical_.store(false);
      gcu_alive_bit_rate_low_.store(false);
    }
  } else {
    auto throttle_clock = rclcpp::Clock();
    RCLCPP_INFO_THROTTLE(rclcpp::get_logger("ASBSystemHardware"), throttle_clock, 10000, "WAITING FIRST HEARTBEAT");
  }

  // The VCU CANOpen node considers the received data correct only if the bIsAlive bit in the TPDO1 of the GCU is
  // flipped with a period between 20ms and 100ms. Therefore, we send the TPDO1 to the VCU periodically, but only if the
  // software_emergency_stop is not enabled.
  if (!software_emergency_stop_.load() && first_heartbeat_received_.load())
  {
    GCU_->set_TPDO_1(gcu_alive_bit_current_value_.load(), (bool)std::round(pump_bool_command_));
  } else {
    // if a software emergency is requested, send a 0 velocity command to the control system, disable the pump, and stop doing anything
    int16_t left_speed_ref = 0;
    int16_t right_speed_ref = 0;
    int16_t fan_speed_ref = 0;
    GCU_->set_TPDO_1(gcu_alive_bit_current_value_.load(), false);
    GCU_->set_TPDO_2(right_speed_ref, left_speed_ref, fan_speed_ref);
    // TODO check on actual machine:
    //  sending a speed ref command (TPDO2), even though it's 0, while switching from control mode STOP or RCU to GCU may prevent from
    //  switching control mode.
  }
}

void ASBSystemHardware::run_canopen_nodes() {

  io::IoGuard io_guard;
  io::Context ctx;
  io::Poll poll(ctx);
  ev::Loop loop(poll.get_poll());
  auto exec = loop.get_executor();

  try {
    io::Timer timer_GCU(poll, exec, CLOCK_MONOTONIC);
    io::CanController ctrl_GCU(cfg_.can_interface_name.c_str());
    io::CanChannel chan_GCU(poll, exec);
    chan_GCU.open(ctrl_GCU);

    io::Timer timer_MDL_receiver(poll, exec, CLOCK_MONOTONIC);
    io::CanController ctrl_MDL_receiver(cfg_.can_interface_name.c_str());
    io::CanChannel chan_MDL_receiver(poll, exec);
    chan_MDL_receiver.open(ctrl_MDL_receiver);

    io::Timer timer_MDR_receiver(poll, exec, CLOCK_MONOTONIC);
    io::CanController ctrl_MDR_receiver(cfg_.can_interface_name.c_str());
    io::CanChannel chan_MDR_receiver(poll, exec);
    chan_MDR_receiver.open(ctrl_MDR_receiver);

    io::Timer timer_FAN_receiver(poll, exec, CLOCK_MONOTONIC);
    io::CanController ctrl_FAN_receiver(cfg_.can_interface_name.c_str());
    io::CanChannel chan_FAN_receiver(poll, exec);
    chan_FAN_receiver.open(ctrl_FAN_receiver);

    try{
      GCU_ = std::make_shared<CANOpenGCUNode>(timer_GCU, chan_GCU, cfg_.GCU_canopen_node_config, "");
      GCU_->node_name_ = "GCU";
      GCU_->Reset();
      RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "GCU CANOpen node initialized");

      motor_left_receiver_ = std::make_shared<CANOpenMotorDriveReceiverNode>(
              timer_MDL_receiver, chan_MDL_receiver, cfg_.motor_left_receiver_canopen_node_config, "");
      motor_left_receiver_->node_name_ = "MDL_receiver";
      motor_left_receiver_->Reset();
      RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "MDL_receiver CANOpen node initialized");

      motor_right_receiver_ = std::make_shared<CANOpenMotorDriveReceiverNode>(
              timer_MDR_receiver, chan_MDR_receiver, cfg_.motor_right_receiver_canopen_node_config, "");
      motor_right_receiver_->node_name_ = "MDR_receiver";
      motor_right_receiver_->Reset();
      RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "MDR_receiver CANOpen node initialized");

      motor_fan_receiver_ = std::make_shared<CANOpenMotorDriveReceiverNode>(
              timer_FAN_receiver, chan_FAN_receiver, cfg_.motor_fan_receiver_canopen_node_config, "");
      motor_fan_receiver_->node_name_ = "FAN_receiver";
      motor_fan_receiver_->Reset();
      RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "FAN_receiver CANOpen node initialized");

      canopen_nodes_initialized_.store(true);

      try {
        while (!end_canopen_thread_.load()) {
          loop.run_for(10ms);
          this->timer();
          GCU_->timer();
        }
      } catch (const std::system_error& e) {
        RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                     "An exception occurred in the CANOpen loop function [%s].", e.what());
        exit(1);  // TODO: this may be too brutal, it kills the ros2_control_node (package: controller_manager)
      }

    } catch (const std::system_error& e) {
      RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                   "An exception occurred while initializing the CANOpen nodes [%s].", e.what());
      RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "ctx.shutdown()");
      canopen_initialization_error_.store(true);
      return;
    }

  } catch (const std::system_error& e) {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                 "Could not open CAN network channel [%s]. Check the CAN adapter is set up and working.", e.what());
    RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "ctx.shutdown()");
    canopen_initialization_error_.store(true);
    return;
  }

}

hardware_interface::CallbackReturn ASBSystemHardware::on_configure(const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "Configuring hardware interface, initializing CANOpen nodes");
  auto canopen_timeout_start = std::chrono::system_clock::now();

  canopen_nodes_thread_ = std::thread(std::bind(&ASBSystemHardware::run_canopen_nodes, this));

  auto now = std::chrono::system_clock::now();
  while(!canopen_nodes_initialized_.load() && !canopen_initialization_error_.load())
  {
    now = std::chrono::system_clock::now();
    if(now - canopen_timeout_start > cfg_.canopen_init_timeout)
    {
      auto canopen_init_timeout_milliseconds =
              std::chrono::duration_cast<std::chrono::milliseconds>(cfg_.canopen_init_timeout);
      RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"),
                   "slave CANOpen node failed to initialize before timeout (%li ms)",
                   canopen_init_timeout_milliseconds.count());
      end_canopen_thread_.store(true);
      return hardware_interface::CallbackReturn::ERROR;
    }
    rclcpp::sleep_for(10ms);
  }

  if(canopen_initialization_error_.load()) {
    RCLCPP_ERROR(rclcpp::get_logger("ASBSystemHardware"), "Failed to initialize CANOpen nodes");
    end_canopen_thread_.store(true);
    return hardware_interface::CallbackReturn::ERROR;
  }

  auto canopen_init_duration_milliseconds =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - canopen_timeout_start);
  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"),
              "Successfully activated, it took %li ms (or less)", canopen_init_duration_milliseconds.count());

  return hardware_interface::CallbackReturn::SUCCESS;
}

void ASBSystemHardware::shutdown_callback() {
  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "shutdown_callback: terminating CANOpen nodes");
  end_canopen_thread_.store(true);
  canopen_nodes_thread_.join();
  RCLCPP_INFO(rclcpp::get_logger("ASBSystemHardware"), "shutdown_callback: finished cleaning up");
}

hardware_interface::return_type ASBSystemHardware::read(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (end_canopen_thread_.load()) return hardware_interface::return_type::OK;

  // control system (i.e., the VCU CANOpen node)
  vcu_comm_ok_bool_state_ = GCU_->VCU_comm_ok_.load();
  vcu_comm_started_bool_state_ = GCU_->VCU_comm_started_.load();
  gcu_comm_started_bool_state_ = first_heartbeat_received_.load();
  gcu_alive_bit_rate_low_bool_state_ = gcu_alive_bit_rate_low_.load();
  gcu_alive_bit_rate_critical_bool_state_ = gcu_alive_bit_rate_critical_.load();
  vcu_safety_status_bool_state_ = GCU_->VCU_safety_status_bit_.load();
  control_mode_int_state_ = GCU_->control_mode_.load();
  more_recent_alarm_id_to_confirm_int_state_ = GCU_->more_recent_alarm_id_to_confirm_.load();
  more_recent_active_alarm_id_int_state_ = GCU_->more_recent_active_alarm_id_.load();

  // software emergency stop
  software_emergency_stop_bool_state_ = software_emergency_stop_.load();

  // pump
  pump_bool_state_ = GCU_->VCU_pump_status_bit_.load();

  // compute differential rotor position to avoid jumps when the rotor position raw data over/underflows
  long left_rotor_position_raw = motor_left_receiver_->rotor_position_.load();
  long left_rotor_position_raw_delta = left_rotor_position_raw - prev_left_rotor_position_raw_;
  if(left_rotor_position_raw_delta < -RAW_DATA_ROTOR_POSITION_MAX) left_rotor_position_raw_delta += 2 * RAW_DATA_ROTOR_POSITION_MAX;
  if(left_rotor_position_raw_delta > RAW_DATA_ROTOR_POSITION_MAX) left_rotor_position_raw_delta -= 2 * RAW_DATA_ROTOR_POSITION_MAX;
  prev_left_rotor_position_raw_ = left_rotor_position_raw;

  // left motor state
  track_left_position_state_ += (double)left_rotor_position_raw_delta * 2 * M_PI * RAW_DATA_STEP_VALUE_rotor_position;
  track_left_velocity_state_ = motor_left_receiver_->motor_RPM_.load() * 2 * M_PI / 60.0;
  track_left_velocity_setpoint_state_ = track_left_velocity_command_;
  track_left_controller_temperature_state_ = motor_left_receiver_->controller_temperature_.load() * RAW_DATA_STEP_VALUE_temperature;
  track_left_motor_temperature_state_ = motor_left_receiver_->motor_temperature_.load() * RAW_DATA_STEP_VALUE_temperature;
  track_left_battery_current_state_ = motor_left_receiver_->battery_current_display_.load() * RAW_DATA_STEP_VALUE_current;
  track_left_motor_torque_state_ = motor_left_receiver_->motor_torque_.load() * RAW_DATA_STEP_VALUE_torque;
  track_left_BDI_percentage_state_ = motor_left_receiver_->BDI_percentage_.load() * RAW_DATA_STEP_VALUE_bdi_percentage;
  track_left_keyswitch_voltage_state_ = motor_left_receiver_->keyswitch_voltage_.load() * RAW_DATA_STEP_VALUE_voltage;
  track_left_zero_speed_threshold_state_ = motor_left_receiver_->zero_speed_threshold_.load();
  track_left_interlock_bool_state_ = motor_left_receiver_->interlock_status_.load();

  // compute differential rotor position to avoid jumps when the rotor position raw data over/underflows
  long right_rotor_position_raw = motor_right_receiver_->rotor_position_.load();
  long right_rotor_position_raw_delta = right_rotor_position_raw - prev_right_rotor_position_raw_;
  if(right_rotor_position_raw_delta < -RAW_DATA_ROTOR_POSITION_MAX) right_rotor_position_raw_delta += 2 * RAW_DATA_ROTOR_POSITION_MAX;
  if(right_rotor_position_raw_delta > RAW_DATA_ROTOR_POSITION_MAX) right_rotor_position_raw_delta -= 2 * RAW_DATA_ROTOR_POSITION_MAX;
  prev_right_rotor_position_raw_ = right_rotor_position_raw;

  // right motor state
  track_right_position_state_ += (double)right_rotor_position_raw_delta * 2 * M_PI * RAW_DATA_STEP_VALUE_rotor_position;
  track_right_velocity_state_ = motor_right_receiver_->motor_RPM_.load() * 2 * M_PI / 60;
  track_right_velocity_setpoint_state_ = track_right_velocity_command_;
  track_right_controller_temperature_state_ = motor_right_receiver_->controller_temperature_.load() * RAW_DATA_STEP_VALUE_temperature;
  track_right_motor_temperature_state_ = motor_right_receiver_->motor_temperature_.load() * RAW_DATA_STEP_VALUE_temperature;
  track_right_battery_current_state_ = motor_right_receiver_->battery_current_display_.load() * RAW_DATA_STEP_VALUE_current;
  track_right_motor_torque_state_ = motor_right_receiver_->motor_torque_.load() * RAW_DATA_STEP_VALUE_torque;
  track_right_BDI_percentage_state_ = motor_right_receiver_->BDI_percentage_.load() * RAW_DATA_STEP_VALUE_bdi_percentage;
  track_right_keyswitch_voltage_state_ = motor_right_receiver_->keyswitch_voltage_.load() * RAW_DATA_STEP_VALUE_voltage;
  track_right_zero_speed_threshold_state_ = motor_right_receiver_->zero_speed_threshold_.load();
  track_right_interlock_bool_state_ = motor_right_receiver_->interlock_status_.load();

  // fan motor state
  fan_position_revs_state_ = motor_fan_receiver_->rotor_position_.load() * RAW_DATA_STEP_VALUE_rotor_position;
  fan_speed_rpm_state_ = motor_fan_receiver_->motor_RPM_.load();
  fan_speed_setpoint_rpm_state_ = fan_speed_ref_rpm_command_;
  fan_controller_temperature_state_ = motor_fan_receiver_->controller_temperature_.load() * RAW_DATA_STEP_VALUE_temperature;
  fan_motor_temperature_state_ = motor_fan_receiver_->motor_temperature_.load() * RAW_DATA_STEP_VALUE_temperature;
  fan_battery_current_state_ = motor_fan_receiver_->battery_current_display_.load() * RAW_DATA_STEP_VALUE_current;
  fan_motor_torque_state_ = motor_fan_receiver_->motor_torque_.load() * RAW_DATA_STEP_VALUE_torque;
  fan_BDI_percentage_state_ = motor_fan_receiver_->BDI_percentage_.load() * RAW_DATA_STEP_VALUE_bdi_percentage;
  fan_keyswitch_voltage_state_ = motor_fan_receiver_->keyswitch_voltage_.load() * RAW_DATA_STEP_VALUE_voltage;
  fan_zero_speed_threshold_state_ = motor_fan_receiver_->zero_speed_threshold_.load();
  fan_interlock_bool_state_ = motor_fan_receiver_->interlock_status_.load();

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type asb_ros2_control ::ASBSystemHardware::write(const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  if (end_canopen_thread_.load()) return hardware_interface::return_type::OK;

  // Pass the heartbeat alive bit to the VCU in TPDO1
  auto gcu_alive_bit_prev_value_ = gcu_alive_bit_current_value_.load();
  gcu_alive_bit_current_value_.store((bool)std::round(heartbeat_alive_bit_bool_command_));

  // Check when the alive bit changes, so we can print a warning in the timer if the change period is too low.
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
  if (gcu_alive_bit_prev_value_ != gcu_alive_bit_current_value_.load())
  {
    first_heartbeat_received_.store(true);
    gcu_alive_bit_last_value_change_.store(now);
  }

  bool prev_software_emergency_stop_ = software_emergency_stop_.load();
  software_emergency_stop_.store((bool)std::round(set_software_emergency_stop_bool_command_));
  if (software_emergency_stop_.load() && !prev_software_emergency_stop_)
  {
    RCLCPP_WARN(rclcpp::get_logger("ASBSystemHardware"), "SOFTWARE EMERGENCY STOP ENABLED");
  }
  if (!software_emergency_stop_.load() && prev_software_emergency_stop_)
  {
    RCLCPP_WARN(rclcpp::get_logger("ASBSystemHardware"), "SOFTWARE EMERGENCY STOP DISABLED");
  }
  if (!software_emergency_stop_.load())
  {
    // send the velocity data through TPDO2 of the GCU CANOpen node
    double left_speed_ref_rpm = track_left_velocity_command_ * 60 / (2*M_PI) * 0.8;  // *0.8 because the velocity reference received through the can bus is rescaled by 2400 rpm / 3000 rpm
    double right_speed_ref_rpm = track_right_velocity_command_ * 60 / (2*M_PI) * 0.8;  // *0.8 because the velocity reference received through the can bus is rescaled by 2400 rpm / 3000 rpm
    auto left_speed_ref_rpm_clipped = clip<int16_t>(
            (int16_t)std::round(left_speed_ref_rpm),
            (int16_t)-cfg_.tracks_maximum_velocity_rpm * 0.8,  // *0.8 because the velocity reference received through the can bus is rescaled by 2400 rpm / 3000 rpm
            (int16_t)cfg_.tracks_maximum_velocity_rpm * 0.8);  // *0.8 because the velocity reference received through the can bus is rescaled by 2400 rpm / 3000 rpm
    auto right_speed_ref_rpm_clipped = clip<int16_t>(
            (int16_t)std::round(right_speed_ref_rpm),
            (int16_t)-cfg_.tracks_maximum_velocity_rpm * 0.8,  // *0.8 because the velocity reference received through the can bus is rescaled by 2400 rpm / 3000 rpm
            (int16_t)cfg_.tracks_maximum_velocity_rpm * 0.8);  // *0.8 because the velocity reference received through the can bus is rescaled by 2400 rpm / 3000 rpm
    auto fan_speed_ref_rpm_clipped = clip<int16_t>(
            (int16_t)std::round(fan_speed_ref_rpm_command_),
            0,
            (int16_t)cfg_.fan_maximum_velocity_rpm);
    GCU_->set_TPDO_2(right_speed_ref_rpm_clipped, left_speed_ref_rpm_clipped, fan_speed_ref_rpm_clipped);
  }

  return hardware_interface::return_type::OK;
}

template<typename T> T ASBSystemHardware::clip(T x, T min, T max) {
  return (x < max) ? ((x > min) ? x : min) : max;
}

}  // namespace asb_ros2_control

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  asb_ros2_control::ASBSystemHardware, hardware_interface::SystemInterface)
