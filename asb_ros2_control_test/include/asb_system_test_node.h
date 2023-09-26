#ifndef ASB_ROS2_CONTROL_TEST__ASB_SYSTEM_TEST_NODE_H_H
#define ASB_ROS2_CONTROL_TEST__ASB_SYSTEM_TEST_NODE_H_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include <chrono>

#define RAW_DATA_STEP_VALUE_temperature 0.1 // 0.1 °C
#define RAW_DATA_STEP_VALUE_current 0.1 // 0.1 A
#define RAW_DATA_STEP_VALUE_torque 1 // 1 Nm
#define RAW_DATA_STEP_VALUE_bdi_percentage 1 // 1 %
#define RAW_DATA_STEP_VALUE_voltage 0.01 // 0.01 V
#define RAW_DATA_STEP_VALUE_rotor_position (1/4096.) // 2^12 revolutions

using std::placeholders::_1;
using namespace std::chrono_literals;

class VCUCANOpenSlaveNode;

class MotorDriveCANOpenSlaveNode;

struct MotorDriveTestState {
  int speed_ref = 0;
  int motor_rpm = 0;
  double rotor_position = 0.0;
};

enum ControlMode {
  STOP = 0,
  RCU = 1,
  GCU = 2,
  WAIT = 3,
};

class ASBSystemTestNode : public rclcpp_lifecycle::LifecycleNode {

  std::string VCU_canopen_node_config_;
  std::string MDL_canopen_node_config_;
  std::string MDR_canopen_node_config_;
  std::string FAN_canopen_node_config_;
  std::string can_interface_name_;
  std::chrono::milliseconds gcu_is_alive_timeout_ = 100ms;

  std::thread VCU_canopen_node_thread_;
  std::thread MDL_canopen_node_thread_;
  std::thread MDR_canopen_node_thread_;
  std::thread FAN_canopen_node_thread_;
  std::shared_ptr<VCUCANOpenSlaveNode> VCU_canopen_slave_node_ = nullptr;
  std::shared_ptr<MotorDriveCANOpenSlaveNode> MDL_canopen_slave_node_ = nullptr;
  std::shared_ptr<MotorDriveCANOpenSlaveNode> MDR_canopen_slave_node_ = nullptr;
  std::shared_ptr<MotorDriveCANOpenSlaveNode> FAN_canopen_slave_node_ = nullptr;

  rclcpp::TimerBase::SharedPtr gcu_is_alive_timer_;
  rclcpp::TimerBase::SharedPtr test_loop_timer_;

  std::atomic<bool> lifecycle_node_active_ = false;

  bool last_VCU_alive_bit_ = false;

  rclcpp::Time last_GCU_message_time_ = rclcpp::Time(0);
  rclcpp::Time last_GCU_alive_bit_change_time_ = rclcpp::Time(0);
  bool last_GCU_alive_bit_ = false;

  rclcpp::Time last_test_loop_time_;
  MotorDriveTestState left_motor_drive_test_state_;
  MotorDriveTestState right_motor_drive_test_state_;
  MotorDriveTestState fan_motor_drive_test_state_;
  bool pump_test_state_ = false;

  void test_loop_timer_ros2_callback();

  void gcu_is_alive_timer_ros2_callback();

  void vcu_alive_test_callback(bool pump_status_bit, bool vcu_safety_status,
                               uint8_t control_mode,
                               uint8_t more_recent_alarm_id_to_confirm, uint8_t more_recent_active_alarm_id);

  void run_VCU_canopen_node();

  void run_MDL_canopen_node();

  void run_MDR_canopen_node();

  void run_FAN_canopen_node();

public:
  explicit ASBSystemTestNode(const std::string &node_name, bool intra_process_comms = false)
          : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(
          intra_process_comms)) {
    this->declare_parameter<std::string>("dummy_VCU_canopen_node_config", "test_slave.eds");
    this->declare_parameter<std::string>("dummy_MDL_canopen_node_config", "test_slave.eds");
    this->declare_parameter<std::string>("dummy_MDR_canopen_node_config", "test_slave.eds");
    this->declare_parameter<std::string>("dummy_FAN_canopen_node_config", "test_slave.eds");
    this->declare_parameter<std::string>("can_interface_name", "vcan0");

    last_test_loop_time_ = this->get_clock()->now();
  };

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_configure(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_activate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_deactivate(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_cleanup(const rclcpp_lifecycle::State &) override;

  rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
  on_shutdown(const rclcpp_lifecycle::State &) override;

  void gcu_alive_canopen_callback(bool GCU_is_alive_bit, bool pump_cmd_bit);

  void speed_ref_canopen_callback(int16_t right_speed_ref, int16_t left_speed_ref, int16_t fan_speed_ref);

  void motor_drive_left_test_callback(
          double controller_temperature, double motor_temperature, int motor_rpm, double battery_current_display,
          double motor_torque, double bdi_percentage, double keyswitch_voltage, int zero_speed_threshold,
          bool interlock_status,
          double rotor_position);

  void motor_drive_right_test_callback(
          double controller_temperature, double motor_temperature, int motor_rpm, double battery_current_display,
          double motor_torque, double bdi_percentage, double keyswitch_voltage, int zero_speed_threshold,
          bool interlock_status,
          double rotor_position);

  void motor_drive_fan_test_callback(
          double controller_temperature, double motor_temperature, int motor_rpm, double battery_current_display,
          double motor_torque, double bdi_percentage, double keyswitch_voltage, int zero_speed_threshold,
          bool interlock_status,
          double rotor_position);

};

#endif //ASB_ROS2_CONTROL_TEST__ASB_SYSTEM_TEST_NODE_H_H
