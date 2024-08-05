#ifndef ASB_SIM__ASB_SYSTEM_TEST_NODE_H_H
#define ASB_SIM__ASB_SYSTEM_TEST_NODE_H_H

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "asb_msgs/msg/sim_state_cmd.hpp"
#include "asb_msgs/msg/sim_state.hpp"
#include "std_msgs/msg/int16.hpp"

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

class MotorDriveTestState {
public:
  int speed_ref = 0;
  int motor_rpm = 0;
  int32_t rotor_position_raw = 0;
//  int32_t rotor_position_raw = 2147483648 - 8 * 4096;

  void apply_motor_speed_ref(rclcpp::Duration time_delta) {
    motor_rpm = speed_ref;
    apply_motor_speed(time_delta);
  }

  void apply_motor_speed(rclcpp::Duration time_delta) {
    rotor_position_raw += (int32_t)((motor_rpm / 60.) * time_delta.seconds() / RAW_DATA_STEP_VALUE_rotor_position);
  }

  double rotor_position() const {
    return (double)rotor_position_raw * RAW_DATA_STEP_VALUE_rotor_position;
  }

};

enum ControlMode {
  STOP = 0,
  RCU = 1,
  GCU = 2,
  WAIT = 3,
};

class ASBSystemTestNode : public rclcpp_lifecycle::LifecycleNode {

  bool print_debug_ = false;
  bool start_in_control_mode_GCU_ = false;
  bool use_simulator_ = false;
  std::string VCU_canopen_node_config_;
  std::string MDL_canopen_node_config_;
  std::string MDR_canopen_node_config_;
  std::string FAN_canopen_node_config_;
  std::string can_interface_name_;
  std::chrono::milliseconds gcu_is_alive_timeout_ = 200ms;

  std::thread canopen_nodes_thread_;
  std::shared_ptr<VCUCANOpenSlaveNode> VCU_canopen_slave_node_ = nullptr;
  std::shared_ptr<MotorDriveCANOpenSlaveNode> MDL_canopen_slave_node_ = nullptr;
  std::shared_ptr<MotorDriveCANOpenSlaveNode> MDR_canopen_slave_node_ = nullptr;
  std::shared_ptr<MotorDriveCANOpenSlaveNode> FAN_canopen_slave_node_ = nullptr;

  rclcpp::TimerBase::SharedPtr gcu_is_alive_timer_;
  rclcpp::TimerBase::SharedPtr test_loop_timer_;

  rclcpp::Subscription<std_msgs::msg::Int16>::SharedPtr control_mode_subscriber_;
  rclcpp::Subscription<asb_msgs::msg::SimState>::SharedPtr sim_state_subscriber_;
  rclcpp::Publisher<asb_msgs::msg::SimStateCmd>::SharedPtr sim_state_cmd_publisher_;

  std::atomic<bool> lifecycle_node_active_ = false;

  bool VCU_alive_bit_ = false;

  bool comm_started_ = false;
  rclcpp::Time last_GCU_message_time_ = rclcpp::Time(0);
  rclcpp::Time last_GCU_alive_bit_change_time_ = rclcpp::Time(0);
  bool last_GCU_alive_bit_ = false;

  rclcpp::Time last_test_loop_time_;
  MotorDriveTestState left_motor_drive_test_state_;
  MotorDriveTestState right_motor_drive_test_state_;
  MotorDriveTestState fan_motor_drive_test_state_;
  ControlMode control_mode_test_state_ = ControlMode::GCU;
  bool pump_test_state_ = false;

  void control_mode_ros2_callback(const std_msgs::msg::Int16::SharedPtr msg);

  void sim_ros2_callback(const asb_msgs::msg::SimState::SharedPtr msg);

  void test_loop_timer_ros2_callback();

  void gcu_is_alive_timer_ros2_callback();

  void vcu_alive_test_callback(bool pump_status_bit, bool vcu_safety_status,
                               uint8_t control_mode,
                               uint8_t more_recent_alarm_id_to_confirm, uint8_t more_recent_active_alarm_id);

  void run_canopen_nodes();


public:
  explicit ASBSystemTestNode(const std::string &node_name, bool intra_process_comms = false)
          : rclcpp_lifecycle::LifecycleNode(node_name, rclcpp::NodeOptions().use_intra_process_comms(
          intra_process_comms)) {
    this->declare_parameter<bool>("print_debug", false);
    this->declare_parameter<bool>("start_in_control_mode_GCU", false);
    this->declare_parameter<bool>("use_simulator", false);
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
          int32_t rotor_position_raw);

  void motor_drive_right_test_callback(
          double controller_temperature, double motor_temperature, int motor_rpm, double battery_current_display,
          double motor_torque, double bdi_percentage, double keyswitch_voltage, int zero_speed_threshold,
          bool interlock_status,
          int32_t rotor_position_raw);

  void motor_drive_fan_test_callback(
          double controller_temperature, double motor_temperature, int motor_rpm, double battery_current_display,
          double motor_torque, double bdi_percentage, double keyswitch_voltage, int zero_speed_threshold,
          bool interlock_status,
          int32_t rotor_position_raw);

};

#endif //ASB_SIM__ASB_SYSTEM_TEST_NODE_H_H
