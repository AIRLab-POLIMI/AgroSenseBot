#ifndef TEST_ROS2_CANOPEN_BRIDGE_CANOPEN_SLAVE_NODE_H
#define TEST_ROS2_CANOPEN_BRIDGE_CANOPEN_SLAVE_NODE_H

#include <lely/ev/loop.hpp>
#include <lely/io2/linux/can.hpp>
#include <lely/io2/posix/poll.hpp>
#include <lely/io2/sys/io.hpp>
#include <lely/io2/sys/sigset.hpp>
#include <lely/io2/sys/timer.hpp>
#include <lely/coapp/slave.hpp>

#include <future>
#include <atomic>
#include <mutex>
#include <thread>
#include <chrono>

// PDO register indices
#define IDX_MOTOR_DRIVE_DATA 0x2110
#define IDX_VCU_IS_ALIVE 0x2111
#define IDX_MOTOR_SPEED_REF 0x2112
#define IDX_GCU_IS_ALIVE 0x2113

// RPDO1
#define SUB_IDX_VCU_is_alive 0x01
#define SUB_IDX_control_mode 0x02

#define BIT_IDX_VCU_is_alive 0
#define BIT_IDX_VCU_safety_status 1

// RPDO2
#define SUB_IDX_MDL_controller_temperature 0x01
#define SUB_IDX_MDL_motor_temperature 0x02
#define SUB_IDX_MDL_motor_RPM 0x03
#define SUB_IDX_MDL_battery_current_display 0x04

// RPDO3
#define SUB_IDX_MDR_controller_temperature 0x05
#define SUB_IDX_MDR_motor_temperature 0x06
#define SUB_IDX_MDR_motor_RPM 0x07
#define SUB_IDX_MDR_battery_current_display 0x08

// RPDO4
#define SUB_IDX_FAN_controller_temperature 0x09
#define SUB_IDX_FAN_motor_temperature 0x0A
#define SUB_IDX_FAN_motor_RPM 0x0B
#define SUB_IDX_FAN_battery_current_display 0x0C

//TPDO1
#define SUB_IDX_GCU_is_alive 0x01

//TPDO2
#define SUB_IDX_RightSpeedRef 0x01
#define SUB_IDX_LeftSpeedRef 0x02

using namespace lely;
using namespace std::chrono_literals;

class CANOpenSlaveNode : public canopen::BasicSlave {
private:

  void OnWrite(uint16_t idx, uint8_t subidx) noexcept override;

  // VCU comm check variables
  std::chrono::steady_clock::time_point last_VCU_is_alive_bit_change_;
  bool previous_VCU_is_alive_bit_ = false;
  bool VCU_comm_started_ = false;

  //  left and right motor position hack (TODO)
  std::chrono::steady_clock::time_point last_RPM_data_left_;
  std::chrono::steady_clock::time_point last_RPM_data_right_;
  std::chrono::duration<double> since_last_RPM_data_left_;
  std::chrono::duration<double> since_last_RPM_data_right_;
  double rotor_position_left_local_ = 0;
  double rotor_position_right_local_ = 0;
  bool first_data_left_ = true;
  bool first_data_right_ = true;

public:

  CANOpenSlaveNode(io::TimerBase &timer, io::CanChannelBase &chan, const std::string &dcfTxt, const std::string &dcfBin):
          BasicSlave(timer, chan, dcfTxt, dcfBin) {
  };

  void timer();

  void send_TPDO_1(uint8_t);

  void send_TPDO_2(int16_t, int16_t);

  std::string node_name_ = "";

  // VCU status
  std::atomic<bool> VCU_comm_ok_ = true;
  std::atomic<bool> VCU_safety_status_bit_ = false;
  std::atomic<uint8_t> control_mode_ = 0;

  // left motor status
  std::atomic<int16_t> controller_temperature_left_ = 0;
  std::atomic<int16_t> motor_temperature_left_ = 0;
  std::atomic<int16_t> motor_RPM_left_ = 0;
  std::atomic<int16_t> battery_current_display_left_ = 0;

  // right motor status
  std::atomic<int16_t> controller_temperature_right_ = 0;
  std::atomic<int16_t> motor_temperature_right_ = 0;
  std::atomic<int16_t> motor_RPM_right_ = 0;
  std::atomic<int16_t> battery_current_display_right_ = 0;

  // fan motor status
  std::atomic<int16_t> controller_temperature_fan_ = 0;
  std::atomic<int16_t> motor_temperature_fan_ = 0;
  std::atomic<int16_t> motor_RPM_fan_ = 0;
  std::atomic<int16_t> battery_current_display_fan_ = 0;

  // left and right motor position hack (TODO)
  std::atomic<double> rotor_position_left_ = 0;
  std::atomic<double> rotor_position_right_ = 0;

};

#endif //TEST_ROS2_CANOPEN_BRIDGE_CANOPEN_SLAVE_NODE_H
