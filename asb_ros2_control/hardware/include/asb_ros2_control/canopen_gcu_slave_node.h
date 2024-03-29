#ifndef ASB_ROS2_CONTROL__CANOPEN_GCU_SLAVE_NODE_H
#define ASB_ROS2_CONTROL__CANOPEN_GCU_SLAVE_NODE_H

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

using namespace lely;
using namespace std::chrono_literals;

class CANOpenGCUNode : public canopen::BasicSlave {
private:

  // TPDO data
  bool new_TPDO_1_ = false;
  bool new_TPDO_2_ = false;
  bool gcu_alive_bit_ = false;
  bool pump_cmd_bit_ = false;
  int16_t right_speed_ref_ = 0;
  int16_t left_speed_ref_ = 0;
  int16_t fan_speed_ref_ = 0;

  // just a variable to throttle terminal output
  std::chrono::steady_clock::time_point last_throttled_output_;

  // VCU comm check variables
  std::chrono::steady_clock::time_point last_VCU_is_alive_bit_change_;
  bool previous_VCU_is_alive_bit_ = false;

  void send_TPDO_1();

  void send_TPDO_2();

  void OnRpdo(int num, ::std::error_code ec, const void* p, ::std::size_t n) noexcept override;

public:

  CANOpenGCUNode(io::TimerBase &timer, io::CanChannelBase &chan, const std::string &dcfTxt, const std::string &dcfBin):
          BasicSlave(timer, chan, dcfTxt, dcfBin) {
  };

  void timer();

  void set_TPDO_1(bool gcu_alive_bit, bool pump_cmd_bit);

  void set_TPDO_2(int16_t right_speed_ref, int16_t left_speed_ref, int16_t fan_speed_ref);


  std::string node_name_;

  // VCU status
  std::atomic<bool> VCU_comm_ok_ = true;
  std::atomic<bool> VCU_comm_started_ = false;
  std::atomic<bool> VCU_safety_status_bit_ = false;
  std::atomic<bool> VCU_pump_status_bit_ = false;
  std::atomic<uint8_t> control_mode_ = 0;
  std::atomic<uint8_t> more_recent_alarm_id_to_confirm_ = false;
  std::atomic<uint8_t> more_recent_active_alarm_id_ = false;

};

#endif //ASB_ROS2_CONTROL__CANOPEN_GCU_SLAVE_NODE_H
