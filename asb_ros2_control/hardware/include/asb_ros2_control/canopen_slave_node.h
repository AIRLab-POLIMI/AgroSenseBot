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

using namespace lely;
using namespace std::chrono_literals;

class CANOpenSlaveNode : public canopen::BasicSlave {
private:

  void OnWrite(uint16_t idx, uint8_t subidx) noexcept override;

  // VCU comm check variables
  std::chrono::steady_clock::time_point last_VCU_is_alive_bit_change_;
  bool previous_VCU_is_alive_bit_ = false;
  bool VCU_comm_started_ = false;

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
  std::atomic<bool> VCU_pump_status_bit_ = false;
  std::atomic<uint8_t> control_mode_ = 0;
  std::atomic<uint8_t> more_recent_alarm_id_to_confirm_ = false;
  std::atomic<uint8_t> more_recent_active_alarm_id_ = false;

};

#endif //TEST_ROS2_CANOPEN_BRIDGE_CANOPEN_SLAVE_NODE_H
