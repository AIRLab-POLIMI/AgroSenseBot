#ifndef ASB_SIM__VCU_CANOPEN_SLAVE_NODE_H
#define ASB_SIM__VCU_CANOPEN_SLAVE_NODE_H


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
#include <bitset>

using namespace lely;
using namespace std::chrono_literals;

class ASBSystemTestNode;

class VCUCANOpenSlaveNode : public canopen::BasicSlave {
private:
  ASBSystemTestNode *ros2_bridge_node_;

  // TPDO data
  bool new_TPDO_1_ = false;
  bool VCU_is_alive_bit_ = false;
  bool VCU_safety_status_bit_ = false;
  bool pump_status_bit_ = false;
  uint8_t control_mode_ = 0;
  uint8_t more_recent_alarm_id_to_confirm_ = 0;
  uint8_t more_recent_active_alarm_id_ = 0;

  void send_TPDO_1();

  void OnRpdo(int num, ::std::error_code ec, const void *p, ::std::size_t n) noexcept override;

public:
  VCUCANOpenSlaveNode(io::TimerBase &timer, io::CanChannelBase &chan, const std::string &dcfTxt,
                      const std::string &dcfBin,
                      ASBSystemTestNode *ros2_bridge_node) :
          BasicSlave(timer, chan, dcfTxt, dcfBin),
          ros2_bridge_node_(ros2_bridge_node) { // TODO pass callback function
  };

  void timer();

  void set_TPDO_1(bool VCU_is_alive_bit, bool VCU_safety_status_bit, bool pump_status_bit,
                  uint8_t control_mode,
                  uint8_t more_recent_alarm_id_to_confirm, uint8_t more_recent_active_alarm_id);

};

#endif //ASB_SIM__VCU_CANOPEN_SLAVE_NODE_H
