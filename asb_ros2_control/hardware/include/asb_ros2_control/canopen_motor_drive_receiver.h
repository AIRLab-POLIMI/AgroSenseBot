#ifndef ASB_ROS2_CONTROL__CANOPEN_MOTOR_DRIVE_RECEIVER_H
#define ASB_ROS2_CONTROL__CANOPEN_MOTOR_DRIVE_RECEIVER_H

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

class CANOpenMotorDriveReceiverNode : public canopen::BasicSlave {
private:

  void OnWrite(uint16_t idx, uint8_t subidx) noexcept override;

public:

  CANOpenMotorDriveReceiverNode(io::TimerBase &timer, io::CanChannelBase &chan, const std::string &dcfTxt, const std::string &dcfBin):
          BasicSlave(timer, chan, dcfTxt, dcfBin) {
  };

  std::string node_name_;

  // comm variables
  std::atomic<std::chrono::steady_clock::time_point> last_data_received_time_;

  // RPDO 1
  std::atomic<int16_t> controller_temperature_ = 0;
  std::atomic<int16_t> motor_temperature_ = 0;
  std::atomic<int16_t> motor_RPM_ = 0;
  std::atomic<int16_t> battery_current_display_ = 0;

  // RPDO 2
  std::atomic<int16_t> motor_torque_ = 0;
  std::atomic<int16_t> BDI_percentage_ = 0;
  std::atomic<int16_t> keyswitch_voltage_ = 0;
  std::atomic<int16_t> zero_speed_threshold_ = 0;

  // RPDO 3
  std::atomic<bool> interlock_status_ = false;

  // RPDO 4
  std::atomic<int32_t> rotor_position_ = 0;

};

#endif //ASB_ROS2_CONTROL__CANOPEN_MOTOR_DRIVE_RECEIVER_H
