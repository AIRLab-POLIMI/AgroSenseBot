#ifndef TEST_ROS2_CANOPEN_MOTOR_DRIVE_RECEIVER_H
#define TEST_ROS2_CANOPEN_MOTOR_DRIVE_RECEIVER_H

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


// RPDO1 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_RPDO1 0x3000
#define SUB_IDX_RPDO1_1_controller_temperature 0x01
#define SUB_IDX_RPDO1_2_motor_temperature 0x02
#define SUB_IDX_RPDO1_3_motor_RPM 0x03
#define SUB_IDX_RPDO1_4_battery_current_display 0x04

// RPDO2 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_RPDO2 0x3001
#define SUB_IDX_RPDO2_1_motor_torque 0x01
#define SUB_IDX_RPDO2_2_BDI_percentage 0x02
#define SUB_IDX_RPDO2_3_keyswitch_voltage 0x03
#define SUB_IDX_RPDO2_4_zero_speed_threshold 0x04

// RPDO3 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_RPDO3 0x3002
#define SUB_IDX_RPDO3_1_motor_drive_status 0x01
#define BIT_IDX_interlock_status 1

// RPDO4 in [MDL/MDR/FAN]_receiver.dcf
#define IDX_RPDO4 0x3003
#define SUB_IDX_RPDO4_1_rotor_position 0x01


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

#endif //TEST_ROS2_CANOPEN_MOTOR_DRIVE_RECEIVER_H
