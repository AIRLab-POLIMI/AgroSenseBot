#ifndef ASB_SIM__MOTOR_DRIVE_CANOPEN_SLAVE_NODE_H
#define ASB_SIM__MOTOR_DRIVE_CANOPEN_SLAVE_NODE_H


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

class MotorDriveCANOpenSlaveNode : public canopen::BasicSlave {
private:

  // TPDO data
  bool new_TPDO_1_ = false;
  bool new_TPDO_2_ = false;
  bool new_TPDO_3_ = false;
  bool new_TPDO_4_ = false;
  int16_t controller_temperature_ = 0;
  int16_t motor_temperature_ = 0;
  int16_t motor_rpm_ = 0;
  int16_t battery_current_display_ = 0;
  int16_t motor_torque_ = 0;
  int16_t bdi_percentage_ = 0;
  int16_t keyswitch_voltage_ = 0;
  int16_t zero_speed_threshold_ = 0;
  bool interlock_status_bit_ = false;
  int32_t rotor_position_ = 0;

  void send_TPDO_1();

  void send_TPDO_2();

  void send_TPDO_3();

  void send_TPDO_4();

public:
  MotorDriveCANOpenSlaveNode(io::TimerBase &timer, io::CanChannelBase &chan, const std::string &dcfTxt,
                             const std::string &dcfBin) :
          BasicSlave(timer, chan, dcfTxt, dcfBin) {
  };

  std::string node_name_;

  void timer();

  void set_TPDO_1(int16_t controller_temperature, int16_t motor_temperature,
                  int16_t motor_rpm, int16_t battery_current_display);


  void set_TPDO_2(int16_t motor_torque, int16_t bdi_percentage,
                  int16_t keyswitch_voltage, int16_t zero_speed_threshold);

  void set_TPDO_3(bool interlock_status_bit);


  void set_TPDO_4(int32_t rotor_position);


};

#endif //ASB_SIM__MOTOR_DRIVE_CANOPEN_SLAVE_NODE_H
