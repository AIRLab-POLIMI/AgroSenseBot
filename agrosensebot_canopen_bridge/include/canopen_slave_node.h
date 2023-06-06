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

// PDO register indices
#define IDX_MOTOR_DRIVE_DATA 0x2110
#define IDX_GCU_IS_ALIVE 0x2111
#define IDX_MOTOR_SPEED_REF 0x2112

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

class ROS2BridgeNode;

class CANOpenSlaveNode : public canopen::BasicSlave {
private:
    ROS2BridgeNode *ros2_bridge_node_;

    void OnWrite(uint16_t idx, uint8_t subidx) noexcept override;

public:
    CANOpenSlaveNode(io::TimerBase &timer, io::CanChannelBase &chan, const std::string &dcfTxt, const std::string &dcfBin,
                     ROS2BridgeNode *ros2_bridge_node) :
            BasicSlave(timer, chan, dcfTxt, dcfBin), ros2_bridge_node_(ros2_bridge_node) { // TODO pass callback function
    };

    void send_TPDO_1(uint8_t);
    void send_TPDO_2(int16_t, int16_t);

};

#endif //TEST_ROS2_CANOPEN_BRIDGE_CANOPEN_SLAVE_NODE_H
