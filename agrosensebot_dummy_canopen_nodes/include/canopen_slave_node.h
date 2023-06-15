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
#include <bitset>


// PDO register indices
#define IDX_MOTOR_DRIVE_DATA 0x2110
#define IDX_VCU_IS_ALIVE 0x2111
#define IDX_MOTOR_SPEED_REF 0x2112
#define IDX_GCU_IS_ALIVE 0x2113

// TPDO1
#define SUB_IDX_VCU_is_alive 0x01
#define SUB_IDX_control_mode 0x02

// TPDO2
#define SUB_IDX_MDL_controller_temperature 0x01
#define SUB_IDX_MDL_motor_temperature 0x02
#define SUB_IDX_MDL_motor_RPM 0x03
#define SUB_IDX_MDL_battery_current_display 0x04

// TPDO3
#define SUB_IDX_MDR_controller_temperature 0x05
#define SUB_IDX_MDR_motor_temperature 0x06
#define SUB_IDX_MDR_motor_RPM 0x07
#define SUB_IDX_MDR_battery_current_display 0x08

// TPDO4
#define SUB_IDX_FAN_controller_temperature 0x09
#define SUB_IDX_FAN_motor_temperature 0x0A
#define SUB_IDX_FAN_motor_RPM 0x0B
#define SUB_IDX_FAN_battery_current_display 0x0C

//RPDO1
#define SUB_IDX_GCU_is_alive 0x01

#define BIT_IDX_GCU_is_alive 0
#define BIT_IDX_GCU_is_ready 1

//RPDO2
#define SUB_IDX_RightSpeedRef 0x01
#define SUB_IDX_LeftSpeedRef 0x02

using namespace lely;
using namespace std::chrono_literals;

class ROS2BridgeNode;

class CANOpenSlaveNode : public canopen::BasicSlave {
private:
    ROS2BridgeNode *ros2_bridge_node_;

    void OnWrite(uint16_t idx, uint8_t subidx)

    noexcept override;

public:
    CANOpenSlaveNode(io::TimerBase &timer, io::CanChannelBase &chan, const std::string &dcfTxt,
                     const std::string &dcfBin,
                     ROS2BridgeNode *ros2_bridge_node) :
            BasicSlave(timer, chan, dcfTxt, dcfBin),
            ros2_bridge_node_(ros2_bridge_node) { // TODO pass callback function
    };

    void send_TPDO_1(bool, bool, uint8_t);

    void send_TPDO_2(int16_t, int16_t, int16_t, int16_t);

    void send_TPDO_3(int16_t, int16_t, int16_t, int16_t);

    void send_TPDO_4(int16_t, int16_t, int16_t, int16_t);

};

#endif //TEST_ROS2_CANOPEN_BRIDGE_CANOPEN_SLAVE_NODE_H
