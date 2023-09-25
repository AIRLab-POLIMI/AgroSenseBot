#ifndef TEST_ROS2_CANOPEN_BRIDGE_MOTOR_DRIVE_CANOPEN_SLAVE_NODE_H
#define TEST_ROS2_CANOPEN_BRIDGE_MOTOR_DRIVE_CANOPEN_SLAVE_NODE_H


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

class ROS2BridgeNode;

class MotorDriveCANOpenSlaveNode : public canopen::BasicSlave {
private:
    ROS2BridgeNode *ros2_bridge_node_;

public:
    MotorDriveCANOpenSlaveNode(io::TimerBase &timer, io::CanChannelBase &chan, const std::string &dcfTxt,
                     const std::string &dcfBin, ROS2BridgeNode *ros2_bridge_node) :
            BasicSlave(timer, chan, dcfTxt, dcfBin),
            ros2_bridge_node_(ros2_bridge_node) { // TODO pass callback function
    };

    void send_TPDO_1(int16_t, int16_t, int16_t, int16_t);

};

#endif //TEST_ROS2_CANOPEN_BRIDGE_MOTOR_DRIVE_CANOPEN_SLAVE_NODE_H
