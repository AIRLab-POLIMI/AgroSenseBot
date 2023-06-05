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

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/lifecycle_node.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/u_int32.hpp"

// RPDO4
#define IDX_FAN_controller_temperature 0x2110
#define SUB_IDX_FAN_controller_temperature 0x09

#define IDX_FAN_motor_temperature 0x2110
#define SUB_IDX_FAN_motor_temperature 0x0A

#define IDX_FAN_motor_RPM 0x2110
#define SUB_IDX_FAN_motor_RPM 0x0B

#define IDX_FAN_battery_current_display 0x2110
#define SUB_IDX_FAN_battery_current_display 0x0C

//TPDO1
#define IDX_FAN_battery_current_display 0x2110
#define SUB_IDX_FAN_battery_current_display 0x0C


using std::placeholders::_1;

using namespace lely;
using namespace std::chrono_literals;

class ROS2BridgeNode;

class CANOpenSlaveNode : public canopen::BasicSlave {
private:
    ROS2BridgeNode *ros2_bridge_node_;

public:
    CANOpenSlaveNode(io::TimerBase &timer, io::CanChannelBase &chan, const std::string &dcfTxt, const std::string &dcfBin,
                     uint8_t id, ROS2BridgeNode *ros2_bridge_node) :
            BasicSlave(timer, chan, dcfTxt, dcfBin, id), ros2_bridge_node_(ros2_bridge_node) {
    };
    CANOpenSlaveNode(io::TimerBase &timer, io::CanChannelBase &chan, const std::string &dcfTxt, const std::string &dcfBin,
                     ROS2BridgeNode *ros2_bridge_node) :
            BasicSlave(timer, chan, dcfTxt, dcfBin), ros2_bridge_node_(ros2_bridge_node) {
    };

    void send_TPDO(uint32_t data);

protected:
    void OnWrite(uint16_t idx, uint8_t subidx) noexcept override;
};

#endif //TEST_ROS2_CANOPEN_BRIDGE_CANOPEN_SLAVE_NODE_H
