#include "VCU_canopen_slave_node.h"
#include "asb_system_test_node.h"

// PDO register indices

// TPDO1
#define IDX_TPDO1_VCU_STATUS 0x2111
#define SUB_IDX_TPDO1_1_VCU_status 0x01
#define SUB_IDX_TPDO1_2_control_mode 0x02
#define SUB_IDX_TPDO1_3_more_recent_alarm_id_to_confirm 0x03
#define SUB_IDX_TPDO1_4_more_recent_active_alarm_id 0x04

#define BIT_IDX_VCU_is_alive 0
#define BIT_IDX_VCU_safety_status 1
#define BIT_IDX_pump_status 2

//RPDO1
#define IDX_GCU_IS_ALIVE 0x2113
#define SUB_IDX_GCU_state 0x01

#define BIT_IDX_GCU_is_alive 0
#define BIT_IDX_pump_cmd 2

//RPDO2
#define IDX_MOTOR_SPEED_REF 0x2112
#define SUB_IDX_RightSpeedRef 0x01
#define SUB_IDX_LeftSpeedRef 0x02
#define SUB_IDX_FanSpeedRef 0x03

void VCUCANOpenSlaveNode::send_TPDO_1(bool VCU_is_alive_bit, bool VCU_safety_status_bit, bool pump_status_bit,
                                      uint8_t control_mode,
                                      uint8_t more_recent_alarm_id_to_confirm, uint8_t more_recent_active_alarm_id) {
//    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[dummy_VCU] TPDO_1");

    std::bitset<8> VCU_status_bitset;
    VCU_status_bitset[BIT_IDX_VCU_is_alive] = VCU_is_alive_bit;
    VCU_status_bitset[BIT_IDX_VCU_safety_status] = VCU_safety_status_bit;
    VCU_status_bitset[BIT_IDX_pump_status] = pump_status_bit;
    uint8_t VCU_status = VCU_status_bitset.to_ulong();

    (*this)[IDX_TPDO1_VCU_STATUS][SUB_IDX_TPDO1_1_VCU_status] = VCU_status;
    (*this)[IDX_TPDO1_VCU_STATUS][SUB_IDX_TPDO1_2_control_mode] = control_mode;
    (*this)[IDX_TPDO1_VCU_STATUS][SUB_IDX_TPDO1_3_more_recent_alarm_id_to_confirm] = more_recent_alarm_id_to_confirm;
    (*this)[IDX_TPDO1_VCU_STATUS][SUB_IDX_TPDO1_4_more_recent_active_alarm_id] = more_recent_active_alarm_id;

    this->TpdoEvent(1);
}

// This function gets called every time an RPDO is received.
void VCUCANOpenSlaveNode::OnRpdo(int num, ::std::error_code /*ec*/, const void* /*p*/, ::std::size_t /*n*/) noexcept {
//    RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[dummy_VCU] OnRpdo %i", num);

    // RPDO 1 (from GCU node)
    if (num == 1) {
//        RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[dummy_VCU] RPDO_1");
        uint8_t GCU_state = (*this)[IDX_GCU_IS_ALIVE][SUB_IDX_GCU_state];
        bool GCU_is_alive_bit = (GCU_state >> BIT_IDX_GCU_is_alive) & 1;
        bool pump_cmd_bit = (GCU_state >> BIT_IDX_pump_cmd) & 1;
        ros2_bridge_node_->gcu_alive_canopen_callback(GCU_is_alive_bit, pump_cmd_bit);
    }

    // RPDO 2 (from GCU node)
    if (num == 2) {
//        RCLCPP_INFO(ros2_bridge_node_->get_logger(), "[dummy_VCU] RPDO_2");
        int16_t right_speed_ref = (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_RightSpeedRef];
        int16_t left_speed_ref = (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_LeftSpeedRef];
        int16_t fan_speed_ref = (*this)[IDX_MOTOR_SPEED_REF][SUB_IDX_FanSpeedRef];
        ros2_bridge_node_->speed_ref_canopen_callback(right_speed_ref, left_speed_ref, fan_speed_ref);
    }
}
