#include <iostream>
#include <bitset>
#include "asb_ros2_control/canopen_gcu_slave_node.h"
#include "asb_ros2_control/utils.hpp"

// RPDO1 in GCU.dcf
#define IDX_RPDO1 0x3000
#define SUB_IDX_RPDO1_1_VCU_state 0x01
#define SUB_IDX_RPDO1_2_control_mode 0x02
#define SUB_IDX_RPDO1_3_more_recent_alarm_id_to_confirm 0x03
#define SUB_IDX_RPDO1_4_more_recent_active_alarm_id 0x04

#define BIT_IDX_VCU_is_alive 0
#define BIT_IDX_VCU_safety_status 1
#define BIT_IDX_VCU_pump_status 2

// TPDO1 in GCU.dcf
#define IDX_TPDO1 0x3800
#define SUB_IDX_TPDO1_1_GCU_state 0x01
#define SUB_IDX_TPDO1_2 0x02

#define BIT_IDX_GCU_is_alive 0
#define BIT_IDX_VCU_pump_cmd 2

// TPDO2 in GCU.dcf
#define IDX_TPDO2 0x3801
#define SUB_IDX_TPDO2_1_right_speed_ref 0x01
#define SUB_IDX_TPDO2_2_left_speed_ref 0x02
#define SUB_IDX_TPDO2_3_fan_speed_ref 0x03

void CANOpenGCUNode::timer() {
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  send_TPDO_1();
  send_TPDO_2();

  if(VCU_comm_started_) {
    VCU_comm_ok_.store((now - last_VCU_is_alive_bit_change_ < 200ms));  // TODO make parameter
    if (!VCU_comm_ok_.load()) {
      std::cerr << "[" << node_name_ << "]" << "[CANOpenGCUNode::timer] VCU COMM TIMEOUT ("
                << chrono_duration_to_s(last_VCU_is_alive_bit_change_, now)  << "s)" << std::endl;
    }
  } else {
    std::cout << "[" << node_name_ << "]" << "[CANOpenGCUNode::timer] VCU COMM NOT STARTED YET" << std::endl;
  }
}

void CANOpenGCUNode::set_TPDO_1(bool gcu_alive_bit, bool pump_cmd_bit) {
  gcu_alive_bit_ = gcu_alive_bit;
  pump_cmd_bit_ = pump_cmd_bit;
  new_TPDO_1_ = true;
}

void CANOpenGCUNode::send_TPDO_1() {
  if(!new_TPDO_1_) return;
  new_TPDO_1_ = false;
//  std::cout << "[" << node_name_ << "]" << " TPDO 1 " << std::endl;
  std::bitset<8> gcu_state_data_bitset;
  gcu_state_data_bitset[BIT_IDX_GCU_is_alive] = gcu_alive_bit_;
  gcu_state_data_bitset[BIT_IDX_VCU_pump_cmd] = pump_cmd_bit_;
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_1_GCU_state] = (uint8_t) gcu_state_data_bitset.to_ulong();
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_2] = (uint8_t)0;  // TPDO1_2 is not used
  this->TpdoEvent(1);
}

void CANOpenGCUNode::set_TPDO_2(int16_t right_speed_ref, int16_t left_speed_ref, int16_t fan_speed_ref) {
  right_speed_ref_ = right_speed_ref;
  left_speed_ref_ = left_speed_ref;
  fan_speed_ref_ = fan_speed_ref;
  new_TPDO_2_ = true;
}

void CANOpenGCUNode::send_TPDO_2() {
  if(!new_TPDO_2_) return;
  new_TPDO_2_ = false;
//    std::cout << "[" << node_name_ << "]" << " TPDO 2 " << std::endl;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_1_right_speed_ref] = right_speed_ref_;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_2_left_speed_ref] = left_speed_ref_;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_3_fan_speed_ref] = fan_speed_ref_;
  this->TpdoEvent(2);
}

// This function gets called every time an RPDO is received.
void CANOpenGCUNode::OnRpdo(int num, ::std::error_code /*ec*/, const void* /*p*/, ::std::size_t /*n*/) noexcept {
//  std::cout << "[" << node_name_ << "]" << " OnWrite " << std::hex << (int)idx << " " << std::hex << (int)subidx << std::endl;

  // RPDO 1
  if (num == 1) {
//    std::cout << "[" << node_name_ << "]" << " RPDO 1 " << "COB-ID: " << (int)(uint32_t)(*this)[0x1400][0x01] << std::endl;

    uint8_t VCU_state = (*this)[IDX_RPDO1][SUB_IDX_RPDO1_1_VCU_state];
    bool VCU_is_alive_bit = (VCU_state >> BIT_IDX_VCU_is_alive) & 1;
    VCU_safety_status_bit_.store((VCU_state >> BIT_IDX_VCU_safety_status) & 1);
    VCU_pump_status_bit_.store((VCU_state >> BIT_IDX_VCU_pump_status) & 1);

    if (!VCU_comm_started_) {
      last_VCU_is_alive_bit_change_ = std::chrono::steady_clock::now();
      previous_VCU_is_alive_bit_ = VCU_is_alive_bit;
    } else {
      if (VCU_is_alive_bit != previous_VCU_is_alive_bit_) {
        last_VCU_is_alive_bit_change_ = std::chrono::steady_clock::now();
      }
      previous_VCU_is_alive_bit_ = VCU_is_alive_bit;
    }

    control_mode_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_2_control_mode]);

    more_recent_alarm_id_to_confirm_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_3_more_recent_alarm_id_to_confirm]);
    more_recent_active_alarm_id_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_4_more_recent_active_alarm_id]);

    VCU_comm_started_ = true;
  }

}
