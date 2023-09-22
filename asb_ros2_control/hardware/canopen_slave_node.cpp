#include <iostream>
#include "asb_ros2_control/canopen_slave_node.h"
#include "asb_ros2_control/utils.hpp"

void CANOpenSlaveNode::timer() {
  std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();

  if(VCU_comm_started_)
  {
    VCU_comm_ok_.store((now - last_VCU_is_alive_bit_change_ < 200ms));  // TODO make parameter
    if (!VCU_comm_ok_.load()) {
      std::cerr << "[" << node_name_ << "]" << "[CANOpenSlaveNode::timer] VCU COMM TIMEOUT ("
                << chrono_duration_to_s(last_VCU_is_alive_bit_change_, now)  << "s)" << std::endl;
    }
  } else {
    std::cout << "[" << node_name_ << "]" << "[CANOpenSlaveNode::timer] VCU COMM NOT STARTED YET" << std::endl;
  }
}

void CANOpenSlaveNode::send_TPDO_1(uint8_t data) {
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_1_GCU_state] = data;
  (*this)[IDX_TPDO1][SUB_IDX_TPDO1_2] = (uint8_t)0;  // TPDO1_2 is not used
  this->TpdoEvent(1);
}

void CANOpenSlaveNode::send_TPDO_2(int16_t right_speed_ref, int16_t left_speed_ref) {
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_1_right_speed_ref] = right_speed_ref;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_2_left_speed_ref] = left_speed_ref;
  (*this)[IDX_TPDO2][SUB_IDX_TPDO2_3_fan_speed_ref] = (int16_t)0;  // TODO
  this->TpdoEvent(2);
}

// This function gets called every time a value is written to the local object dictionary by an SDO or RPDO.
void CANOpenSlaveNode::OnWrite(uint16_t idx, uint8_t subidx) noexcept {

  // RPDO 1
  if (idx == IDX_RPDO1 && subidx == SUB_IDX_RPDO1_2_control_mode) {  // TODO SUB_IDX_RPDO1_4_more_recent_active_alarm_id
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

    // TODO SUB_IDX_RPDO1_3_more_recent_alarm_id_to_confirm
    // TODO SUB_IDX_RPDO1_4_more_recent_active_alarm_id

    VCU_comm_started_ = true;
  }

}
