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

    VCU_comm_started_ = true;

    if (VCU_is_alive_bit != previous_VCU_is_alive_bit_)
    {
      last_VCU_is_alive_bit_change_ = std::chrono::steady_clock::now();
    }
    previous_VCU_is_alive_bit_ = VCU_is_alive_bit;

    control_mode_.store((*this)[IDX_RPDO1][SUB_IDX_RPDO1_2_control_mode]);

    // TODO SUB_IDX_RPDO1_3_more_recent_alarm_id_to_confirm
    // TODO SUB_IDX_RPDO1_4_more_recent_active_alarm_id
  }


//  // RPDO 1 (from MDL node)
//  if (idx == IDX_MOTOR_DRIVE_DATA && subidx == SUB_IDX_MDL_battery_current_display) {
//    std::cout << std::endl << "[" << node_name_ << "]" << " RPDO 2" << std::endl;
//    controller_temperature_left_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_controller_temperature]);
//    motor_temperature_left_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_motor_temperature]);
//    int16_t motor_RPM_left_tmp = (*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_motor_RPM];
//    motor_RPM_left_.store(motor_RPM_left_tmp);
//    battery_current_display_left_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDL_battery_current_display]);
//
//    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
//    since_last_RPM_data_left_ = now - last_RPM_data_left_;
//    last_RPM_data_left_ = std::chrono::steady_clock::now();
//
//    if(first_data_left_)
//    {
//      first_data_left_ = false;
//    } else {
//      double since_last_RPM_data_left_seconds = 1e-6 * (int)std::chrono::duration_cast<std::chrono::microseconds>(since_last_RPM_data_left_).count();
//      rotor_position_left_local_ += motor_RPM_left_tmp / 60.0 * since_last_RPM_data_left_seconds;
//      rotor_position_left_.store(rotor_position_left_local_);
//    }
//  }

  // RPDO 3 (from MDR node)
//  if (idx == IDX_MOTOR_DRIVE_DATA && subidx == SUB_IDX_MDR_battery_current_display) {
//    std::cout << std::endl << "[" << node_name_ << "]" << " RPDO 3" << std::endl;
//    controller_temperature_right_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_controller_temperature]);
//    motor_temperature_right_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_motor_temperature]);
//    int16_t motor_RPM_right_tmp =(*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_motor_RPM];
//    motor_RPM_right_.store(motor_RPM_right_tmp);
//    battery_current_display_right_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_MDR_battery_current_display]);
//
//    std::chrono::steady_clock::time_point now = std::chrono::steady_clock::now();
//    since_last_RPM_data_right_ = now - last_RPM_data_right_;
//    last_RPM_data_right_ = std::chrono::steady_clock::now();
//
//    if(first_data_right_)
//    {
//      first_data_right_ = false;
//    } else {
//      double since_last_RPM_data_right_seconds = 1e-6 * (int)std::chrono::duration_cast<std::chrono::microseconds>(since_last_RPM_data_right_).count();
//      rotor_position_right_local_ += motor_RPM_right_tmp / 60.0 * since_last_RPM_data_right_seconds;
//      rotor_position_right_.store(rotor_position_right_local_);
//    }
//  }
//
//  // RPDO 4 (from FAN node)
//  if (idx == IDX_MOTOR_DRIVE_DATA && subidx == SUB_IDX_FAN_battery_current_display) {
//    std::cout << std::endl << "[" << node_name_ << "]" << " RPDO 4" << std::endl;
//    controller_temperature_fan_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_controller_temperature]);
//    motor_temperature_fan_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_motor_temperature]);
//    motor_RPM_fan_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_motor_RPM]);
//    battery_current_display_fan_.store((*this)[IDX_MOTOR_DRIVE_DATA][SUB_IDX_FAN_battery_current_display]);
//  }

}
