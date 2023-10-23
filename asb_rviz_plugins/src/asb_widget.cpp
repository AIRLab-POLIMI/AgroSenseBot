#include <asb_rviz_plugins/asb_widget.h>
#include <qwt/qwt_scale_widget.h>
#include "qwt/qwt_color_map.h"

namespace asb_rviz_plugins
{
  ASBWidget::ASBWidget(QWidget *parent): QWidget(parent), ui_(new Ui::ASBWidgetUI) {
    ui_->setupUi(this);

    left_motor_velocity_ = new ASBThermo;
    left_motor_velocity_->setUpperBound(2900);
    left_motor_velocity_->setLowerBound(-2900);
    left_motor_velocity_->setScaleMaxMajor(0);
    left_motor_velocity_->setScaleMaxMinor(0);
    left_motor_velocity_->setScaleStepSize(2400);
    left_motor_velocity_->setOriginMode(ASBThermo::OriginMode::OriginCustom);
    left_motor_velocity_->setOrigin(0);
    left_motor_velocity_->setAlarmEnabled(true);
    left_motor_velocity_->setAlarmLevel(2400);
    left_motor_velocity_->setSpacing(5);

    ui_->left_motor_velocity_layout->addWidget(left_motor_velocity_);
    left_motor_velocity_->setValue(2200);
  }

  void ASBWidget::control_system_state_callback(const asb_msgs::msg::ControlSystemState::SharedPtr control_system_state) const {

//    System status
    ui_->sw_emergency_stop_disp->setText(QString(control_system_state->software_emergency_stop ? "ENABLED":"OK"));
    ui_->sw_emergency_stop_disp->setStyleSheet(control_system_state->software_emergency_stop ? "QLabel {background-color: yellow;}":"QLabel {}");

    if(control_system_state->vcu_comm_ok) {
      ui_->vcu_comm_disp->setText(QString("OK"));
      ui_->vcu_comm_disp->setStyleSheet("QLabel {}");
      ui_->vcu_safety_disp->setText(QString(control_system_state->vcu_safety_status ? "LOCK":"OK"));
      ui_->vcu_safety_disp->setStyleSheet(control_system_state->vcu_safety_status ? "QLabel {background-color: yellow;}":"QLabel {}");
      ui_->control_mode_disp->setText(control_mode_string[control_system_state->control_mode]);
      ui_->pump_disp->setText(QString(control_system_state->pump_state ? "ON":"OFF"));
      control_system_state->more_recent_active_alarm_id ?
        ui_->errors_disp->setText(QString("CODE %1").arg(control_system_state->more_recent_active_alarm_id)) :
        ui_->errors_disp->setText(QString("NONE"));
      control_system_state->more_recent_active_alarm_id ?
        ui_->errors_disp->setStyleSheet("QLabel {background-color: yellow;}") :
        ui_->errors_disp->setStyleSheet("QLabel {}");
    } else {
      ui_->vcu_comm_disp->setText(QString("DOWN"));
      ui_->vcu_comm_disp->setStyleSheet("QLabel {background-color: yellow;}");
      ui_->vcu_safety_disp->setText(QString("UNKNOWN"));
      ui_->vcu_safety_disp->setStyleSheet("QLabel {}");
      ui_->control_mode_disp->setText(QString("UNKNOWN"));
      ui_->pump_disp->setText(QString("UNKNOWN"));
      ui_->errors_disp->setText(QString("UNKNOWN"));
      ui_->errors_disp->setStyleSheet("QLabel {}");
    }

//    Battery
    ui_->battery_soc->setValue((double)control_system_state->left_motor_bdi_percentage);
    ui_->battery_soc_disp->setText(QString("%1 %").arg(
            control_system_state->left_motor_bdi_percentage));

    ui_->battery_voltage->setValue(control_system_state->left_motor_keyswitch_voltage);
    ui_->battery_voltage_disp->setText(QString("%1 V").arg(
            control_system_state->left_motor_keyswitch_voltage, 0, 'f', 2));

//    Fan Motor
    ui_->fan_motor_current->setValue(control_system_state->fan_motor_battery_current);
    ui_->fan_motor_current_disp->setText(QString("%1 A").arg(
            control_system_state->fan_motor_battery_current, 0, 'f', 1));

    ui_->fan_motor_torque->setValue(control_system_state->fan_motor_torque);
    ui_->fan_motor_torque_disp->setText(QString("%1 Nm").arg(
            control_system_state->fan_motor_torque, 0, 'f', 0));

    ui_->fan_motor_temperature->setValue(control_system_state->fan_motor_temperature);
    ui_->fan_motor_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->fan_motor_temperature, 0, 'f', 1));

    ui_->fan_controller_temperature->setValue(control_system_state->fan_motor_controller_temperature);
    ui_->fan_controller_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->fan_motor_controller_temperature, 0, 'f', 1));

    ui_->fan_motor_velocity->setValue(control_system_state->fan_motor_velocity_rpm);
    ui_->fan_motor_velocity_disp->setText(QString("%1").arg(
            control_system_state->fan_motor_velocity_rpm, 0, 'f', 0));

//    Left Motor
    ui_->left_motor_current->setValue(control_system_state->left_motor_battery_current);
    ui_->left_motor_current_disp->setText(QString("%1 A").arg(
            control_system_state->left_motor_battery_current, 0, 'f', 1));

    ui_->left_motor_torque->setValue(control_system_state->left_motor_torque);
    ui_->left_motor_torque_disp->setText(QString("%1 Nm").arg(
            control_system_state->left_motor_torque, 0, 'f', 0));

    ui_->left_motor_temperature->setValue(control_system_state->left_motor_temperature);
    ui_->left_motor_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->left_motor_temperature, 0, 'f', 1));

    ui_->left_controller_temperature->setValue(control_system_state->left_motor_controller_temperature);
    ui_->left_controller_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->left_motor_controller_temperature, 0, 'f', 1));

    double left_motor_velocity_kRPM = control_system_state->left_motor_velocity * 60 / (2 * M_PI);
//    left_motor_velocity_->setValue(left_motor_velocity_kRPM);
    ui_->left_motor_velocity_disp->setText(QString("%1").arg(
            left_motor_velocity_kRPM, 0, 'f', 0));

//    Right Motor
    ui_->right_motor_current->setValue(control_system_state->right_motor_battery_current);
    ui_->right_motor_current_disp->setText(QString("%1 A").arg(
            control_system_state->right_motor_battery_current, 0, 'f', 1));

    ui_->right_motor_torque->setValue(control_system_state->right_motor_torque);
    ui_->right_motor_torque_disp->setText(QString("%1 Nm").arg(
            control_system_state->right_motor_torque, 0, 'f', 0));

    ui_->right_motor_temperature->setValue(control_system_state->right_motor_temperature);
    ui_->right_motor_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->right_motor_temperature, 0, 'f', 1));

    ui_->right_controller_temperature->setValue(control_system_state->right_motor_controller_temperature);
    ui_->right_controller_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->right_motor_controller_temperature, 0, 'f', 1));

    double right_motor_velocity_kRPM = control_system_state->right_motor_velocity * 60 / (2 * M_PI);
    ui_->right_motor_velocity->setValue(right_motor_velocity_kRPM);
    ui_->right_motor_velocity_disp->setText(QString("%1").arg(
            right_motor_velocity_kRPM, 0, 'f', 0));

  }

}
