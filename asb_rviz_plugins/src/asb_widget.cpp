#include <asb_rviz_plugins/asb_widget.h>

namespace asb_rviz_plugins {

  ASBWidget::ASBWidget(QWidget *parent): QWidget(parent), ui_(new Ui::ASBWidgetUI) {

//  remove comma as a thousand separator from scales in ASB/Qwt Thermo widgets
    QLocale locale;
    locale.setNumberOptions(QLocale::OmitGroupSeparator);
    QLocale::setDefault(locale);

    ui_->setupUi(this);

//  Battery
    battery_voltage_ = new ASBThermo;
    substitute_widget(battery_voltage_, ui_->battery_voltage, ui_->battery_voltage_layout,
                      28, 22);

    battery_soc_ = new ASBThermo;
    substitute_widget(battery_soc_, ui_->battery_soc, ui_->battery_soc_layout,
                      100, 10);

//  Left Motor
    left_motor_velocity_ = new ASBThermo;
    substitute_widget(left_motor_velocity_, ui_->left_motor_velocity, ui_->left_motor_velocity_layout,
                      2400, -2400, true);

    left_motor_current_ = new ASBThermo;
    substitute_widget(left_motor_current_, ui_->left_motor_current, ui_->left_motor_current_layout,
                      900, 0);

    left_motor_torque_ = new ASBThermo;
    substitute_widget(left_motor_torque_, ui_->left_motor_torque, ui_->left_motor_torque_layout,
                      900, 0);

    left_motor_temperature_ = new ASBThermo;
    substitute_widget(left_motor_temperature_, ui_->left_motor_temperature, ui_->left_motor_temperature_layout,
                      70, -10);

    left_controller_temperature_ = new ASBThermo;
    substitute_widget(left_controller_temperature_, ui_->left_controller_temperature, ui_->left_controller_temperature_layout,
                      70, -10);

//  Right Motor
    right_motor_velocity_ = new ASBThermo;
    substitute_widget(right_motor_velocity_, ui_->right_motor_velocity, ui_->right_motor_velocity_layout,
                      2400, -2400, true);

    right_motor_current_ = new ASBThermo;
    substitute_widget(right_motor_current_, ui_->right_motor_current, ui_->right_motor_current_layout,
                      900, 0);

    right_motor_torque_ = new ASBThermo;
    substitute_widget(right_motor_torque_, ui_->right_motor_torque, ui_->right_motor_torque_layout,
                      900, 0);

    right_motor_temperature_ = new ASBThermo;
    substitute_widget(right_motor_temperature_, ui_->right_motor_temperature, ui_->right_motor_temperature_layout,
                      70, -10);

    right_controller_temperature_ = new ASBThermo;
    substitute_widget(right_controller_temperature_, ui_->right_controller_temperature, ui_->right_controller_temperature_layout,
                      70, -10);

//  Fan Motor
    fan_motor_velocity_ = new ASBThermo;
    substitute_widget(fan_motor_velocity_, ui_->fan_motor_velocity, ui_->fan_motor_velocity_layout,
                      2400, 0, true);

    fan_motor_current_ = new ASBThermo;
    substitute_widget(fan_motor_current_, ui_->fan_motor_current, ui_->fan_motor_current_layout,
                      900, 0);

    fan_motor_torque_ = new ASBThermo;
    substitute_widget(fan_motor_torque_, ui_->fan_motor_torque, ui_->fan_motor_torque_layout,
                      900, 0);

    fan_motor_temperature_ = new ASBThermo;
    substitute_widget(fan_motor_temperature_, ui_->fan_motor_temperature, ui_->fan_motor_temperature_layout,
                      70, -10);

    fan_controller_temperature_ = new ASBThermo;
    substitute_widget(fan_controller_temperature_, ui_->fan_controller_temperature, ui_->fan_controller_temperature_layout,
                      70, -10);

  }

  void ASBWidget::substitute_widget(ASBThermo* widget, QwtThermo* orig_widget, QLayout* layout,
                                    double upper_alarm_level, double lower_alarm_level, bool setpoint_enabled) {
    widget->setMinimumHeight(orig_widget->minimumHeight());
    widget->setMinimumHeight(orig_widget->minimumHeight());
    widget->setUpperBound(orig_widget->upperBound());
    widget->setLowerBound(orig_widget->lowerBound());
    widget->setScaleMaxMajor(orig_widget->scaleMaxMajor());
    widget->setScaleMaxMinor(orig_widget->scaleMaxMinor());
    widget->setScaleStepSize(orig_widget->scaleStepSize());
    widget->setOriginMode(ASBThermo::OriginMode::OriginCustom);
    widget->setOrigin(orig_widget->origin());
    widget->setUpperAlarmLevel(upper_alarm_level);
    widget->setLowerAlarmLevel(lower_alarm_level);
    widget->setSpacing(orig_widget->spacing());
    widget->setPalette(orig_widget->palette());
    widget->setSetpointEnabled(setpoint_enabled);
    orig_widget->setHidden(true);
    layout->addWidget(widget);
  }

  void ASBWidget::control_system_state_callback(const asb_msgs::msg::ControlSystemState::SharedPtr control_system_state) const {

    left_motor_velocity_->setSetpointValue(1000);
    right_motor_velocity_->setSetpointValue(1000);
//    fan_motor_velocity_->setSetpointValue(1400);

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
    battery_soc_->setValue((double)control_system_state->left_motor_bdi_percentage);
    ui_->battery_soc_disp->setText(QString("%1 %").arg(
            control_system_state->left_motor_bdi_percentage));

    battery_voltage_->setValue(control_system_state->left_motor_keyswitch_voltage);
    ui_->battery_voltage_disp->setText(QString("%1 V").arg(
            control_system_state->left_motor_keyswitch_voltage, 0, 'f', 2));

//    Left Motor
    double left_motor_velocity_RPM = control_system_state->left_motor_velocity * 60 / (2 * M_PI);
    left_motor_velocity_->setValue(left_motor_velocity_RPM);
    ui_->left_motor_velocity_disp->setText(QString("%1").arg(
            left_motor_velocity_RPM, 0, 'f', 0));

    left_motor_current_->setValue(control_system_state->left_motor_battery_current);
    ui_->left_motor_current_disp->setText(QString("%1 A").arg(
            control_system_state->left_motor_battery_current, 0, 'f', 0));

    left_motor_torque_->setValue(control_system_state->left_motor_torque);
    ui_->left_motor_torque_disp->setText(QString("%1 Nm").arg(
            control_system_state->left_motor_torque, 0, 'f', 0));

    left_motor_temperature_->setValue(control_system_state->left_motor_temperature);
    ui_->left_motor_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->left_motor_temperature, 0, 'f', 1));

    left_controller_temperature_->setValue(control_system_state->left_motor_controller_temperature);
    ui_->left_controller_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->left_motor_controller_temperature, 0, 'f', 1));

//    Right Motor
    double right_motor_velocity_RPM = control_system_state->right_motor_velocity * 60 / (2 * M_PI);
    right_motor_velocity_->setValue(right_motor_velocity_RPM);
    ui_->right_motor_velocity_disp->setText(QString("%1").arg(
            right_motor_velocity_RPM, 0, 'f', 0));

    right_motor_current_->setValue(control_system_state->right_motor_battery_current);
    ui_->right_motor_current_disp->setText(QString("%1 A").arg(
            control_system_state->right_motor_battery_current, 0, 'f', 0));

    right_motor_torque_->setValue(control_system_state->right_motor_torque);
    ui_->right_motor_torque_disp->setText(QString("%1 Nm").arg(
            control_system_state->right_motor_torque, 0, 'f', 0));

    right_motor_temperature_->setValue(control_system_state->right_motor_temperature);
    ui_->right_motor_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->right_motor_temperature, 0, 'f', 1));

    right_controller_temperature_->setValue(control_system_state->right_motor_controller_temperature);
    ui_->right_controller_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->right_motor_controller_temperature, 0, 'f', 1));

//    Fan Motor
    fan_motor_velocity_->setValue(control_system_state->fan_motor_velocity_rpm);
    ui_->fan_motor_velocity_disp->setText(QString("%1").arg(
            control_system_state->fan_motor_velocity_rpm, 0, 'f', 0));

    fan_motor_current_->setValue(control_system_state->fan_motor_battery_current);
    ui_->fan_motor_current_disp->setText(QString("%1 A").arg(
            control_system_state->fan_motor_battery_current, 0, 'f', 0));

    fan_motor_torque_->setValue(control_system_state->fan_motor_torque);
    ui_->fan_motor_torque_disp->setText(QString("%1 Nm").arg(
            control_system_state->fan_motor_torque, 0, 'f', 0));

    fan_motor_temperature_->setValue(control_system_state->fan_motor_temperature);
    ui_->fan_motor_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->fan_motor_temperature, 0, 'f', 1));

    fan_controller_temperature_->setValue(control_system_state->fan_motor_controller_temperature);
    ui_->fan_controller_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->fan_motor_controller_temperature, 0, 'f', 1));

  }

}
