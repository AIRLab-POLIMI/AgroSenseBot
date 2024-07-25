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
                      56, 46);

    battery_soc_ = new ASBThermo;
    substitute_widget(battery_soc_, ui_->battery_soc, ui_->battery_soc_layout,
                      100, 20);

//  Left Motor
    left_motor_velocity_ = new ASBThermo;
    substitute_widget(left_motor_velocity_, ui_->left_motor_velocity, ui_->left_motor_velocity_layout,
                      2400, -2400, true);

    left_motor_current_ = new ASBThermo;
    substitute_widget(left_motor_current_, ui_->left_motor_current, ui_->left_motor_current_layout,
                      130, 0);

    left_motor_temperature_ = new ASBThermo;
    substitute_widget(left_motor_temperature_, ui_->left_motor_temperature, ui_->left_motor_temperature_layout,
                      60, -10);

    left_controller_temperature_ = new ASBThermo;
    substitute_widget(left_controller_temperature_, ui_->left_controller_temperature, ui_->left_controller_temperature_layout,
                      60, -10);

//  Right Motor
    right_motor_velocity_ = new ASBThermo;
    substitute_widget(right_motor_velocity_, ui_->right_motor_velocity, ui_->right_motor_velocity_layout,
                      2400, -2400, true);

    right_motor_current_ = new ASBThermo;
    substitute_widget(right_motor_current_, ui_->right_motor_current, ui_->right_motor_current_layout,
                      130, 0);

    right_motor_temperature_ = new ASBThermo;
    substitute_widget(right_motor_temperature_, ui_->right_motor_temperature, ui_->right_motor_temperature_layout,
                      60, -10);

    right_controller_temperature_ = new ASBThermo;
    substitute_widget(right_controller_temperature_, ui_->right_controller_temperature, ui_->right_controller_temperature_layout,
                      60, -10);

//  Fan Motor
    fan_motor_velocity_ = new ASBThermo;
    substitute_widget(fan_motor_velocity_, ui_->fan_motor_velocity, ui_->fan_motor_velocity_layout,
                      2400, 0, true);

    fan_motor_current_ = new ASBThermo;
    substitute_widget(fan_motor_current_, ui_->fan_motor_current, ui_->fan_motor_current_layout,
                      20, 0);

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

  bool ASBWidget::is_alarm_on(ASBThermo* widget) {
    return (widget->value() > widget->upperAlarmLevel()) || (widget->value() < widget->lowerAlarmLevel());
  }

  void ASBWidget::control_system_state_callback(const asb_msgs::msg::ControlSystemState::SharedPtr control_system_state) const {

    control_system_state_timeout_timer_->reset();

//    System status
    ui_->sw_emergency_stop_disp->setText(QString(control_system_state->software_emergency_stop ? "ENABLED":"OK"));
    ui_->sw_emergency_stop_disp->setStyleSheet(control_system_state->software_emergency_stop ? yel_bg:no_bg);

    if(control_system_state->gcu_comm_started) {
      if (control_system_state->gcu_alive_bit_rate_critical) {
        ui_->gcu_comm_disp->setText(QString("DOWN"));
        ui_->gcu_comm_disp->setStyleSheet(yel_bg);
      } else if (control_system_state->gcu_alive_bit_rate_low) {
        ui_->gcu_comm_disp->setText(QString("RATE LOW"));
        ui_->gcu_comm_disp->setStyleSheet(yel_bg);
      } else {
        ui_->gcu_comm_disp->setText(QString("OK"));
        ui_->gcu_comm_disp->setStyleSheet(no_bg);
      }
    } else {
      ui_->gcu_comm_disp->setText(QString("WAITING"));
      ui_->gcu_comm_disp->setStyleSheet(yel_bg);
    }

    if(control_system_state->vcu_comm_started) {
      if(control_system_state->vcu_comm_ok) {
        ui_->vcu_comm_disp->setText(QString("OK"));
        ui_->vcu_comm_disp->setStyleSheet(no_bg);

        ui_->vcu_safety_disp->setText(QString(control_system_state->vcu_safety_status ? "OK":"LOCK"));
        ui_->vcu_safety_disp->setStyleSheet(control_system_state->vcu_safety_status ? no_bg:yel_bg);

        ui_->control_mode_disp->setText(control_mode_string[control_system_state->control_mode]);
        ui_->control_mode_disp->setStyleSheet(no_bg);

        ui_->pump_disp->setText(QString(control_system_state->pump_state ? "ON":"OFF"));
        ui_->pump_disp->setStyleSheet(no_bg);

        if((control_system_state->more_recent_active_alarm_id == 92) || (control_system_state->more_recent_active_alarm_id == 0)) {
          ui_->vcu_error_disp->setText(QString("OK (%1)").arg(control_system_state->more_recent_active_alarm_id));
          ui_->vcu_error_disp->setStyleSheet(no_bg);
        } else {
          ui_->vcu_error_disp->setText(QString("CODE %1").arg(control_system_state->more_recent_active_alarm_id));
          ui_->vcu_error_disp->setStyleSheet(yel_bg);
        }
      } else {
        ui_->vcu_comm_disp->setText(QString("DOWN"));
        ui_->vcu_comm_disp->setStyleSheet(yel_bg);

        ui_->vcu_safety_disp->setText(QString("UNKNOWN"));
        ui_->vcu_safety_disp->setStyleSheet(no_bg);

        ui_->control_mode_disp->setText(QString("UNKNOWN"));
        ui_->control_mode_disp->setStyleSheet(no_bg);

        ui_->pump_disp->setText(QString("UNKNOWN"));
        ui_->pump_disp->setStyleSheet(no_bg);

        ui_->vcu_error_disp->setText(QString("UNKNOWN"));
        ui_->vcu_error_disp->setStyleSheet(no_bg);
      }
    } else {
      ui_->vcu_comm_disp->setText(QString("WAITING"));
      ui_->vcu_comm_disp->setStyleSheet(yel_bg);

      ui_->vcu_safety_disp->setText(QString("UNKNOWN"));
      ui_->vcu_safety_disp->setStyleSheet(no_bg);

      ui_->control_mode_disp->setText(QString("UNKNOWN"));
      ui_->control_mode_disp->setStyleSheet(no_bg);

      ui_->pump_disp->setText(QString("UNKNOWN"));
      ui_->pump_disp->setStyleSheet(no_bg);

      ui_->vcu_error_disp->setText(QString("UNKNOWN"));
      ui_->vcu_error_disp->setStyleSheet(no_bg);
    }

//    Battery
    battery_soc_->setValue((double)control_system_state->left_motor_bdi_percentage);
    ui_->battery_soc_disp->setText(QString("%1 %").arg(
            control_system_state->left_motor_bdi_percentage));
    ui_->battery_soc_disp->setStyleSheet(is_alarm_on(battery_soc_) ? red_bg:no_bg);

    battery_voltage_->setValue(control_system_state->left_motor_keyswitch_voltage);
    ui_->battery_voltage_disp->setText(QString("%1 V").arg(
            control_system_state->left_motor_keyswitch_voltage, 0, 'f', 2));
    ui_->battery_voltage_disp->setStyleSheet(is_alarm_on(battery_voltage_) ? red_bg:no_bg);

//    Left Motor
    double left_motor_velocity_RPM = control_system_state->left_motor_velocity * 60 / (2 * M_PI);
    left_motor_velocity_->setValue(left_motor_velocity_RPM);
    double left_motor_velocity_setpoint_RPM = control_system_state->left_motor_velocity_setpoint * 60 / (2 * M_PI);
    left_motor_velocity_->setSetpointValue(left_motor_velocity_setpoint_RPM);
    ui_->left_motor_velocity_disp->setText(QString("%1").arg(
            left_motor_velocity_RPM, 0, 'f', 0));
    ui_->left_motor_velocity_disp->setStyleSheet(is_alarm_on(left_motor_velocity_) ? red_bg:no_bg);

    left_motor_current_->setValue(std::abs(control_system_state->left_motor_battery_current));
    ui_->left_motor_current_disp->setText(QString("%1 A").arg(
            control_system_state->left_motor_battery_current, 0, 'f', 1));
    ui_->left_motor_current_disp->setStyleSheet(is_alarm_on(left_motor_current_) ? red_bg:no_bg);

    left_motor_temperature_->setValue(control_system_state->left_motor_temperature);
    ui_->left_motor_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->left_motor_temperature, 0, 'f', 1));
    ui_->left_motor_temperature_disp->setStyleSheet(is_alarm_on(left_motor_temperature_) ? red_bg:no_bg);

    left_controller_temperature_->setValue(control_system_state->left_motor_controller_temperature);
    ui_->left_controller_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->left_motor_controller_temperature, 0, 'f', 1));
    ui_->left_controller_temperature_disp->setStyleSheet(is_alarm_on(left_controller_temperature_) ? red_bg:no_bg);

//    Right Motor
    double right_motor_velocity_RPM = control_system_state->right_motor_velocity * 60 / (2 * M_PI);
    right_motor_velocity_->setValue(right_motor_velocity_RPM);
    double right_motor_velocity_setpoint_RPM = control_system_state->right_motor_velocity_setpoint * 60 / (2 * M_PI);
    right_motor_velocity_->setSetpointValue(right_motor_velocity_setpoint_RPM);
    ui_->right_motor_velocity_disp->setText(QString("%1").arg(
            right_motor_velocity_RPM, 0, 'f', 0));
    ui_->right_motor_velocity_disp->setStyleSheet(is_alarm_on(right_motor_velocity_) ? red_bg:no_bg);

    right_motor_current_->setValue(std::abs(control_system_state->right_motor_battery_current));
    ui_->right_motor_current_disp->setText(QString("%1 A").arg(
            control_system_state->right_motor_battery_current, 0, 'f', 1));
    ui_->right_motor_current_disp->setStyleSheet(is_alarm_on(right_motor_current_) ? red_bg:no_bg);

    right_motor_temperature_->setValue(control_system_state->right_motor_temperature);
    ui_->right_motor_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->right_motor_temperature, 0, 'f', 1));
    ui_->right_motor_temperature_disp->setStyleSheet(is_alarm_on(right_motor_temperature_) ? red_bg:no_bg);

    right_controller_temperature_->setValue(control_system_state->right_motor_controller_temperature);
    ui_->right_controller_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->right_motor_controller_temperature, 0, 'f', 1));
    ui_->right_controller_temperature_disp->setStyleSheet(is_alarm_on(right_controller_temperature_) ? red_bg:no_bg);

//    Fan Motor
    fan_motor_velocity_->setValue(control_system_state->fan_motor_velocity_rpm);
    fan_motor_velocity_->setSetpointValue(control_system_state->fan_motor_velocity_setpoint_rpm);
    ui_->fan_motor_velocity_disp->setText(QString("%1").arg(
            control_system_state->fan_motor_velocity_rpm, 0, 'f', 0));
    ui_->fan_motor_velocity_disp->setStyleSheet(is_alarm_on(fan_motor_velocity_) ? red_bg:no_bg);

    fan_motor_current_->setValue(std::abs(control_system_state->fan_motor_battery_current));
    ui_->fan_motor_current_disp->setText(QString("%1 A").arg(
            control_system_state->fan_motor_battery_current, 0, 'f', 1));
    ui_->fan_motor_current_disp->setStyleSheet(is_alarm_on(fan_motor_current_) ? red_bg:no_bg);

    fan_motor_temperature_->setValue(control_system_state->fan_motor_temperature);
    ui_->fan_motor_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->fan_motor_temperature, 0, 'f', 1));
    ui_->fan_motor_temperature_disp->setStyleSheet(is_alarm_on(fan_motor_temperature_) ? red_bg:no_bg);

    fan_controller_temperature_->setValue(control_system_state->fan_motor_controller_temperature);
    ui_->fan_controller_temperature_disp->setText(QString("%1 °C").arg(
            control_system_state->fan_motor_controller_temperature, 0, 'f', 1));
    ui_->fan_controller_temperature_disp->setStyleSheet(is_alarm_on(fan_controller_temperature_) ? red_bg:no_bg);

  }

  void ASBWidget::control_system_state_timeout_callback() {

    ui_->sw_emergency_stop_disp->setText(QString("UNKNOWN"));
    ui_->sw_emergency_stop_disp->setStyleSheet(no_bg);

    ui_->gcu_comm_disp->setText(QString("NO ROS DATA"));
    ui_->gcu_comm_disp->setStyleSheet(yel_bg);

    ui_->vcu_comm_disp->setText(QString("UNKNOWN"));
    ui_->vcu_comm_disp->setStyleSheet(no_bg);

    ui_->vcu_safety_disp->setText(QString("UNKNOWN"));
    ui_->vcu_safety_disp->setStyleSheet(no_bg);

    ui_->vcu_error_disp->setText(QString("UNKNOWN"));
    ui_->vcu_error_disp->setStyleSheet(no_bg);

    ui_->control_mode_disp->setText(QString("UNKNOWN"));
    ui_->control_mode_disp->setStyleSheet(no_bg);

    ui_->pump_disp->setText(QString("UNKNOWN"));
    ui_->pump_disp->setStyleSheet(no_bg);

  }

  void ASBWidget::rtk_status_callback(const microstrain_inertial_msgs::msg::HumanReadableStatus::SharedPtr rtk_status) const {

    rtk_status_timeout_timer_->reset();

    ui_->gnss_state_disp->setText(QString(rtk_status->gnss_state.c_str()));
    ui_->gnss_state_disp->setStyleSheet(rtk_status->gnss_state == "RTK Fixed" ? no_bg : yel_bg);

    auto dual_antenna_fix_type_qstring = QString(rtk_status->dual_antenna_fix_type.c_str());
    dual_antenna_fix_type_qstring.replace("Dual Antenna", "Dual Ant.");
    ui_->dual_antenna_fix_type_disp->setText(dual_antenna_fix_type_qstring);
    ui_->dual_antenna_fix_type_disp->setStyleSheet(rtk_status->dual_antenna_fix_type == "Dual Antenna Fixed" ? no_bg : yel_bg);

    ui_->filter_state_disp->setText(QString(rtk_status->filter_state.c_str()));
    ui_->filter_state_disp->setStyleSheet(rtk_status->filter_state == "Full Nav" ? no_bg : yel_bg);

    std::string status_flags_disp;
    for(size_t i = 0; i < rtk_status->status_flags.size(); i++) {
      if(rtk_status->status_flags.size() > 1) status_flags_disp.append("- ");
      status_flags_disp.append(rtk_status->status_flags[i]);
      if(i < rtk_status->status_flags.size() - 1) status_flags_disp.append("\n");
    }
    ui_->status_flags_disp->setText(QString(status_flags_disp.c_str()));
    ui_->status_flags_disp->setStyleSheet((rtk_status->status_flags.size() == 1) && (rtk_status->status_flags[0] == "Stable") ? no_bg : yel_bg);

  }

  void ASBWidget::rtk_status_timeout_callback() {

    ui_->gnss_state_disp->setText(QString("UNKNOWN"));
    ui_->gnss_state_disp->setStyleSheet(yel_bg);

    ui_->dual_antenna_fix_type_disp->setText(QString("UNKNOWN"));
    ui_->dual_antenna_fix_type_disp->setStyleSheet(yel_bg);

    ui_->filter_state_disp->setText(QString("UNKNOWN"));
    ui_->filter_state_disp->setStyleSheet(yel_bg);

    ui_->status_flags_disp->setText(QString("UNKNOWN"));
    ui_->status_flags_disp->setStyleSheet(yel_bg);

  }

}
