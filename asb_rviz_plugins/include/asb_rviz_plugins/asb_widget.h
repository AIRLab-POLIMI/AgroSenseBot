#ifndef ASB_RVIZ_PLUGINS__ASB_WIDGET_H
#define ASB_RVIZ_PLUGINS__ASB_WIDGET_H

#include <QWidget>
#include <QPushButton>


#include <memory>
#include <vector>

#include <rclcpp/qos.hpp>
#include "asb_msgs/msg/platform_state.hpp"
#include "microstrain_inertial_msgs/msg/human_readable_status.hpp"

#include <asb_rviz_plugins/asb_linear_dial.h>

#include "ui_asb.h"

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

using std::placeholders::_1;

namespace Ui {
  class ASBWidgetUI;
}

namespace asb_rviz_plugins
{
  class ASBWidget : public QWidget
  {
  Q_OBJECT
  public:

    explicit ASBWidget(QWidget * parent = nullptr);

    ~ASBWidget() override = default;

    static void substitute_widget(ASBThermo* widget, QwtThermo* orig_widget, QLayout* layout,
                                  double upper_alarm_level, double lower_alarm_level, bool setpoint_enabled = false);

    static bool is_alarm_on(ASBThermo* widget);

    void platform_state_callback(asb_msgs::msg::PlatformState::SharedPtr platform_state) const;

    void rtk_status_callback(microstrain_inertial_msgs::msg::HumanReadableStatus::SharedPtr rtk_status) const;

    void platform_state_timeout_callback();

    void rtk_status_timeout_callback();

    rclcpp::TimerBase::SharedPtr platform_state_timeout_timer_;

    rclcpp::TimerBase::SharedPtr rtk_status_timeout_timer_;

  public Q_SLOTS:
  private Q_SLOTS:
  protected:
    std::unique_ptr<Ui::ASBWidgetUI> ui_;

//  Battery
    ASBThermo* battery_voltage_ = nullptr;
    ASBThermo* battery_soc_ = nullptr;

//  Left Motor
    ASBThermo* left_motor_velocity_ = nullptr;
    ASBThermo* left_motor_current_ = nullptr;
    ASBThermo* left_motor_temperature_ = nullptr;
    ASBThermo* left_controller_temperature_ = nullptr;

//  Right Motor
    ASBThermo* right_motor_velocity_ = nullptr;
    ASBThermo* right_motor_current_ = nullptr;
    ASBThermo* right_motor_temperature_ = nullptr;
    ASBThermo* right_controller_temperature_ = nullptr;

//  Fan Motor
    ASBThermo* fan_motor_velocity_ = nullptr;
    ASBThermo* fan_motor_current_ = nullptr;
    ASBThermo* fan_motor_temperature_ = nullptr;
    ASBThermo* fan_controller_temperature_ = nullptr;

  private:
    std::vector<QString> control_mode_string = {"STOP", "MANUAL", "AUTO", "OVERRIDE"};
    QString yel_bg = "QLabel {background-color: yellow;}";
    QString red_bg = "QLabel {background-color: red;}";
    QString no_bg = "QLabel {}";

  };
}
// namespace asb_rviz_plugins

#endif  // ASB_RVIZ_PLUGINS__ASB_WIDGET_H
