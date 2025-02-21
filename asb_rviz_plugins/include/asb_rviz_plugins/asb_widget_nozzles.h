#ifndef ASB_RVIZ_PLUGINS__ASB_WIDGET_H
#define ASB_RVIZ_PLUGINS__ASB_WIDGET_H

#include <QWidget>
#include <QPushButton>


#include <memory>
#include <vector>

#include <rclcpp/qos.hpp>
#include "asb_msgs/msg/nozzle_command_array.hpp"
#include "asb_msgs/msg/nozzle_command.hpp"
#include "microstrain_inertial_msgs/msg/human_readable_status.hpp"

#include <asb_rviz_plugins/asb_linear_dial.h>

#include "ui_asb_nozzles.h"

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

using std::placeholders::_1;

namespace Ui {
  class ASBWidgetNozzlesUI;
}

namespace asb_rviz_plugins
{
  class ASBWidgetNozzles : public QWidget
  {
  Q_OBJECT
  public:

    explicit ASBWidgetNozzles(QWidget * parent = nullptr);

    ~ASBWidgetNozzles() override = default;

    void nozzles_command_callback(asb_msgs::msg::NozzleCommandArray::SharedPtr nozzles_command) const;

    void nozzles_command_timeout_callback();

    rclcpp::TimerBase::SharedPtr nozzles_command_timeout_timer_;

  public Q_SLOTS:
  private Q_SLOTS:
  protected:
    std::unique_ptr<Ui::ASBWidgetNozzlesUI> ui_;

  private:
    std::vector<QString> control_mode_string = {"STOP", "MANUAL", "AUTO", "OVERRIDE"};
    QString yel_bg = "QLabel {background-color: yellow;}";
    QString red_bg = "QLabel {background-color: red;}";
    QString no_bg = "QLabel {}";

  };
}
// namespace asb_rviz_plugins

#endif  // ASB_RVIZ_PLUGINS__ASB_WIDGET_H
