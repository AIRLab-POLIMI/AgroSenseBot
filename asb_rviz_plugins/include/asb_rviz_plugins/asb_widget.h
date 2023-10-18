#ifndef ASB_RVIZ_PLUGINS__ASB_WIDGET_H
#define ASB_RVIZ_PLUGINS__ASB_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <memory>
#include <vector>

#include <rclcpp/qos.hpp>
#include "asb_msgs/msg/control_system_state.hpp"

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

    void control_system_state_callback(asb_msgs::msg::ControlSystemState::SharedPtr control_system_state) const;

  public Q_SLOTS:
  private Q_SLOTS:
  protected:
    std::unique_ptr<Ui::ASBWidgetUI> ui_;

  private:

  };
}
// namespace asb_rviz_plugins

#endif  // ASB_RVIZ_PLUGINS__ASB_WIDGET_H
