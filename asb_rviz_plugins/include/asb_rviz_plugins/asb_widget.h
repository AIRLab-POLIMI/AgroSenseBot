#ifndef ASB_RVIZ_PLUGINS__ASB_WIDGET_H
#define ASB_RVIZ_PLUGINS__ASB_WIDGET_H

#include <QWidget>
#include <QPushButton>
#include <memory>
#include <vector>

#include "ui_asb.h"
//#include "asb_rviz_plugins/ui_asb.h"

#ifndef Q_MOC_RUN
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#endif

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
  public Q_SLOTS:
  private Q_SLOTS:
  protected:
    std::unique_ptr<Ui::ASBWidgetUI> ui_;

  private:
    rclcpp::Node::SharedPtr _node;
//    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscriber_;
  };
}
// namespace asb_rviz_plugins

#endif  // ASB_RVIZ_PLUGINS__ASB_WIDGET_H
