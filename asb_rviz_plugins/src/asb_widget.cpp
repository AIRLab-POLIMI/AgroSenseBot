#include <asb_rviz_plugins/asb_widget.h>

namespace asb_rviz_plugins
{
  ASBWidget::ASBWidget(QWidget *parent): QWidget(parent), ui_(new Ui::ASBWidgetUI)
  {
    ui_->setupUi(this);

    auto options = rclcpp::NodeOptions().arguments({"--ros-args --remap __node:=dialog_action_client"});
    _node = std::make_shared<rclcpp::Node>("_", options);

//    _publisher = _node->create_publisher<std_msgs::msg::String>("gui_control", 10);
  }

}
