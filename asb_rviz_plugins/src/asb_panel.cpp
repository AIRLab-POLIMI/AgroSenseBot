#include <QVBoxLayout>

//#include <memory>
#include <vector>
#include <utility>
#include <rviz_common/display_context.hpp>

#include "asb_rviz_plugins/asb_panel.hpp"

namespace asb_rviz_plugins
{
  ASBPanel::ASBPanel(QWidget * parent) : Panel(parent)
  {
    widget_ = new ASBWidget(parent);
    QVBoxLayout * layout = new QVBoxLayout;
    layout->addWidget(widget_);
    layout->setContentsMargins(10, 10, 10, 10);
    setLayout(layout);
  }

  void ASBPanel::save(rviz_common::Config config) const {
    Panel::save(config);
  }

  void ASBPanel::load(const rviz_common::Config &conf) {
    Panel::load(conf);
  }

  void ASBPanel::onInitialize() {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    control_system_state_subscriber_ = node->create_subscription<asb_msgs::msg::ControlSystemState>(
            "/asb_control_system_status_controller/control_system_state", rclcpp::SensorDataQoS(),
            std::bind(&ASBWidget::control_system_state_callback, widget_, _1));

    RCLCPP_INFO(node->get_logger(), "AgroSenseBot Panel initialized");
  }

  ASBPanel::~ASBPanel() = default;

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(asb_rviz_plugins::ASBPanel, rviz_common::Panel)
