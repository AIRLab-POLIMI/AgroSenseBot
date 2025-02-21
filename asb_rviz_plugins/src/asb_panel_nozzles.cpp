#include <QVBoxLayout>

//#include <memory>
#include <vector>
#include <utility>
#include <rviz_common/display_context.hpp>

#include "asb_rviz_plugins/asb_panel_nozzles.hpp"

using namespace std::chrono_literals;

namespace asb_rviz_plugins
{
  ASBPanelNozzles::ASBPanelNozzles(QWidget * parent) : Panel(parent)
  {
    widget_ = new ASBWidgetNozzles(parent);
    auto* layout = new QVBoxLayout;
    layout->addWidget(widget_);
    layout->setContentsMargins(10, 10, 10, 10);
    setLayout(layout);
  }

  void ASBPanelNozzles::save(rviz_common::Config config) const {
    Panel::save(config);
  }

  void ASBPanelNozzles::load(const rviz_common::Config &conf) {
    Panel::load(conf);
  }

  void ASBPanelNozzles::onInitialize() {
    auto node = getDisplayContext()->getRosNodeAbstraction().lock()->get_raw_node();
    nozzles_command_subscriber_ = node->create_subscription<asb_msgs::msg::NozzleCommandArray>(
      "/nozzles_command", rclcpp::SensorDataQoS(),
      std::bind(&ASBWidgetNozzles::nozzles_command_callback, widget_, _1));

    std::chrono::duration nozzles_command_msg_timeout = 100ms;
    widget_->nozzles_command_timeout_timer_ = rclcpp::create_timer(
      node, node->get_clock(), rclcpp::Duration(nozzles_command_msg_timeout),
      std::bind(&ASBWidgetNozzles::nozzles_command_timeout_callback, widget_));

    RCLCPP_INFO(node->get_logger(), "AgroSenseBot Panel initialized");
  }

  ASBPanelNozzles::~ASBPanelNozzles() = default;

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(asb_rviz_plugins::ASBPanelNozzles, rviz_common::Panel)
