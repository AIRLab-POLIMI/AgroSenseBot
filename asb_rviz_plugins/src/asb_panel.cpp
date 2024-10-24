#include <QVBoxLayout>

//#include <memory>
#include <vector>
#include <utility>
#include <rviz_common/display_context.hpp>

#include "asb_rviz_plugins/asb_panel.hpp"

using namespace std::chrono_literals;

namespace asb_rviz_plugins
{
  ASBPanel::ASBPanel(QWidget * parent) : Panel(parent)
  {
    widget_ = new ASBWidget(parent);
    auto* layout = new QVBoxLayout;
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
    platform_state_subscriber_ = node->create_subscription<asb_msgs::msg::PlatformState>(
      "/asb_platform_controller/platform_state", rclcpp::SensorDataQoS(),
      std::bind(&ASBWidget::platform_state_callback, widget_, _1));

    std::chrono::duration platform_state_msg_timeout = 100ms;
    widget_->platform_state_timeout_timer_ = rclcpp::create_timer(
      node, node->get_clock(), rclcpp::Duration(platform_state_msg_timeout),
      std::bind(&ASBWidget::platform_state_timeout_callback, widget_));

    rtk_status_subscriber_ = node->create_subscription<microstrain_inertial_msgs::msg::HumanReadableStatus>(
      "/ekf/status", rclcpp::SensorDataQoS(),
      std::bind(&ASBWidget::rtk_status_callback, widget_, _1));

    std::chrono::duration rtk_status_msg_timeout = 2000ms;
    widget_->rtk_status_timeout_timer_ = rclcpp::create_timer(
      node, node->get_clock(), rclcpp::Duration(rtk_status_msg_timeout),
      std::bind(&ASBWidget::rtk_status_timeout_callback, widget_));

    RCLCPP_INFO(node->get_logger(), "AgroSenseBot Panel initialized");
  }

  ASBPanel::~ASBPanel() = default;

}

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(asb_rviz_plugins::ASBPanel, rviz_common::Panel)
