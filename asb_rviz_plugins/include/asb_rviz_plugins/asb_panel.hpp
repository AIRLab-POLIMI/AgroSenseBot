#ifndef ASB_RVIZ_PLUGINS__ASB_PANEL_HPP
#define ASB_RVIZ_PLUGINS__ASB_PANEL_HPP

#include <QVBoxLayout>
#include <QLabel>

#include <rviz_common/panel.hpp>
#include "asb_rviz_plugins/asb_widget.h"

namespace asb_rviz_plugins
{
//  class ASBWidget;

  class ASBPanel : public rviz_common::Panel
  {
  Q_OBJECT
  public:
    explicit ASBPanel(QWidget * parent = nullptr);
    virtual ~ASBPanel();

    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config &conf) override;


  private:
    ASBWidget *widget_;
    rclcpp::Subscription<asb_msgs::msg::PlatformState>::SharedPtr platform_state_subscriber_;
    rclcpp::Subscription<microstrain_inertial_msgs::msg::HumanReadableStatus>::SharedPtr rtk_status_subscriber_;

  };
}

#endif // ASB_RVIZ_PLUGINS__ASB_PANEL_HPP
