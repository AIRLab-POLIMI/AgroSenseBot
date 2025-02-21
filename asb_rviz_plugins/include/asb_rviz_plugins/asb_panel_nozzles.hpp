#ifndef ASB_RVIZ_PLUGINS__ASB_PANEL_HPP
#define ASB_RVIZ_PLUGINS__ASB_PANEL_HPP

#include <QVBoxLayout>
#include <QLabel>

#include <rviz_common/panel.hpp>
#include "asb_rviz_plugins/asb_widget_nozzles.h"

namespace asb_rviz_plugins
{

  class ASBPanelNozzles : public rviz_common::Panel
  {
  Q_OBJECT
  public:
    explicit ASBPanelNozzles(QWidget * parent = nullptr);
    virtual ~ASBPanelNozzles();

    void onInitialize() override;
    void save(rviz_common::Config config) const override;
    void load(const rviz_common::Config &conf) override;


  private:
    ASBWidgetNozzles *widget_;
    rclcpp::Subscription<asb_msgs::msg::NozzleCommandArray>::SharedPtr nozzles_command_subscriber_;

  };
}

#endif // ASB_RVIZ_PLUGINS__ASB_PANEL_HPP
