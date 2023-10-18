#include <asb_rviz_plugins/asb_widget.h>

namespace asb_rviz_plugins
{
  ASBWidget::ASBWidget(QWidget *parent): QWidget(parent), ui_(new Ui::ASBWidgetUI) {
    ui_->setupUi(this);
  }

  void ASBWidget::control_system_state_callback(const asb_msgs::msg::ControlSystemState::SharedPtr control_system_state) const {
//    ui_->battery_soc->setValue(control_system_state->left_motor_bdi_percentage);
//    ui_->battery_soc_disp->setText(QString::asprintf("%.1f %%", control_system_state->left_motor_bdi_percentage));
    std::cout << control_system_state->left_motor_bdi_percentage << std::endl;
//    RCLCPP_INFO(client_node_->get_logger(), "ASBWidget::control_system_state_callback");
  }

}
