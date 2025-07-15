#include <asb_rviz_plugins/asb_widget_nozzles.h>

namespace asb_rviz_plugins {

  ASBWidgetNozzles::ASBWidgetNozzles(QWidget *parent): QWidget(parent), ui_(new Ui::ASBWidgetNozzlesUI) {

//  remove comma as a thousand separator from scales in ASB/Qwt Thermo widgets
    QLocale locale;
    locale.setNumberOptions(QLocale::OmitGroupSeparator);
    QLocale::setDefault(locale);

    ui_->setupUi(this);

  }

  void ASBWidgetNozzles::nozzles_command_callback(const asb_msgs::msg::NozzleCommandArray::SharedPtr nozzles_command) const {

    nozzles_command_timeout_timer_->reset();

    std::map<std::string, int> rates;
    for(auto &nozzle_command: nozzles_command->nozzle_command_array) {
      rates[nozzle_command.nozzle_id] = (int) (100 * nozzle_command.rate);
    }

    ui_->l1_thermo->setValue(rates["1L"]);
    ui_->l1_value->setText(QString("%1").arg(rates["1L"]));
    ui_->l1_value->setStyleSheet(rates["1L"] > 0 ? blue_bg : no_bg);

    ui_->l2_thermo->setValue(rates["2L"]);
    ui_->l2_value->setText(QString("%1").arg(rates["2L"]));
    ui_->l2_value->setStyleSheet(rates["2L"] > 0 ? blue_bg : no_bg);

    ui_->l3_thermo->setValue(rates["3L"]);
    ui_->l3_value->setText(QString("%1").arg(rates["3L"]));
    ui_->l3_value->setStyleSheet(rates["3L"] > 0 ? blue_bg : no_bg);

    ui_->l4_thermo->setValue(rates["4L"]);
    ui_->l4_value->setText(QString("%1").arg(rates["4L"]));
    ui_->l4_value->setStyleSheet(rates["4L"] > 0 ? blue_bg : no_bg);

    ui_->l5_thermo->setValue(rates["5L"]);
    ui_->l5_value->setText(QString("%1").arg(rates["5L"]));
    ui_->l5_value->setStyleSheet(rates["5L"] > 0 ? blue_bg : no_bg);

    ui_->l6_thermo->setValue(rates["6L"]);
    ui_->l6_value->setText(QString("%1").arg(rates["6L"]));
    ui_->l6_value->setStyleSheet(rates["6L"] > 0 ? blue_bg : no_bg);

    ui_->r1_thermo->setValue(rates["1R"]);
    ui_->r1_value->setText(QString("%1").arg(rates["1R"]));
    ui_->r1_value->setStyleSheet(rates["1R"] > 0 ? blue_bg : no_bg);

    ui_->r2_thermo->setValue(rates["2R"]);
    ui_->r2_value->setText(QString("%1").arg(rates["2R"]));
    ui_->r2_value->setStyleSheet(rates["2R"] > 0 ? blue_bg : no_bg);

    ui_->r3_thermo->setValue(rates["3R"]);
    ui_->r3_value->setText(QString("%1").arg(rates["3R"]));
    ui_->r3_value->setStyleSheet(rates["3R"] > 0 ? blue_bg : no_bg);

    ui_->r4_thermo->setValue(rates["4R"]);
    ui_->r4_value->setText(QString("%1").arg(rates["4R"]));
    ui_->r4_value->setStyleSheet(rates["4R"] > 0 ? blue_bg : no_bg);

    ui_->r5_thermo->setValue(rates["5R"]);
    ui_->r5_value->setText(QString("%1").arg(rates["5R"]));
    ui_->r5_value->setStyleSheet(rates["5R"] > 0 ? blue_bg : no_bg);

    ui_->r6_thermo->setValue(rates["6R"]);
    ui_->r6_value->setText(QString("%1").arg(rates["6R"]));
    ui_->r6_value->setStyleSheet(rates["6R"] > 0 ? blue_bg : no_bg);

  }

  void ASBWidgetNozzles::nozzles_command_timeout_callback() {

    ui_->l1_thermo->setValue(0.0);
    ui_->l1_value->setText(QString("---"));
    ui_->l1_value->setStyleSheet(yel_bg);

    ui_->l2_thermo->setValue(0.0);
    ui_->l2_value->setText(QString("---"));
    ui_->l2_value->setStyleSheet(yel_bg);

    ui_->l3_thermo->setValue(0.0);
    ui_->l3_value->setText(QString("---"));
    ui_->l3_value->setStyleSheet(yel_bg);

    ui_->l4_thermo->setValue(0.0);
    ui_->l4_value->setText(QString("---"));
    ui_->l4_value->setStyleSheet(yel_bg);

    ui_->l5_thermo->setValue(0.0);
    ui_->l5_value->setText(QString("---"));
    ui_->l5_value->setStyleSheet(yel_bg);

    ui_->l6_thermo->setValue(0.0);
    ui_->l6_value->setText(QString("---"));
    ui_->l6_value->setStyleSheet(yel_bg);

    ui_->r1_thermo->setValue(0.0);
    ui_->r1_value->setText(QString("---"));
    ui_->r1_value->setStyleSheet(yel_bg);

    ui_->r2_thermo->setValue(0.0);
    ui_->r2_value->setText(QString("---"));
    ui_->r2_value->setStyleSheet(yel_bg);

    ui_->r3_thermo->setValue(0.0);
    ui_->r3_value->setText(QString("---"));
    ui_->r3_value->setStyleSheet(yel_bg);

    ui_->r4_thermo->setValue(0.0);
    ui_->r4_value->setText(QString("---"));
    ui_->r4_value->setStyleSheet(yel_bg);

    ui_->r5_thermo->setValue(0.0);
    ui_->r5_value->setText(QString("---"));
    ui_->r5_value->setStyleSheet(yel_bg);

    ui_->r6_thermo->setValue(0.0);
    ui_->r6_value->setText(QString("---"));
    ui_->r6_value->setStyleSheet(yel_bg);

  }

}
