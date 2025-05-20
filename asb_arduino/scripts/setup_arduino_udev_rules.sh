#!/bin/bash

asb_arduino_conf_dir=`ros2 pkg prefix asb_arduino`/share/asb_arduino/config/

sudo cp ${asb_arduino_conf_dir}/80-arduino-onboard.rules /etc/udev/rules.d/

echo "Installed arduino udev configuration files"

sudo udevadm control --reload-rules
sudo udevadm trigger

echo "Reloaded udev rules"
