#!/bin/bash

asb_ros2_control_conf_dir=`ros2 pkg prefix asb_ros2_control`/share/asb_ros2_control/can_config/

sudo cp ${asb_ros2_control_conf_dir}/80-can2.link /etc/systemd/network/
sudo cp ${asb_ros2_control_conf_dir}/80-can2-network.rules /etc/udev/rules.d/
sudo cp ${asb_ros2_control_conf_dir}/80-can2.network /etc/systemd/network/

sudo cp ${asb_ros2_control_conf_dir}/80-can3.link /etc/systemd/network/
sudo cp ${asb_ros2_control_conf_dir}/80-can3-network.rules /etc/udev/rules.d/
sudo cp ${asb_ros2_control_conf_dir}/80-can3.network /etc/systemd/network/

sudo cp ${asb_ros2_control_conf_dir}/80-vcan0.netdev /etc/systemd/network/
sudo cp ${asb_ros2_control_conf_dir}/80-vcan0.network /etc/systemd/network/

sudo cp ${asb_ros2_control_conf_dir}/80-vcan1.netdev /etc/systemd/network/
sudo cp ${asb_ros2_control_conf_dir}/80-vcan1.network /etc/systemd/network/

sudo cp ${asb_ros2_control_conf_dir}/vcan.conf /etc/modules-load.d/

echo "Installed network and udev configuration files"

sudo udevadm control --reload-rules

echo "Reloaded udev rules"
