#!/bin/bash

asb_ros2_control_conf_dir=`ros2 pkg prefix asb_ros2_control`/share/asb_ros2_control/config/
sudo cp ${asb_ros2_control_conf_dir}/80-can0.link /etc/systemd/network/
sudo cp ${asb_ros2_control_conf_dir}/80-can0.network /etc/systemd/network/
sudo cp ${asb_ros2_control_conf_dir}/80-vcan0.netdev /etc/systemd/network/
sudo cp ${asb_ros2_control_conf_dir}/80-vcan0.network /etc/systemd/network/
sudo cp ${asb_ros2_control_conf_dir}/vcan.conf /etc/modules-load.d/
