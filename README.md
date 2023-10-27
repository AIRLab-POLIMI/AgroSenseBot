# AgroSenseBot

## Install
After [installing ROS2 Humble](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html) (Ubuntu 22.04), run these commands:
```shell
sudo add-apt-repository -y ppa:lely/ppa  # lely PPA repo
sudo apt-get update
sudo apt-get install -y liblely-coapp-dev liblely-co-tools python3-dcf-tools  # lely canopen
sudo apt-get install -y can-utils  # linux can utility
sudo apt-get install -y ros-humble-xacro ros-humble-ros2controlcli ros-humble-ros2-controllers-test-nodes ros-humble-diff-drive-controller ros-humble-joint-state-broadcaster  # ROS2 dependencies
sudo apt-get install -y libqwt-qt5-dev  # Qwt widgets for the Qt UI
mkdir -p ~/w/agrosensebot_ws/src/
cd ~/w/agrosensebot_ws/src/
git clone -b humble https://github.com/AIRLab-POLIMI/AgroSenseBot.git
cd ~/w/agrosensebot_ws/
colcon build --symlink-install
```

## asb_ros2_control
The ROS2 package `asb_ros2_control` provides the ros2_control hardware interface which communicates with the CANOpen control system.
The hardware interface is meant to be used with ros2_control controllers, such as the ros2_control diff_drive_controller.


### Test

In separate terminals:

Set up the vcan0 virtual CAN network
```shell
ros2 run asb_ros2_control setup_vcan0.sh
candump -tz vcan0
```

Launch the hardware interface and test node with the following command
```shell
ros2 launch asb_ros2_control_test test.launch.py gui:=true
```

*Either* run the `teleop_twist_keyboard` node to publish cmd_vel and press the forward key (i), with the following command
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/asb_base_controller/cmd_vel_unstamped
```
*Or*, run the following command to use a joy-pad, note that the key bindings (i.e., the enable_button) are specified in the launch file, and will likely need to be set based on the joy-pad model
```shell
ros2 launch asb_ros2_control joy.launch.py
```

In the Rviz window, the odom frame should move according to the keyboard or joy-pad input.

To shut down the vcan0 CAN network, use the following command
```shell
ros2 run asb_ros2_control shutdown_vcan0.sh
```