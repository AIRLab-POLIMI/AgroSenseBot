# AgroSenseBot

## Install

To install everything needed to run the AgroSenseBot software on the onboard computer or on a development PC, 
[install ROS2 Iron](https://docs.ros.org/en/iron/Installation/Ubuntu-Install-Debians.html) (Ubuntu 22.04) 
and [install Webots](https://cyberbotics.com/doc/guide/installing-webots), then run these commands:

```shell
sudo add-apt-repository -y ppa:lely/ppa &&  # lely PPA repo
sudo apt-get update &&
sudo apt-get install -y liblely-coapp-dev liblely-co-tools python3-dcf-tools &&  # lely canopen
sudo apt-get install -y can-utils &&  # linux can utility
sudo apt-get install -y openssh-server byobu &&  # ssh
sudo apt-get install -y libqwt-qt5-dev &&  # Qwt widgets for the Qt UI
sudo apt-get install -y python3-websocket &&  # websocket python package for asb_sim android_sensors.py
sudo apt-get install -y xterm &&  # used in asb_logging play_bag.launch.py
sudo apt-get install -y ros-iron-xacro ros-iron-ros2controlcli ros-iron-ros2-controllers-test-nodes ros-iron-diff-drive-controller ros-iron-joint-state-broadcaster ros-iron-webots-ros2 ros-iron-rqt-gui ros-iron-rqt-tf-tree ros-iron-nav2-* ros-iron-robot-localization &&  # ROS2 dependencies
sudo apt-get install -y ros-iron-microstrain-inertial-driver ros-iron-microstrain-inertial-rqt ros-iron-ntrip-client &&  # MicroStrain 3DM-GQ7-GNSS/INS and RTK NTRIP client
sudo apt-get install -y ros-iron-octomap-ros &&  # Octomap packages for canopy estimation
pip3 install rosbags &&  # used in asb_logging bags_utils.py
mkdir -p ~/asb_logs/ &&
mkdir -p ~/w/agrosensebot_ws/src/ &&
cd ~/w/agrosensebot_ws/src/ &&
git clone -b iron https://github.com/AIRLab-POLIMI/AgroSenseBot.git &&
cd ~/w/agrosensebot_ws/ &&
colcon build --symlink-install
```

Installing openssh-server and byobu is only necessary on the onboard computer.

To use the MicroStrain 3DM-GQ7-GNSS/INS and RTK NTRIP client it is necessary to create a file at `~/NTRIP_caster_password` 
containing the password used to authenticate with the NTRIP caster of the RTK correction service (such as SPIN3 GNSS).
The password is not stored on this repository since it should not be published.

### Set up the CAN network

Before running the AgroSenseBot ROS2 software, it is necessary to set up the CAN network.
The can0 network is used to communicate with the real hardware, and the vcan0 virtual CAN network is used when running 
tests in the simulator.
To persistently set up the CAN networks can0 and vcan0, run the following command in a terminal.
```shell
ros2 run asb_ros2_control setup_can_networks.sh
```

Otherwise, the CAN networks can be manually set up with the following commands, but will not persist after rebooting the operating system:
```shell
ros2 run asb_ros2_control setup_vcan0.sh  # to set up the vcan0 network, or
ros2 run asb_ros2_control setup_can0.sh  # to set up the can0 network
```
To shut down the CAN networks that have been manually set up (p.e., for testing), use the following commands:
```shell
ros2 run asb_ros2_control shutdown_vcan0.sh  # to shut down the vcan0 network, or
ros2 run asb_ros2_control shutdown_can0.sh  # to shut down the can0 network
```


### Connect to the onboard computer through SSH

On the GUI laptop, run the following command. 
```shell
ssh agrosensebot@asb-onboard.local
byobu
```

The byobu command creates a persistent shell on the onboard computer and additionally acts as a terminal-based window manager.
Using byobu allows the commands to still run while the wireless connection is broken.
Useful byobu keybindings:
```
F2 - Create a new window
F3 - Move to previous window
F4 - Move to next window
F6 - Detach from this session
```
For more documentation run `man byobu` in a terminal and read the manual (press `q` to exit the manual).


## Simulation and System Test

To test the software it's possible to run the AgroSenseBot system in the simulator.
If the CAN network vcan0 have not been set up persistently and have not already been set up manually, set up the vcan0 
virtual CAN network with the following command.
```shell
ros2 run asb_ros2_control setup_vcan0.sh
```
To verify if the network is up, run the following command which prints the network traffic.
```shell
candump -tz vcan0
```
If the network is not up, an error is printed.

Launch the hardware interface and test node with the following command
```shell
ros2 launch asb_sim nav_dual_rtk_no_lidar_sim.launch.py record:=false
```

*Either* run the `teleop_twist_keyboard` node to publish cmd_vel and press the forward key (i), with the following command
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```
*Or*, run the following command to use a joy-pad, note that the key bindings (i.e., the enable_button) are specified in 
the launch file, and will likely need to be set based on the joy-pad model
```shell
ros2 launch asb_nav joy.launch.py
```
*Or*, in RViz, send a navigation goal by clicking on `Nav2 Goal`.

In the Rviz window, the robot model should move according to the keyboard or joy-pad input, or navigate to the goal.

## Packages and Nodes

### asb_ros2_control
Provides the ros2_control hardware interface which communicates with the CANOpen control system.
The hardware interface is meant to be used with ros2_control controllers, such as the ros2_control diff_drive_controller.

### asb_control_system_status_controller
Implements a ros2 controller that publishes the ROS2 topic `/asb_control_system_status_controller/control_system_state`
containing the control system state from the ros2_control hardware interface and subscribes to the following topics
containing commands sent to the ros2_control hardware interface:
```
/asb_control_system_status_controller/emergency_stop_cmd
/asb_control_system_status_controller/fan_cmd
/asb_control_system_status_controller/heartbeat
/asb_control_system_status_controller/pump_cmd
```

TODO finish packages documentation, add ros2 control and ROS2 nodes diagrams
