# AgroSenseBot

## Install
After installing ROS2 Humble, run these commands:
```bash
sudo apt install can-utils ros-humble-xacro ros-humble-ros2controlcli ros-humble-ros2-controllers-test-nodes ros-humble-diff-drive-controller ros-humble-joint-state-broadcaster
mkdir -p ~/w/agrosensebot_ws/src/
cd ~/w/agrosensebot_ws/src/
git clone -b humble https://github.com/ros-industrial/ros2_canopen.git
git clone -b humble-dev https://github.com/AIRLab-POLIMI/AgroSenseBot.git
cd ~/w/agrosensebot_ws/
colcon build
```

## asb_ros2_control
The ROS2 package `asb_ros2_control` provides the ros2_control hardware interface which communicates with the CANOpen control system.
The hardware interface is meant to be used with ros2_control controllers, such as the ros2_control diff_drive_controller.


### Test

In separate terminals:

Set up the vcan0 virtual CAN network
```shell
sudo ~/w/agrosensebot_ws/src/AgroSenseBot/agrosensebot_canopen_bridge/scripts/setup_vcan0.sh
candump -tz vcan0
```

Launch ros2_control hardware interface and dummy CANOpen node
```shell
ros2 launch asb_ros2_control_test test.launch.py
```

Run script that provides and receives data to the dummy node
```shell
~/w/agrosensebot_ws/src/AgroSenseBot/asb_ros2_control_test/scripts/test_ros2_control.py
```

Run ROS2 node to publish cmd_vel and press forward key (i).
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/asb_base_controller/cmd_vel_unstamped
```


## agrosensebot_canopen_bridge
The ROS2 package `agrosensebot_canopen_bridge` provides a bridge between the CANOpen control system and ROS2 by translating the CANOpen data.
`agrosensebot_canopen_bridge` and `asb_ros2_control` should not be run at the same time.

### Test 

⚠️<font style='color:yellow;background-color:black;font-family:monospace'>
DO NOT run this test on the physical CAN network (can0) by changing the parameter `can_interface_name`.
Only use the virtual network vcan0 for testing, as set by default in the launch files.
</font>

In terminal 1:
```shell
sudo ~/w/agrosensebot_ws/src/AgroSenseBot/agrosensebot_canopen_bridge/scripts/setup_vcan0.sh
candump -tz vcan0
```
No messages will be displayed at first.


In terminal 2:
```shell
ros2 launch asb_ros2_control_test dummy_canopen_node.launch.py
```
This will launch the GCU node and a dummy node in their own ROS2 namespace (/dummy_test).
Error messages (in red) `GCU COMM TIMEOUT` and `VCU COMM TIMEOUT` will be printed because the nodes are still not exchanging messages.

In terminal 3:
```shell
~/w/agrosensebot_ws/src/AgroSenseBot/asb_ros2_control_test/scripts/test.py
```
This script publishes ROS2 messages to the GCU and dummy nodes, which will communicate through the CAN network simulating the communication on the physical CAN network.
CAN messages should be printed in terminal 1 and the error messages from terminal 2 should stop being printed.
The list of published and received ROS2 test messages will be printed in terminal 3 as follows (the content of the messages is random):
```
...

Publishing gcu_alive 0x0 
Publishing vcu_state (True, 0)
Publishing speed_ref (-15350, 16433)
Publishing motor_drive_right (514.4, 746.5, 6978, -1561.6000000000001)
Publishing motor_drive_left (3107.3, -3061.4, 2643, -577.5)
Publishing motor_drive_fan (1126.2, -1017.5, -11543, 1336.9)
Received   vcu_state (True, 0)
Received   motor_drive_right (514.4, 746.5, 6978, -1561.6000000000001)
Received   motor_drive_left (3107.3, -3061.4, 2643, -577.5)
Received   motor_drive_fan (1126.2, -1017.5, -11543, 1336.9)
Received   speed_ref (-15350, 16433)

...
```
If everything is working correctly, all published messages except gcu_alive should also be received,
and the received messages should be equal to the published messages with the same name, as in the example above.
The equality check is **NOT** performed automatically.

