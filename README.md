# AgroSenseBot

## Install

```bash
sudo apt install software-properties-common libtool python3-wheel
sudo add-apt-repository ppa:lely/ppa -y
sudo apt-get update
sudo apt install liblely-coapp-dev liblely-co-tools python3-dcf-tools
sudo apt install can-utils
mkdir -p ~/w/agrosensebot_ws/src/
cd ~/w/agrosensebot_ws/src/
git clone -b humble https://github.com/ros-industrial/ros2_canopen.git
git clone https://github.com/AIRLab-POLIMI/AgroSenseBot.git
cd ~/w/agrosensebot_ws/
colcon build
```

## Test

⚠️<font style='color:yellow;background-color:black;font-family:monospace'>
DO NOT run this test on the physical CAN network (can0) by changing the parameter `can_interface_name`.
Only use the virtual network vcan0 for testing, as set by default in the launch files.
</font>

In terminal 1:
```bash
sudo ~/w/agrosensebot_ws/src/AgroSenseBot/agrosensebot_canopen_bridge/scripts/setup_vcan0.sh
candump -tz vcan0
```
No messages will be displayed at first.


In terminal 2:
```bash
ros2 launch agrosensebot_dummy_canopen_nodes dummy_canopen_node.launch.py
```
This will launch the GCU node and a dummy node in their own ROS2 namespace (/dummy_test).
Error messages (in red) `GCU COMM TIMEOUT` and `VCU COMM TIMEOUT` will be printed because the nodes are still not exchanging messages.

In terminal 3:
```bash
~/w/agrosensebot_ws/src/AgroSenseBot/agrosensebot_dummy_canopen_nodes/scripts/test.py
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
