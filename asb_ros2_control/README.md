# asb_ros2_control

# Additional dependencies:
```
ros-humble-xacro ros-humble-rviz2 ros-humble-ros2controlcli ros-humble-ros2-controllers-test-nodes ros-humble-ros2-control-demo-description ros-humble-diff-drive-controller ros-humble-joint-state-broadcaster
```

# Testing:

launch ros2_control hardware interface and dummy CANOpen node
```shell
ros2 launch agrosensebot_dummy_canopen_nodes dummy_canopen_node_ros2_control.launch.py
```

run script that provides and receives data to the dummy node
```shell
~/w/agrosensebot_ws/src/AgroSenseBot/agrosensebot_dummy_canopen_nodes/scripts/test_ros2_control.py
```

run ROS2 node to publish cmd_vel
```shell
ros2 run teleop_twist_keyboard teleop_twist_keyboard --ros-args -r /cmd_vel:=/asb_base_controller/cmd_vel_unstamped
```
