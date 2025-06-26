# Preparing the test data
Prepare a rosbag with only the point cloud messages (front and rear) with remapped topics.

In terminal 1:
```bash
rb play ~/asb_logs/2025-06-23/d_0_rosbag2_2025-06-23__10-24-32_all/ \
        --remap /scan_front_multilayer/points:=/update_timestamp/scan_front_multilayer/points \
                /scan_rear_multilayer/points:=/update_timestamp/scan_rear_multilayer/points \
        --start-paused --clock
```

In terminal 2, as soon as the time is right, record the test data rosbag with:
```bash
cd ~/asb_logs/test_data
rb record -o scan_multilayer_test_data --regex /update_timestamp/scan_front_multilayer/points\|/update_timestamp/scan_rear_multilayer/points
```


# Benchmarking the nodes

In terminal 1, run the simulation (or any launch file with the navigation stack and lidar filter):
```bash
sim
```

In terminal 2:
```bash
rr asb_logging ros_bag_msg_timestamp_republisher.py
```

In terminal 3, play the test data, start paused:
```bash
rb play ~/asb_logs/test_data/scan_multilayer_test_data  --start-paused
```

In terminal 4:
```bash
rr asb_logging node_computer_system_usage_logger.py
```

After a few seconds: play the bag by pressing space in terminal 3. After around 13 seconds, the average CPU usage of the nodes is saved to `~/asb_logs/test_data/`
