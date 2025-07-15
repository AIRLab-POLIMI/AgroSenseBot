set the start and stop time variables in asb_canopy_spraying_task canopy_estimation_from_bag.py to the bag times at which the estimation should start and stop for each task plan item:
```python
self.start_estimation_time: dict[str, float] = {
    'inter_row_10_c': 1750671233.92,
    'inter_row_10_b': 1750671533.11,
}
self.stop_estimation_time: dict[str, float] = {
    'inter_row_10_c': 1750671343.63,
    'inter_row_10_b': 1750671643.38,
}
```

In terminal 1:
```shell
rb play ~/asb_logs/2025-06-23/d_0_rosbag2_2025-06-23__10-24-32_all/ --clock --start-paused --start-offset 45 --remap /canopy_data_viz/row_10:=/replay/canopy_data_viz/row_10 /canopy_data:=/replay/canopy_data /canopy_visualization_markers:=/replay/canopy_visualization_markers
```

In terminal 2:
```shell
rl asb_canopy_spraying_task canopy_estimation_from_bag.launch.xml
```

In terminal 3:
```shell
rv-ust
```
