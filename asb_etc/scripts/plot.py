from os import path

import pandas as pd

from bag_to_dataframe import bag_to_dataframe
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = [10, 10]

bag_path = path.expanduser('~/asb_logs/2024-09-23/rosbag2_2024-09-23__09-18-07_all_except_sensors')

plot_relative_time = False

topics_fields_options = {
    # '/asb_platform_controller/platform_state': {
    #     'left_motor_battery_current':
    #         (['-'], {'color': 'blue', 'linewidth': 1.5}),
    #     'right_motor_battery_current':
    #         (['-'], {'color': 'red', 'linewidth': 1.5}),
    # },
    # '/odom': {
    #     'twist.twist.angular.z':
    #         (['-'], {'color': 'green', 'linewidth': 1.5}),
    # },
    # '/cmd_vel': {
    #     'angular.z':
    #         (['-'], {'color': 'cyan', 'linewidth': 1.5}),
    # },
    '/robot_global_path_distance': {
        'data':
            (['-'], {'color': 'black', 'linewidth': 1.5}),
    },
}

topics_fields = dict(zip(topics_fields_options.keys(), map(lambda i: list(map(lambda j: j.split('.'), i.keys())), topics_fields_options.values())))

topic_dataframes = bag_to_dataframe(bag_path=bag_path, topics=topics_fields)
for topic_name, d in topic_dataframes.items():
    if plot_relative_time:
        d.timestamp = d.timestamp - d.timestamp.min()
        d['t'] = d.timestamp * 1E-9
    else:
        d['t'] = pd.to_datetime(d.timestamp * 1E-9, unit='s')

    for field_name in topics_fields_options[topic_name].keys():
        if field_name in d:
            plot_args, plot_kwargs = topics_fields_options[topic_name][field_name]
            plot_kwargs['label'] = f'{topic_name}.{field_name}' if 'label' not in plot_kwargs else plot_kwargs['label']
            plt.plot(d.t, d[field_name], *plot_args, **plot_kwargs)

plt.grid()
plt.legend()
plt.show()
