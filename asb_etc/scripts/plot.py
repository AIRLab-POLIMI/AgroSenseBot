from os import path

from bags_utils import bag_to_dataframe
import matplotlib.pyplot as plt
plt.rcParams['figure.figsize'] = [10, 10]

bag_path = path.expanduser('~/tmp/rosbag2_2024_02_06-14_36_14')

topics_fields_options = {
    '/asb_platform_controller/platform_state': {
        'left_motor_battery_current':
            (['-'], {'color': 'blue', 'linewidth': 1.5}),
        'right_motor_battery_current':
            (['-'], {'color': 'red', 'linewidth': 1.5}),
    },
}

topics_fields = dict(zip(topics_fields_options.keys(), map(lambda i: list(i.keys()), topics_fields_options.values())))
topic_dataframes = bag_to_dataframe(bag_path=bag_path, topics=topics_fields)
for topic_name, d in topic_dataframes.items():
    d.timestamp = d.timestamp - d.timestamp.min()
    d['t'] = d.timestamp * 1E-9
    for field_name in topics_fields_options[topic_name].keys():
        if field_name in d:
            plot_args, plot_kwargs = topics_fields_options[topic_name][field_name]
            plot_kwargs['label'] = f'{topic_name}.{field_name}' if 'label' not in plot_kwargs else plot_kwargs['label']
            plt.plot(d.t, d[field_name], *plot_args, **plot_kwargs)

plt.grid()
plt.legend()
plt.show()
