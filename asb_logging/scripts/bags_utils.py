from typing import Dict
from collections import defaultdict

from pathlib import Path
from os import path

from rosbags.highlevel import AnyReader
import pandas as pd


def bag_to_dataframe(bag_path: str, topics: Dict[str, list]) -> Dict[str, pd.DataFrame]:
    topic_field_data = defaultdict(lambda: defaultdict(list))
    topic_timestamps = defaultdict(list)

    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic in topics.keys()]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            topic_timestamps[connection.topic].append(timestamp)
            for field_name in topics[connection.topic]:
                field_value = msg.__getattribute__(field_name)
                if isinstance(field_value, float) or isinstance(field_value, int):
                    topic_field_data[connection.topic][field_name].append(field_value)

    dfs = dict()
    for topic_name in topics:
        df = pd.DataFrame({'timestamp': topic_timestamps[topic_name]})
        if not len(topic_timestamps[topic_name]):
            print(f"Invalid topic {topic_name}")
            continue
        for field_name in topics[topic_name]:
            if len(topic_field_data[topic_name][field_name]):
                df[field_name] = topic_field_data[topic_name][field_name]
            else:
                print(f"Invalid data (not int or float) in field {field_name} of topic {topic_name}")
        dfs[topic_name] = df

    return dfs


def dataframe_to_csv(bag_path: str, topics: Dict[str, list]) -> None:
    for topic_name, df in bag_to_dataframe(bag_path, topics).items():
        topic_file_name = topic_name[1:] if topic_name[0] == '/' else topic_name
        topic_file_name = topic_file_name.replace('/', '__')
        df.to_csv(path.join(bag_path, f"{topic_file_name}.csv"))


def test():
    test_bag_path = path.expanduser('~/tmp/rosbag2_2024_02_06-14_36_14')
    test_topics = {
        '/not_a_topic': [
            'not_a_field',
        ],
        '/scan_rear_fake': [
            'ranges',
            'angle_min',
        ],
        '/asb_control_system_status_controller/control_system_state': [
            'left_motor_battery_current',
            'right_motor_battery_current',
        ],
    }
    dataframe_to_csv(test_bag_path, test_topics)


if __name__ == '__main__':
    test()
