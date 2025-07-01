from typing import Dict
from collections import defaultdict

from pathlib import Path
from os import path

from rosbags.highlevel import AnyReader
import pandas as pd


def bag_to_dataframe(bag_path: str, topics: Dict[str, list[list]]) -> Dict[str, pd.DataFrame]:
    topic_field_data = defaultdict(lambda: defaultdict(list))
    topic_timestamps = defaultdict(list)

    with AnyReader([Path(bag_path)]) as reader:
        connections = [x for x in reader.connections if x.topic in topics.keys()]
        for connection, timestamp, rawdata in reader.messages(connections=connections):
            msg = reader.deserialize(rawdata, connection.msgtype)
            topic_timestamps[connection.topic].append(timestamp)
            for field_names in topics[connection.topic]:
                msg_object = msg
                field_string = '.'.join(field_names)
                for i, field_name in enumerate(field_names):
                    if field_name not in msg_object.__dict__:
                        print(f"message of topic {connection.topic} does not have field {i+1} [{field_name}] of {field_string}")
                        break
                    msg_object = msg_object.__getattribute__(field_name)

                if isinstance(msg_object, float) or isinstance(msg_object, int):
                    topic_field_data[connection.topic][field_string].append(msg_object)
                else:
                    print("field value is not float or int")

    dfs = dict()
    for topic_name in topics:
        df = pd.DataFrame({'timestamp': topic_timestamps[topic_name]})
        if not len(topic_timestamps[topic_name]):
            print(f"Invalid topic {topic_name}")
            continue
        for field_names in topics[topic_name]:
            field_string = '.'.join(field_names)
            if len(topic_field_data[topic_name][field_string]):
                df[field_string] = topic_field_data[topic_name][field_string]
            else:
                print(f"Invalid data (not int or float) in field {field_string} of topic {topic_name}")
        dfs[topic_name] = df

    return dfs


def dataframe_to_csv(bag_path: str, topics: Dict[str, list]) -> None:
    for topic_name, df in bag_to_dataframe(bag_path, topics).items():
        topic_file_name = topic_name[1:] if topic_name[0] == '/' else topic_name
        topic_file_name = topic_file_name.replace('/', '__')
        df.to_csv(path.join(bag_path, f"{topic_file_name}.csv"))


def test():
    # test_bag_path = path.expanduser('~/tmp/rosbag2_2024-09-10__11-14-08_all_except_sensors')
    test_bag_path = path.expanduser('~/asb_logs/2024-09-23/rosbag2_2024-09-23__09-18-07_all_except_sensors')
    test_topics = {
        '/asb_platform_controller/platform_state': [
            ['left_motor_battery_current'],
            ['right_motor_battery_current'],
        ],
        '/odom': [
            ['twist', 'twist', 'angular', 'z'],
        ],
        '/cmd_vel': [
            ['angular', 'z'],
        ],
    }
    dataframe_to_csv(test_bag_path, test_topics)


if __name__ == '__main__':
    test()
