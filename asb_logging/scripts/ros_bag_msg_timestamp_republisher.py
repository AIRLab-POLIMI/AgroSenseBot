#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import PointCloud2
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

import copy


class RepublishWithCurrentTime(Node):
    def __init__(self):
        super().__init__('republish_with_current_time')

        self.topics_and_types = [
            ('/scan_front_multilayer/points', PointCloud2),
            ('/scan_rear_multilayer/points', PointCloud2)
        ]

        self.subscribers = []
        self.pubs = {}

        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            depth=10
        )

        for (topic, msg_type) in self.topics_and_types:
            pub = self.create_publisher(msg_type, topic, qos_profile)
            self.pubs[topic] = pub

            # Subscribe to input topic
            sub = self.create_subscription(
                msg_type,
                '/update_timestamp' + topic,
                lambda msg, t=topic: self.callback(msg, t),
                qos_profile
            )

            self.subscribers.append(sub)

    def callback(self, msg, topic_name):
        new_msg = copy.deepcopy(msg)

        if hasattr(new_msg, 'header') and hasattr(new_msg.header, 'stamp'):
            now = self.get_clock().now().to_msg()
            new_msg.header.stamp = now

        self.pubs[topic_name].publish(new_msg)


def main(args=None):
    rclpy.init(args=args)
    node = RepublishWithCurrentTime()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
