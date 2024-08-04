#! /usr/bin/python3

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from asb_msgs.msg import Heartbeat


class HeartbeatPublisher(Node):

    def __init__(self):
        super().__init__('heartbeat_publisher')

        qos = rclpy.qos.qos_profile_sensor_data

        # publishers to GCU
        self.pub = self.create_publisher(
            Heartbeat,
            '/asb_control_system_status_controller/heartbeat',
            qos_profile=qos)

        timer_period = 0.025  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.alive_bit = False

    def timer_callback(self):
        self.alive_bit = not self.alive_bit
        self.pub.publish(Heartbeat(
            stamp=self.get_clock().now().to_msg(),
            alive_bit=self.alive_bit))


def main(args=None):
    rclpy.init(args=args)
    node = HeartbeatPublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
