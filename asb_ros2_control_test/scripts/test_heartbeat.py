#! /usr/bin/python3

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from asb_msgs.msg import Heartbeat


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test_node')

        qos = rclpy.qos.qos_profile_sensor_data

        # publishers to GCU
        self.pub = self.create_publisher(
            Heartbeat,
            '/asb_control_system_status_controller/heartbeat',
            qos_profile=qos)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9
        self.alive_bit = False

    def timer_callback(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time_s = (now_s - self.start_time_s) % 12.0
        print(f"elapsed_time:                             {elapsed_time_s}")

        # increase velocity_rpm from -100 to 2500, at 400 RPM/s, from 2 seconds after start time
        self.alive_bit = not self.alive_bit

        msg = Heartbeat(
            stamp=self.get_clock().now().to_msg(),
            alive_bit=self.alive_bit)
        self.pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = MinimalPublisher()

    rclpy.spin(minimal_publisher)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    minimal_publisher.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
