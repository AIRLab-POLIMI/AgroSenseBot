#! /usr/bin/python3

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from asb_msgs.msg import PlatformState, PumpCmd


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test_node')

        qos = rclpy.qos.qos_profile_sensor_data

        # publishers to GCU
        self.pub = self.create_publisher(
            PumpCmd,
            '/asb_platform_controller/pump_cmd',
            qos_profile=qos)

        # subscribers from dummy
        self.platform_state_sub = self.create_subscription(
            PlatformState,
            '/asb_platform_controller/platform_state',
            self.platform_state_callback, qos_profile=qos)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9
        self.pump_state = None

    def timer_callback(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time_s = (now_s - self.start_time_s)
        print(f"elapsed_time:                             {elapsed_time_s}")

        # set pump_cmd to True after 5 seconds from start time
        pump_cmd = True if 5 < elapsed_time_s else False

        # after 10 seconds, stop publishing
        if elapsed_time_s < 10:
            msg = PumpCmd(
                stamp=self.get_clock().now().to_msg(),
                pump_cmd=pump_cmd)
            self.pub.publish(msg)

            print(f"published pump_cmd:                       {msg.pump_cmd}")
        else:
            print(f"published pump_cmd:                       ")

        print(f"received platform_state.pump_state: {self.pump_state} \n")

    def platform_state_callback(self, msg):
        self.pump_state = msg.pump_state


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
