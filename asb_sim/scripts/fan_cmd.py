#! /usr/bin/python3

import random
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from asb_msgs.msg import PlatformState, FanCmd


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test_node')

        qos = rclpy.qos.qos_profile_sensor_data

        # publishers to GCU
        self.pub = self.create_publisher(
            FanCmd,
            '/asb_platform_controller/fan_cmd',
            qos_profile=qos)

        # subscribers from dummy
        self.platform_state_sub = self.create_subscription(
            PlatformState,
            '/asb_platform_controller/platform_state',
            self.platform_state_callback, qos_profile=qos)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.fan_motor_position = None
        self.fan_motor_velocity_rpm = None

    def timer_callback(self):
        velocity_rpm = int(sys.argv[1])
        msg = FanCmd(
            stamp=self.get_clock().now().to_msg(),
            velocity_rpm=velocity_rpm
        )
        self.pub.publish(msg)

        print(f"published velocity_rpm:                         {msg.velocity_rpm}")
        print(f"received platform_state.fan_motor_velocity_rpm: {self.fan_motor_velocity_rpm}")

    def platform_state_callback(self, msg):
        self.fan_motor_position = msg.fan_motor_position
        self.fan_motor_velocity_rpm = msg.fan_motor_velocity_rpm


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
