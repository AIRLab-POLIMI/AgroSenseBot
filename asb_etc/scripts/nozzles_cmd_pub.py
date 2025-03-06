#!/usr/bin/python3
import os
import sys
import yaml

import numpy as np
import rclpy
from asb_msgs.msg import NozzleCommandArray, NozzleCommand
from rclpy import qos
from rclpy.node import Node

from numpy import pi
from geometry_msgs.msg import Twist, Vector3


class LUTNozzlesPublisher(Node):

    def __init__(self):
        super().__init__('lut_nozzles_publisher')

        publish_rate: float = 100.0

        self.nozzles_command_pub = self.create_publisher(NozzleCommandArray, '/nozzles_command', qos_profile=qos.qos_profile_sensor_data)
        self.create_timer(1.0/publish_rate, self.timer_callback)

    def timer_callback(self):
        nozzle_rate = 1.0
        nozzle_id = "1R"
        self.get_logger().info(f"nozzle: {nozzle_id}  rate: {nozzle_rate:.4f}")

        nozzle_command_msg = NozzleCommandArray(
            stamp=self.get_clock().now().to_msg(),
            nozzle_command_array=[
                NozzleCommand(
                    nozzle_id=nozzle_id,
                    rate=nozzle_rate,
                ),
            ],
        )

        self.nozzles_command_pub.publish(nozzle_command_msg)


def main(args=None):
    rclpy.init(args=args)
    node = LUTNozzlesPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
