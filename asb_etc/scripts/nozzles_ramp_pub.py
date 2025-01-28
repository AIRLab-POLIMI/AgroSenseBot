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

        self.nozzle_ids = list()
        self.declare_parameter('nozzles_configuration_file_path', rclpy.Parameter.Type.STRING)
        nozzles_configuration_file_path = os.path.expanduser(self.get_parameter('nozzles_configuration_file_path').get_parameter_value().string_value)
        with open(nozzles_configuration_file_path, 'r') as f:
            nozzles_configuration = yaml.safe_load(f)
        for nozzle_configuration in nozzles_configuration:
            self.nozzle_ids.append(nozzle_configuration['id'])

        publish_rate: float = 100.0

        self.loop_lut: bool = True
        self.loop_pause: float = 2.0
        self.include_unknown_nozzle: bool = True

        lut = {
            0.0: -0.1,
            5.0: 1.1,
            5.1: 0.0,
        }

        self.lut_keys = list(lut.keys())
        self.lut_values = list(lut.values())

        self.t0 = self.get_clock().now()

        self.nozzles_command_pub = self.create_publisher(NozzleCommandArray, '/nozzles_command', qos_profile=qos.qos_profile_sensor_data)
        self.create_timer(1.0/publish_rate, self.timer_callback)

    def timer_callback(self):
        t = (self.get_clock().now() - self.t0).nanoseconds / 1E9

        if self.loop_lut and t > max(self.lut_keys):
            if t > max(self.lut_keys) + self.loop_pause:
                self.t0 = self.get_clock().now()
            else:
                return

        nozzle_rate = np.interp(t, self.lut_keys, self.lut_values)
        self.get_logger().info(f"nozzle_rate: {nozzle_rate:.4f}")

        nozzle_command_msg = NozzleCommandArray(stamp=self.get_clock().now().to_msg())
        for nozzle_id in self.nozzle_ids:
            nozzle_command_msg.nozzle_command_array.append(NozzleCommand(
                nozzle_id=nozzle_id,
                rate=nozzle_rate,
            ))

        if self.include_unknown_nozzle:
            nozzle_command_msg.nozzle_command_array.append(NozzleCommand(
                nozzle_id='test_nozzle_id',
                rate=nozzle_rate,
            ))

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
