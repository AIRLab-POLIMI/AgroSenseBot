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

        self.declare_parameter('do_sequence', rclpy.Parameter.Type.BOOL)
        self.do_sequence: bool = self.get_parameter('do_sequence').get_parameter_value().bool_value

        self.nozzle_ids = list()
        self.declare_parameter('nozzles_configuration_file_path', rclpy.Parameter.Type.STRING)
        nozzles_configuration_file_path: str = os.path.expanduser(self.get_parameter('nozzles_configuration_file_path').get_parameter_value().string_value)
        with open(nozzles_configuration_file_path, 'r') as f:
            nozzles_configuration = yaml.safe_load(f)
        for nozzle_configuration in nozzles_configuration:
            self.nozzle_ids.append(nozzle_configuration['id'])

        publish_rate: float = 100.0

        self.declare_parameter('loop_lut', rclpy.Parameter.Type.BOOL)
        self.loop_lut: bool = self.get_parameter('loop_lut').get_parameter_value().bool_value
        self.loop_pause: float = 2.0
        self.include_unknown_nozzle: bool = False

        # lut = {
        #     0.0: 0.0,
        #     5.0: 1.0,
        #     10.0: 1.0,
        #     15.0: 0.0,
        # }

        x = 1.0  # peak open cmd [0...1]
        t0 = 5.0  # pre-peak closed duration [s]
        t = 10.0  # open duration [s]
        t1 = 5.0  # post peak closed duration [s]
        lut = {
            0.0: 0.0,
            t0-0.01: 0.0,
            t0: x,
            t0+t: x,
            t0+t+0.01: 0.0,
            t0+t+t1: 0.0,
        }

        self.lut_keys = list(lut.keys())
        self.lut_values = list(lut.values())

        self.t0 = self.get_clock().now()

        self.nozzles_command_pub = self.create_publisher(NozzleCommandArray, '/nozzles_command', qos_profile=qos.qos_profile_sensor_data)
        self.create_timer(1.0/publish_rate, self.timer_callback)

    def timer_callback(self):

        t_max = max(self.lut_keys)

        if self.do_sequence:
            t = (self.get_clock().now() - self.t0).nanoseconds / 1E9
            t_m = t % t_max
            i = int(t/t_max)

            if i < len(self.nozzle_ids):
                nozzle_rate = np.interp(t_m, self.lut_keys, self.lut_values)
                self.get_logger().info(f"sequence index: {i}, t: {t:.1f}, t_m: {t_m:.1f}     nozzle: {self.nozzle_ids[i]}  rate: {nozzle_rate:.4f}")

                nozzle_command_msg = NozzleCommandArray(
                    stamp=self.get_clock().now().to_msg(),
                    nozzle_command_array=[
                        NozzleCommand(
                            nozzle_id=self.nozzle_ids[i],
                            rate=nozzle_rate,
                        ),
                    ],
                )

                self.nozzles_command_pub.publish(nozzle_command_msg)
            else:
                if self.loop_lut:
                    if t < t_max * len(self.nozzle_ids) + self.loop_pause:
                        self.get_logger().info(f"waiting to restart without publishing")
                        return
                    else:
                        self.t0 = self.get_clock().now()
                        return  # restart
                else:
                    return  # finished

        else:

            t = (self.get_clock().now() - self.t0).nanoseconds / 1E9

            if t > t_max:
                if self.loop_lut and t > t_max + self.loop_pause:
                    self.t0 = self.get_clock().now()
                else:
                    return

            self.get_logger().info(f"\n**** NOZZLE RATES ****")

            nozzle_command_msg = NozzleCommandArray(stamp=self.get_clock().now().to_msg())
            for nozzle_id in self.nozzle_ids:
                nozzle_rate = np.interp(t, self.lut_keys, self.lut_values)

                # if nozzle_id == "1R":
                #     nozzle_rate = np.interp(t, self.lut_keys, self.lut_values)
                # else:
                #     nozzle_rate = 0.5

                self.get_logger().info(f"{nozzle_id}: {nozzle_rate:.4f}")

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
