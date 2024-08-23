#! /usr/bin/python3

import random

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile

from asb_msgs.msg import PlatformState, EmergencyStopCmd


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('test_node')

        qos = rclpy.qos.qos_profile_sensor_data

        # publishers to GCU
        self.pub = self.create_publisher(
            EmergencyStopCmd,
            '/asb_platform_controller/emergency_stop_cmd',
            qos_profile=qos)

        # subscribers from dummy
        self.platform_state_sub = self.create_subscription(
            PlatformState,
            '/asb_platform_controller/platform_state',
            self.platform_state_callback, qos_profile=qos)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.start_time_s = self.get_clock().now().nanoseconds * 1e-9
        self.software_emergency_stop = None

    def timer_callback(self):
        now_s = self.get_clock().now().nanoseconds * 1e-9
        elapsed_time_s = (now_s - self.start_time_s)
        print(elapsed_time_s)

        # set emergency stop between 2 and 4 seconds from start time
        set_software_emergency_stop = True if 2 < elapsed_time_s < 4 else False

        msg = EmergencyStopCmd(
            stamp=self.get_clock().now().to_msg(),
            set_software_emergency_stop=set_software_emergency_stop)
        self.pub.publish(msg)

        print(f"elapsed_time:                                          {elapsed_time_s} \n"
              f"published set_software_emergency_stop:                 {msg.set_software_emergency_stop} \n"
              f"received platform_state.software_emergency_stop: {self.software_emergency_stop} \n")

    def platform_state_callback(self, msg):
        self.software_emergency_stop = msg.software_emergency_stop


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
