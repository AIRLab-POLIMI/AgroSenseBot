#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int16


class ControlModePublisher(Node):

    def __init__(self):
        super().__init__('control_mode_publisher')

        self.test_control_mode_pub = self.create_publisher(Int16, '/system_test/control_mode', 10)

        timer_period = 0.01  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

        self.count = 0

    def timer_callback(self):

        if self.count < 100:
            self.test_control_mode_pub.publish(Int16(data=1))  # publish MANUAL
        elif self.count < 200:
            self.test_control_mode_pub.publish(Int16(data=2))  # publish AUTO
        else:
            rclpy.shutdown()

        self.count += 1


def main(args=None):
    rclpy.init(args=args)
    node = ControlModePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
