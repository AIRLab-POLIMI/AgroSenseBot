#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from agrosensebot_canopen_bridge_msgs.msg import MotorDrive


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.i = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            depth=10
        )
        self.publisher_ = self.create_publisher(MotorDrive, 'fake_motor_drive', qos_profile=qos_profile)

        timer_period = 0.1  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        data = self.i % 2**15
        msg = MotorDrive()
        msg.fan_controller_temperature = data
        msg.fan_motor_temperature = data
        msg.fan_motor_rpm = data
        msg.fan_battery_current_display = data
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg}")
        self.i += 1


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
