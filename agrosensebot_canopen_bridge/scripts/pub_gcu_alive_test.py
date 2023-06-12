#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
from std_msgs.msg import UInt8


class MinimalPublisher(Node):

    def __init__(self):
        super().__init__('minimal_publisher')

        self.i = 0

        qos_profile = QoSProfile(
            reliability=QoSReliabilityPolicy.RMW_QOS_POLICY_RELIABILITY_RELIABLE,
            durability=QoSDurabilityPolicy.RMW_QOS_POLICY_DURABILITY_TRANSIENT_LOCAL,
            depth=10
        )
        self.publisher_ = self.create_publisher(UInt8, 'gcu_alive', qos_profile=qos_profile)

        timer_period = 0.05  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)

    def timer_callback(self):
        """
        Bit 0 of TPDO1 of the GCU has to change between 0 and 1 with a period between 20ms and 100ms.
        This callback is called every 50ms and publishes a ROS2 message with the first bit flipped from the previous
        call.
        """
        msg = UInt8()
        msg.data = self.i % 2
        self.publisher_.publish(msg)
        self.get_logger().info(f"Publishing: {msg.data:#x}")
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
