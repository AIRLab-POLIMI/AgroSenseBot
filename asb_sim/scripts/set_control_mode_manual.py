#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from asb_msgs.srv import SetControlMode, SetControlMode_Request


class ControlModePublisher(Node):

    def __init__(self):
        super().__init__('control_mode_publisher')

        self.set_control_mode_service = self.create_client(SetControlMode, '/system_test/set_control_mode')

        while not self.set_control_mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"waiting {self.set_control_mode_service.srv_name} service")

        set_manual_mode_future = self.set_control_mode_service.call_async(SetControlMode_Request(control_mode=SetControlMode_Request.CONTROL_MODE_RCU))  # set control mode to MANUAL
        set_manual_mode_future.add_done_callback(self.set_manual_mode_response)
        self.set_manual_mode_timeout_timer = self.create_timer(1.0, self.set_manual_mode_timeout_callback)

    def set_manual_mode_response(self, _):
        self.set_manual_mode_timeout_timer.cancel()
        self.get_logger().info(f"mode set to manual")
        rclpy.shutdown()

    def set_manual_mode_timeout_callback(self):
        self.set_manual_mode_timeout_timer.cancel()
        self.get_logger().error(f"set manual mode service call timeout, retrying.")

        # retry
        set_manual_mode_future = self.set_control_mode_service.call_async(SetControlMode_Request(control_mode=SetControlMode_Request.CONTROL_MODE_RCU))  # set control mode to MANUAL
        set_manual_mode_future.add_done_callback(self.set_manual_mode_response)
        self.set_manual_mode_timeout_timer = self.create_timer(1.0, self.set_manual_mode_timeout_callback)


def main(args=None):
    rclpy.init(args=args)
    node = ControlModePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
