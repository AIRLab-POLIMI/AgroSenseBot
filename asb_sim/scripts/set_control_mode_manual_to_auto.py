#! /usr/bin/python3

import rclpy
from rclpy.node import Node
from rclpy.time import Duration
from asb_msgs.srv import SetControlMode, SetControlMode_Request


class ControlModePublisher(Node):

    def __init__(self):
        super().__init__('control_mode_publisher')

        self.set_control_mode_service = self.create_client(SetControlMode, '/system_test/set_control_mode')
        self.delay_sec = 1.0

        while not self.set_control_mode_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info(f"waiting {self.set_control_mode_service.srv_name} service")

        set_manual_mode_future = self.set_control_mode_service.call_async(SetControlMode_Request(control_mode=SetControlMode_Request.CONTROL_MODE_RCU))  # set control mode to MANUAL
        set_manual_mode_future.add_done_callback(self.set_manual_mode_response)

    def set_manual_mode_response(self, response_future):
        self.get_logger().info(f"set manual mode response: {response_future.result().result}")

        self.create_timer(self.delay_sec, self.delay_timer_callback)

    def delay_timer_callback(self):
        set_auto_mode_future = self.set_control_mode_service.call_async(SetControlMode_Request(control_mode=SetControlMode_Request.CONTROL_MODE_GCU))  # set control mode to AUTO
        set_auto_mode_future.add_done_callback(self.set_auto_mode_response)

    def set_auto_mode_response(self, response_future):
        self.get_logger().info(f"set auto mode response: {response_future.result().result}")
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = ControlModePublisher()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
