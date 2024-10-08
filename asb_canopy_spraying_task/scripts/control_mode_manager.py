#! /usr/bin/python3

from rclpy.node import Node
from asb_msgs.srv import SetControlMode, SetControlMode_Request


class ControlModeManager:

    def __init__(self, node: Node):
        self._node = node
        self._set_control_mode_service = self._node.create_client(SetControlMode, '/system_test/set_control_mode')
        self._set_manual_mode_timeout_timer = None
        self._set_auto_mode_timeout_timer = None

    def set_control_mode_manual(self):
        while not self._set_control_mode_service.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info(f"waiting {self._set_control_mode_service.srv_name} service")

        set_manual_mode_future = self._set_control_mode_service.call_async(SetControlMode_Request(control_mode=SetControlMode_Request.CONTROL_MODE_RCU))  # set control mode to MANUAL
        set_manual_mode_future.add_done_callback(self._set_manual_mode_response)
        self._set_manual_mode_timeout_timer = self._node.create_timer(1.0, self._set_manual_mode_timeout_callback)

    def set_control_mode_auto(self):
        while not self._set_control_mode_service.wait_for_service(timeout_sec=1.0):
            self._node.get_logger().info(f"waiting {self._set_control_mode_service.srv_name} service")

        set_auto_mode_future = self._set_control_mode_service.call_async(SetControlMode_Request(control_mode=SetControlMode_Request.CONTROL_MODE_GCU))  # set control mode to AUTO
        set_auto_mode_future.add_done_callback(self._set_auto_mode_response)
        self._set_auto_mode_timeout_timer = self._node.create_timer(1.0, self._set_auto_mode_timeout_callback)

    def _set_manual_mode_response(self, _):
        self._set_manual_mode_timeout_timer.cancel()
        self._node.get_logger().info(f"mode set to manual")

    def _set_manual_mode_timeout_callback(self):
        self._set_manual_mode_timeout_timer.cancel()
        self._node.get_logger().warn(f"set manual mode service call timeout, retrying.")

        # retry
        set_manual_mode_future = self._set_control_mode_service.call_async(SetControlMode_Request(control_mode=SetControlMode_Request.CONTROL_MODE_RCU))  # set control mode to MANUAL
        set_manual_mode_future.add_done_callback(self._set_manual_mode_response)
        self._set_manual_mode_timeout_timer = self._node.create_timer(1.0, self._set_manual_mode_timeout_callback)

    def _set_auto_mode_response(self, _):
        self._set_auto_mode_timeout_timer.cancel()
        self._node.get_logger().info(f"mode set to auto")

    def _set_auto_mode_timeout_callback(self):
        self._set_auto_mode_timeout_timer.cancel()
        self._node.get_logger().warn(f"set auto mode service call timeout, retrying.")

        # retry
        set_auto_mode_future = self._set_control_mode_service.call_async(SetControlMode_Request(control_mode=SetControlMode_Request.CONTROL_MODE_GCU))  # set control mode to AUTO
        set_auto_mode_future.add_done_callback(self._set_auto_mode_response)
        self._set_auto_mode_timeout_timer = self._node.create_timer(1.0, self._set_auto_mode_timeout_callback)
