#!/usr/bin/python3
import math

import rclpy
from rclpy.node import Node

from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from tf_transformations import euler_from_quaternion

from rosidl_runtime_py import *

import tf2_geometry_msgs  # imports PoseStamped into tf2
from geometry_msgs.msg import PoseStamped
from rclpy.duration import Duration

import os
import sys
import yaml


def distance_greater_than_threshold(p1: PoseStamped, p2: PoseStamped, th: float):
    return (p1.pose.position.x - p2.pose.position.x) ** 2 + (p1.pose.position.y - p2.pose.position.y) ** 2 > th ** 2


class AsbPathLogger(Node):

    def __init__(self):
        super().__init__('asb_path_logger')

        self.map_frame = 'map'
        self.base_frame = 'base_footprint'

        default_poses_file_path = os.path.expanduser("~/asb_path.yaml")
        self.poses_file_path = sys.argv[1] if len(sys.argv) > 1 else default_poses_file_path
        self.get_logger().info(f"Logging path to {self.poses_file_path}")

        default_path_id = "unnamed"
        self.path_id = sys.argv[2] if len(sys.argv) > 2 else default_path_id
        self.get_logger().info(f"Path id set to {self.path_id}")

        default_dist_th = 2.0
        self.dist_th = float(sys.argv[3]) if len(sys.argv) > 3 else default_dist_th
        self.get_logger().info(f"Distance threshold set to {self.dist_th}")

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        self.last_robot_pose = None

        self.timer = self.create_timer(1, self.timer_callback)

    def timer_callback(self):

        # robot pose, untransformed (in base frame)
        current_robot_pose = PoseStamped()
        current_robot_pose.header.frame_id = self.base_frame
        current_robot_pose.pose.orientation.w = 1

        # transform the robot pose into the map frame
        try:
            current_robot_pose = self.tf_buffer.transform(current_robot_pose, self.map_frame, timeout=Duration(seconds=0.1))
        except TransformException as ex:
            self.get_logger().info(f'Could not transform {self.base_frame} to {self.map_frame}: {ex}')
            return

        if self.last_robot_pose is None:
            # save the first robot pose
            self.last_robot_pose = current_robot_pose
            self.log_pose(current_robot_pose)
        elif distance_greater_than_threshold(current_robot_pose, self.last_robot_pose, self.dist_th):
            # if the last saved robot pose is farther from the last logged pose than the threshold, log it
            self.last_robot_pose = current_robot_pose
            self.log_pose(current_robot_pose)

    def log_pose(self, p):

        q = p.pose.orientation
        _, _, yaw = euler_from_quaternion([q.x, q.y, q.z, q.w])
        self.get_logger().info(f"Logging pose: x={p.pose.position.x}, y={p.pose.position.y}, yaw={yaw}")

        try:
            # read existing poses
            with open(self.poses_file_path, 'r') as poses_file:
                poses = yaml.safe_load(poses_file)
        except FileNotFoundError:
            # if the file does not exist, use an empty dict
            poses = {self.path_id: []}
        except Exception as ex:
            # if other exception, raise the warning
            self.get_logger().error(f"Error logging pose: {str(ex)}")
            return

        # insert the new path id if it does not exist
        if self.path_id not in poses:
            poses[self.path_id] = list()

        yaml_msg = yaml.safe_load(message_to_yaml(p))
        poses[self.path_id].append(yaml_msg)

        # write updated poses
        try:
            with open(self.poses_file_path, 'w') as poses_file:
                yaml.dump(poses, poses_file, default_flow_style=False)
        except Exception as ex:
            self.get_logger().error(f"Error logging pose: {str(ex)}")
            return


def main():
    rclpy.init()
    node = AsbPathLogger()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass


if __name__ == '__main__':
    main()
