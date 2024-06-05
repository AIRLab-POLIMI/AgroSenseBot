#!/usr/bin/python3

import os.path
import sys
import yaml

from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from rosidl_runtime_py import *
from ament_index_python.packages import get_package_share_directory


def result2str(r):
    if r == TaskResult.SUCCEEDED:
        return 'succeeded'
    elif r == TaskResult.CANCELED:
        return 'canceled'
    elif r == TaskResult.FAILED:
        return 'failed'
    else:
        return 'unknown'


def main():

    default_poses_file_path = os.path.expanduser("~/asb_path.yaml")
    poses_file_path = sys.argv[1] if len(sys.argv) > 1 else default_poses_file_path
    print(f"Logging path to {poses_file_path}")

    default_path_id = "unnamed"
    path_id = sys.argv[2] if len(sys.argv) > 2 else default_path_id
    print(f"Approach path id set to {path_id}")

    default_behavior_tree_name = "asb_approach_row_navigate_through_poses.xml"
    behavior_tree_name = sys.argv[3] if len(sys.argv) > 3 else default_behavior_tree_name
    behavior_tree = os.path.join(get_package_share_directory("asb_nav"), "config", "behavior_trees", behavior_tree_name)
    print(f"Behavior tree name set to {behavior_tree_name}")

    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer='robot_localization')

    try:
        # read existing poses
        with open(poses_file_path, 'r') as poses_file:
            paths_dict = yaml.safe_load(poses_file)
    except FileNotFoundError:
        # if the file does not exist
        print(f"Error: file does not exists [{poses_file_path}]")
        return
    except Exception as ex:
        # if other exception, raise the warning
        print(f"Error logging pose: {str(ex)}")
        return

    if path_id not in paths_dict:
        print(f"Error: path does not exists [{path_id}]. Available paths:")
        print('\n'.join(map(lambda x: f"- {x}", paths_dict.keys())))
        return

    if len(paths_dict[path_id]) == 0:
        print(f"Error: empty path [{path_id}]")
        return

    print(f"Path has {len(paths_dict[path_id])} poses")

    goal_poses_stamped = []
    for goal_pose in paths_dict[path_id]:
        goal_pose_stamped = PoseStamped()
        set_message_fields(goal_pose_stamped, goal_pose)
        goal_poses_stamped.append(goal_pose_stamped)

    try:
        navigator.goThroughPoses(goal_poses_stamped, behavior_tree=behavior_tree)
        i = 0
        timeout = Duration(seconds=120.0)
        while not navigator.isTaskComplete():
            i += 1
            feedback = navigator.getFeedback()
            if feedback and i % 100 == 0:
                if Duration.from_msg(feedback.navigation_time) > timeout:
                    navigator.cancelTask()
                    print("Timeout exceeded, cancelling navigation task")

                print(f"ETA:     {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f} s")
                print(f"Timeout: {Duration(seconds=60.0).nanoseconds / 1e9 - (Duration.from_msg(feedback.navigation_time)).nanoseconds / 1e9:.1f} s")

        result = navigator.getResult()
        print(f"Nav result: {result2str(result)}")
    except KeyboardInterrupt:
        print("Cancelling navigation task")
        navigator.cancelTask()

    exit(0)


if __name__ == '__main__':
    main()
