#! /usr/bin/env python3
# Copyright 2021 Samsung Research America
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from math import pi


def yaw2q(yaw):
    x, y, z, w = quaternion_from_euler(0, 0, yaw)
    return Quaternion(x=x, y=y, z=z, w=w)


def main():
    rclpy.init()

    navigator = BasicNavigator()

    # Set our demo's initial pose
    # initial_pose = PoseStamped()
    # initial_pose.header.frame_id = 'map'
    # initial_pose.header.stamp = navigator.get_clock().now().to_msg()
    # initial_pose.pose.position.x = 3.45
    # initial_pose.pose.position.y = 2.15
    # initial_pose.pose.orientation.z = 1.0
    # initial_pose.pose.orientation.w = 0.0
    # navigator.setInitialPose(initial_pose)

    # Activate navigation, if not autostarted. This should be called after setInitialPose()
    # or this will initialize at the origin of the map and update the costmap with bogus readings.
    # If autostart, you should `waitUntilNav2Active()` instead.
    # navigator.lifecycleStartup()

    # Wait for navigation to fully activate, since autostarting nav2
    navigator.waitUntilNav2Active(localizer='robot_localization')

    # If desired, you can change or load the map as well
    # navigator.changeMap('/path/to/map.yaml')

    # You may use the navigator to clear or obtain costmaps
    # navigator.clearAllCostmaps()  # also have clearLocalCostmap() and clearGlobalCostmap()
    # global_costmap = navigator.getGlobalCostmap()
    # local_costmap = navigator.getLocalCostmap()

    goal_poses = [
        Pose(position=Point(x=3.0, y=-3.0, z=0.0), orientation=yaw2q(-pi/2)),
        Pose(position=Point(x=0.0, y=-6.0, z=0.0), orientation=yaw2q(pi)),
        Pose(position=Point(x=-3.0, y=-3.0, z=0.0), orientation=yaw2q(pi/2)),
        Pose(position=Point(x=-1.0, y=0.0, z=0.0), orientation=yaw2q(0.0)),
    ]

    goal_poses_stamped = []
    for goal_pose in goal_poses:
        goal_pose_stamped = PoseStamped()
        goal_pose_stamped.header.frame_id = 'map'
        goal_pose_stamped.header.stamp = navigator.get_clock().now().to_msg()
        goal_pose_stamped.pose = goal_pose
        goal_poses_stamped.append(goal_pose_stamped)

    # sanity check a valid path exists
    # path = navigator.getPathThroughPoses(initial_pose, goal_poses)

    navigator.goThroughPoses(goal_poses_stamped)

    i = 0
    while not navigator.isTaskComplete():
        ################################################
        #
        # Implement some code here for your application!
        #
        ################################################

        # Do something with the feedback
        i = i + 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(
                'Estimated time of arrival: '
                + '{0:.0f}'.format(
                    Duration.from_msg(feedback.estimated_time_remaining).nanoseconds
                    / 1e9
                )
                + ' seconds.'
            )

            # Some navigation timeout to demo cancellation
            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                navigator.cancelTask()

    # Do something depending on the return code
    result = navigator.getResult()
    if result == TaskResult.SUCCEEDED:
        print('Goal succeeded!')
    elif result == TaskResult.CANCELED:
        print('Goal was canceled!')
    elif result == TaskResult.FAILED:
        print('Goal failed!')
    else:
        print('Goal has an invalid return status!')

    exit(0)


if __name__ == '__main__':
    main()
