
from tf_transformations import quaternion_from_euler
from geometry_msgs.msg import PoseStamped, Pose, Point, Quaternion
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
import rclpy
from rclpy.duration import Duration
from math import pi


def yaw2q(yaw):
    x, y, z, w = quaternion_from_euler(0, 0, yaw)
    return Quaternion(x=x, y=y, z=z, w=w)


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
    rclpy.init()
    navigator = BasicNavigator()
    navigator.waitUntilNav2Active(localizer='robot_localization')

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

    navigator.goThroughPoses(goal_poses_stamped)
    i = 0
    while not navigator.isTaskComplete():
        i += 1
        feedback = navigator.getFeedback()
        if feedback and i % 5 == 0:
            print(f"ETA: {Duration.from_msg(feedback.estimated_time_remaining).nanoseconds / 1e9:.1f} s")

            if Duration.from_msg(feedback.navigation_time) > Duration(seconds=60.0):
                navigator.cancelTask()

    result = navigator.getResult()
    print(f"Nav result: {result2str(result)}")

    exit(0)


if __name__ == '__main__':
    main()
