#! /usr/bin/python3

import os
import signal
import subprocess
import sys

if len(sys.argv) < 2:
    print("Use the same arguments as ros2 bag play.")
    subprocess.call("ros2 bag play --help".split())
    exit()

rosbag_play_args = ' '.join(sys.argv[1:])
if "--read-ahead-queue-size" not in rosbag_play_args:
    rosbag_play_args = "--read-ahead-queue-size 50000 " + rosbag_play_args
rosbag_play_cmd = f"ros2 bag play {rosbag_play_args}"
print(f"running command: {rosbag_play_cmd}\n")

rviz = subprocess.Popen("ros2 launch asb_nav rviz.launch.py", stdout=subprocess.PIPE, shell=True, preexec_fn=os.setsid)
rosbag_play = subprocess.Popen(rosbag_play_cmd, stdout=subprocess.PIPE, shell=True)

try:
    rosbag_play.wait()
    print("ros2 bag play finished")

    rviz.wait()
    print("rviz finished")

except KeyboardInterrupt:
    print("sending SIGTERM")
    os.killpg(os.getpgid(rviz.pid), signal.SIGTERM)  # Send the signal to all the process groups
    rviz.wait()
    print("rviz terminated")

    rosbag_play.kill()
    rosbag_play.wait()
    print("ros2 bag play terminated")
